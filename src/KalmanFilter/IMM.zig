const IMM = @This();
const std = @import("std");
const KalmanFilter = @import("KalmanFilter.zig");
const mat = @import("Matrix.zig");

const Float = @import("root").config.Float;
const Matrix = mat.Matrix(Float);
const verify = Matrix.verify;

const filter_len = @import("root").config.num_filters;

pub const Estimate = struct {
    state: Matrix,
    covariance: Matrix,
};

smp: std.mem.Allocator,
filters: []KalmanFilter,
mu: Matrix, // filter probabilities
transition: Matrix, //transition matrix
N: usize,

likelihood: Matrix,
cbar: Matrix,
omega: Matrix,
prediction_pending: bool,

x_prior: Matrix,
cov_prior: Matrix,

x_posterior: Matrix,
cov_posterior: Matrix,

x: Matrix,
cov: Matrix,


/// Takes ownership of filter contents, mu, and transition_matrix on success.
/// Borrows filter storage, which must outlive IMM; each matrix retains its own allocator.
/// On failure, the caller remains responsible for all supplied resources.
pub fn init(smp: std.mem.Allocator, filters: []KalmanFilter, mu: *Matrix, transition_matrix: *Matrix, dim: usize) !IMM{
    std.debug.assert(filters.len <= filter_len);
    const num_filters = filters.len;
    if (num_filters == 0 or dim == 0 or dim > std.math.maxInt(c_int)) return error.InvalidDimensions;
    if (mu.rows <= 0 or mu.cols != 1 or @as(usize, @intCast(mu.rows)) != num_filters) {
        return error.InvalidModeProbabilityDimensions;
    }
    if (transition_matrix.rows != mu.rows or transition_matrix.cols != mu.rows) {
        return error.InvalidTransitionMatrixDimensions;
    }

    std.debug.assert(filters.len == mu.data.len);

    var probability_sum: Float = 0;
    for (mu.data) |probability| {
        if (!std.math.isFinite(probability) or probability < 0) return error.InvalidModeProbabilities;
        probability_sum += probability;
    }
    if (!std.math.isFinite(probability_sum) or probability_sum <= 0) return error.InvalidModeProbabilities;

    var normalized_mu = try Matrix.empty(smp, mu.rows, mu.cols, mu.order);
    errdefer normalized_mu.destroy();
    for (normalized_mu.data, mu.data) |*probability, input_probability| {
        probability.* = input_probability / probability_sum;
    }

    var cbar = try Matrix.empty(smp, transition_matrix.rows, normalized_mu.cols, .row_major);
    errdefer cbar.destroy();

    var likelihood = try Matrix.empty(smp, num_filters, 1, .row_major);
    errdefer likelihood.destroy();
    var omega = try Matrix.empty(smp, num_filters, num_filters, .row_major);
    errdefer omega.destroy();

    var x_prior = try Matrix.empty(smp, dim, 1, .row_major);
    errdefer x_prior.destroy();
    var cov_prior = try Matrix.empty(smp, dim, dim, .row_major);
    errdefer cov_prior.destroy();
    var x_posterior = try Matrix.empty(smp, dim, 1, .row_major);
    errdefer x_posterior.destroy();
    var cov_posterior = try Matrix.empty(smp, dim, dim, .row_major);
    errdefer cov_posterior.destroy();
    var x = try Matrix.empty(smp, dim, 1, .row_major);
    errdefer x.destroy();
    var cov = try Matrix.empty(smp, dim, dim, .row_major);
    errdefer cov.destroy();



    var imm: IMM =  .{
        .smp = smp,
        .filters = filters,
        .mu = normalized_mu,
        .transition = transition_matrix.*,
        .N = num_filters,
        .likelihood = likelihood,
        .cbar = cbar,
        .omega = omega,
        .prediction_pending = false,
        .x_prior = x_prior,
        .cov_prior = cov_prior,
        .x_posterior = x_posterior,
        .cov_posterior = cov_posterior,
        .x = x,
        .cov = cov,
    };

    try imm.compute_mixing_probabilities();
    // Transfer the original probability buffer only after initialization succeeds.
    @memcpy(mu.data, normalized_mu.data);
    normalized_mu.destroy();
    imm.mu = mu.*;
    return imm;
}

pub fn deinit(self: *IMM) void {
    for (self.filters) |*filter| {
        filter.deinit();
    }
    self.mu.destroy();
    self.transition.destroy();
    self.likelihood.destroy();
    self.cbar.destroy();
    self.omega.destroy();
    self.x_prior.destroy();
    self.cov_prior.destroy();
    self.x_posterior.destroy();
    self.cov_posterior.destroy();
    self.x.destroy();
    self.cov.destroy();
    self.* = undefined;
}


pub fn get_prior_estimate(self: *IMM) !Estimate{
    var state = try self.x_prior.copy();
    errdefer state.destroy();
    return .{
        .state = state,
        .covariance = try self.cov_prior.copy()
    };
}

/// Returns owned copies of the posterior state and covariance.
pub fn update(self: *IMM, measurement: *Matrix) !Estimate {
    for (self.filters, 0..) |*filter, i| {
        try filter.update(measurement);
        self.likelihood.data[i] = try filter.compute_likelihood();
    }

    var probability_sum: Float = 0;
    for (self.mu.data, self.cbar.data, self.likelihood.data) |*probability, predicted_probability, likelihood| {
        probability.* = predicted_probability * likelihood;
        probability_sum += probability.*;
    }
    if (!std.math.isFinite(probability_sum) or probability_sum <= 0) return error.InvalidModeProbabilities;
    for (self.mu.data) |*probability| {
        probability.* /= probability_sum;
    }

    self.prediction_pending = false;
    try self.compute_mixing_probabilities();
    try self.compute_state_estimate(&self.mu, false);
    @memcpy(self.x_posterior.data, self.x.data);
    @memcpy(self.cov_posterior.data, self.cov.data);

    var state = try self.x_posterior.copy();
    errdefer state.destroy();
    return .{ .state = state, .covariance = try self.cov_posterior.copy() };
}

/// Stores the prediction in x_prior and cov_prior without returning copies.
pub fn predict(self: *IMM) !void {
    const use_prior = self.prediction_pending;
    if (use_prior) {
        @memcpy(self.mu.data, self.cbar.data);
        try self.compute_mixing_probabilities();
    }

    var mixed_estimates_buffer: [filter_len]Estimate = undefined;
    const mixed_estimates = mixed_estimates_buffer[0..self.N];
    var initialized_count: usize = 0;
    defer {
        for (mixed_estimates[0..initialized_count]) |*estimate| {
            estimate.state.destroy();
            estimate.covariance.destroy();
        }
    }

    // Finish mixing every model before any filter's prior is overwritten.
    for (mixed_estimates, 0..) |*estimate, j| {
        var state = try self.x.copy();
        errdefer state.destroy();
        @memset(state.data, 0);
        var covariance = try self.cov.copy();
        errdefer covariance.destroy();
        @memset(covariance.data, 0);

        for (self.filters, 0..) |*filter, i| {
            const source_state = if (use_prior) &filter.x_prior else &filter.x_posterior;
            const weight = self.omega.data[i * self.N + j];
            for (state.data, source_state.data) |*value, source_value| {
                value.* += weight * source_value;
            }
        }
        for (self.filters, 0..) |*filter, i| {
            const source_state = if (use_prior) &filter.x_prior else &filter.x_posterior;
            const source_covariance = if (use_prior) &filter.cov_prior else &filter.cov_posterior;
            const weight = self.omega.data[i * self.N + j];
            var residual = try verify(try source_state.subtract(&state, null));
            defer residual.destroy();
            var outer_product = try auto_outer_product(&residual);
            defer outer_product.destroy();
            var covariance_term = try verify(try outer_product.add(source_covariance, null));
            defer covariance_term.destroy();
            for (covariance.data, covariance_term.data) |*value, source_value| {
                value.* += weight * source_value;
            }
        }
        estimate.* = .{ .state = state, .covariance = covariance };
        initialized_count += 1;
    }

    for (self.filters, mixed_estimates) |*filter, *estimate| {
        // KalmanFilter.predict reads posterior buffers; restore them afterward.
        std.mem.swap(Matrix, &filter.x_posterior, &estimate.state);
        defer std.mem.swap(Matrix, &filter.x_posterior, &estimate.state);
        std.mem.swap(Matrix, &filter.cov_posterior, &estimate.covariance);
        defer std.mem.swap(Matrix, &filter.cov_posterior, &estimate.covariance);
        try filter.predict();
    }

    try self.compute_state_estimate(&self.cbar, true);
    @memcpy(self.x_prior.data, self.x.data);
    @memcpy(self.cov_prior.data, self.cov.data);
    self.prediction_pending = true;
}

fn compute_mixing_probabilities(self: *IMM) !void{
    var transition_t = try verify(try self.transition.transpose(true));
    defer transition_t.destroy();
    var predicted_probabilities = try verify(try transition_t.multiply(&self.mu, null));
    defer predicted_probabilities.destroy();
    @memcpy(self.cbar.data, predicted_probabilities.data);

    for(0..self.N)|i|{
        for(0..self.N)|j|{
            const idx = i * self.N + j;
            const transition_idx = if (self.transition.order == .row_major) idx else j * self.N + i;
            self.omega.data[idx] = if (self.cbar.data[j] == 0) 0 else self.transition.data[transition_idx] * self.mu.data[i] / self.cbar.data[j];
        }
    }
}

fn compute_state_estimate(self: *IMM, mode_probabilities: *Matrix, use_prior: bool) !void{
    var xtmp = try self.x.copy();
    defer xtmp.destroy();
    @memset(xtmp.data, 0.0);

    for(self.filters, mode_probabilities.data)|*f, mu|{
        const source_state = if (use_prior) &f.x_prior else &f.x_posterior;
        var fx = try source_state.copy();
        defer fx.destroy();

        for(0..xtmp.data.len)|i|{
            xtmp.data[i] += fx.data[i] * mu;
        }

    }
    @memcpy(self.x.data, xtmp.data);

    var ctmp = try self.cov_posterior.copy();
    defer ctmp.destroy();
    @memset(ctmp.data, 0.0);

    for(self.filters, mode_probabilities.data)|*f, mu|{
        const source_state = if (use_prior) &f.x_prior else &f.x_posterior;
        const source_covariance = if (use_prior) &f.cov_prior else &f.cov_posterior;
        var y = try verify(try source_state.subtract(&self.x, null));
        defer y.destroy();
        var outer_product = try auto_outer_product(&y);
        defer outer_product.destroy();

        var ycov = try verify(try outer_product.add(source_covariance, null));
        defer ycov.destroy();

        for(0..ycov.data.len)|i|{
            ctmp.data[i] += ycov.data[i] * mu;
        }

    }
    @memcpy(self.cov.data, ctmp.data);
}

fn auto_outer_product(a: *Matrix) !Matrix{
    var a_t = try verify(try a.transpose(true));
    defer a_t.destroy();
    return try verify(try a.multiply(&a_t, null));
}






test "IMM predict matches scalar mixing across missed measurements and updates" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, test_predict, .{});
}

fn test_predict(allocator: std.mem.Allocator) !void {
    inline for (.{ .row_major, .col_major }) |order| {
        var ownership_transferred = false;
        const filters = try allocator.alloc(KalmanFilter, filter_len);
        defer allocator.free(filters);
        var initialized_count: usize = 0;
        defer if (!ownership_transferred) {
            for (filters[0..initialized_count]) |*filter| filter.deinit();
        };
        filters[0] = try test_create_filter(allocator, 0);
        initialized_count += 1;
        filters[1] = try test_create_filter(allocator, 3);
        initialized_count += 1;
        for (filters, 0..) |*filter, i| {
            filter.x_posterior.data[0] = @as(Float, @floatFromInt(i)) * 3;
            filter.cov_posterior.data[0] = 1;
            filter.F.data[0] = @floatFromInt(i + 1);
            filter.Q.data[0] = @as(Float, @floatFromInt(i + 1)) * 0.1;
        }
        var mu = try Matrix.create(allocator, try allocator.dupe(Float, &.{ 0.6, 0.4 }), 2, 1, .row_major);
        defer if (!ownership_transferred) mu.destroy();
        const transition_data: []const Float = if (order == .row_major) &.{ 0.9, 0.1, 0.2, 0.8 } else &.{ 0.9, 0.2, 0.1, 0.8 };
        var transition = try Matrix.create(allocator, try allocator.dupe(Float, transition_data), 2, 2, order);
        defer if (!ownership_transferred) transition.destroy();
        var imm = try init(allocator, filters, &mu, &transition, 1);
        ownership_transferred = true;
        defer imm.deinit();
        const state_buffer = imm.x_prior.data.ptr;
        const covariance_buffer = imm.cov_prior.data.ptr;

        try imm.predict();
        try std.testing.expectApproxEqRel(@as(Float, 2.16), imm.x_prior.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 9.8524), imm.cov_prior.data[0], 1e-12);
        try std.testing.expectEqualSlices(Float, &.{ 0.6, 0.4 }, imm.mu.data);
        try std.testing.expectEqual(@as(Float, 3), filters[1].x_posterior.data[0]);
        try std.testing.expect(imm.prediction_pending);

        // No measurement: propagate from the preceding priors and advance mu.
        try imm.predict();
        try std.testing.expectApproxEqRel(@as(Float, 0.62), imm.mu.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 0.634), imm.cbar.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 3.72), imm.x_prior.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 32.7152), imm.cov_prior.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 0.9463722397476342), filters[0].x_prior.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 8.524590163934429), filters[1].x_prior.data[0], 1e-12);
        try std.testing.expectEqual(@as(Float, 0), imm.x_posterior.data[0]);
        try std.testing.expectEqual(@as(Float, 0), imm.cov_posterior.data[0]);

        var measurement = try Matrix.identity(allocator, 1, 1, .row_major);
        defer measurement.destroy();
        var posterior = try imm.update(&measurement);
        defer posterior.state.destroy();
        defer posterior.covariance.destroy();
        try std.testing.expect(!imm.prediction_pending);
        const posterior_probability = imm.mu.data[0];
        const posterior_state = imm.x_posterior.data[0];
        const posterior_covariance = imm.cov_posterior.data[0];

        // After an update, mix the corrected posterior and retain posterior mu.
        try imm.predict();
        try std.testing.expectApproxEqRel(@as(Float, 1.1953715409559649), imm.x_prior.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 1.6394450301067003), imm.cov_prior.data[0], 1e-12);
        try std.testing.expectEqual(posterior_probability, imm.mu.data[0]);
        try std.testing.expectEqual(posterior_state, imm.x_posterior.data[0]);
        try std.testing.expectEqual(posterior_covariance, imm.cov_posterior.data[0]);
        try std.testing.expectEqualSlices(Float, imm.x.data, imm.x_prior.data);
        try std.testing.expectEqualSlices(Float, imm.cov.data, imm.cov_prior.data);
        try std.testing.expect(imm.x_prior.data.ptr == state_buffer);
        try std.testing.expect(imm.cov_prior.data.ptr == covariance_buffer);
        try std.testing.expect(imm.prediction_pending);
    }
}

test "IMM update matches a two-model scalar calculation" {
    const allocator = std.testing.allocator;
    inline for (.{ .row_major, .col_major }) |order| {
        var ownership_transferred = false;
        const filters = try allocator.alloc(KalmanFilter, filter_len);
        defer allocator.free(filters);
        var initialized_count: usize = 0;
        defer if (!ownership_transferred) {
            for (filters[0..initialized_count]) |*filter| filter.deinit();
        };
        filters[0] = try test_create_filter(allocator, 0);
        initialized_count += 1;
        filters[1] = try test_create_filter(allocator, 3);
        initialized_count += 1;

        var mu = try Matrix.create(allocator, try allocator.dupe(Float, &.{ 0.6, 0.4 }), 2, 1, .row_major);
        defer if (!ownership_transferred) mu.destroy();
        const transition_data: []const Float = if (order == .row_major) &.{ 0.9, 0.1, 0.2, 0.8 } else &.{ 0.9, 0.2, 0.1, 0.8 };
        var transition = try Matrix.create(allocator, try allocator.dupe(Float, transition_data), 2, 2, order);
        defer if (!ownership_transferred) transition.destroy();
        var imm = try init(allocator, filters, &mu, &transition, 1);
        ownership_transferred = true;
        defer imm.deinit();
        var measurement = try Matrix.identity(allocator, 1, 1, .row_major);
        defer measurement.destroy();

        imm.prediction_pending = true;
        var estimate = try imm.update(&measurement);
        defer estimate.state.destroy();
        defer estimate.covariance.destroy();

        // Prior model weights are [0.62, 0.38]. Scalar Kalman posteriors
        // are (mean=0.5, variance=0.5) and (mean=2, variance=0.5).
        const expected_mu = [_]Float{ 0.7754853665570497, 0.2245146334429503 };
        const expected_likelihood = [_]Float{ 0.21969564473386122, 0.1037768743551487 };
        const expected_cbar = [_]Float{ 0.7428397565899348, 0.25716024341006516 };
        const expected_omega = [_]Float{ 0.939552337781811, 0.30155725328058136, 0.060447662218188924, 0.6984427467194189 };
        for (0..2) |i| {
            try std.testing.expectApproxEqRel(expected_mu[i], imm.mu.data[i], 1e-12);
            try std.testing.expectApproxEqRel(expected_likelihood[i], imm.likelihood.data[i], 1e-12);
            try std.testing.expectApproxEqRel(expected_cbar[i], imm.cbar.data[i], 1e-12);
        }
        for (expected_omega, imm.omega.data) |expected, actual| {
            try std.testing.expectApproxEqRel(expected, actual, 1e-12);
        }
        try std.testing.expectApproxEqRel(@as(Float, 0.8367719501644255), estimate.state.data[0], 1e-12);
        try std.testing.expectApproxEqRel(@as(Float, 0.8917425788290879), estimate.covariance.data[0], 1e-12);
        try std.testing.expectEqualSlices(Float, imm.x.data, imm.x_posterior.data);
        try std.testing.expectEqualSlices(Float, imm.cov.data, imm.cov_posterior.data);
        try std.testing.expect(!imm.prediction_pending);

        // Returned estimates remain independent of the internal posterior.
        estimate.state.data[0] = -1;
        estimate.covariance.data[0] = -1;
        try std.testing.expect(imm.x_posterior.data[0] > 0);
        try std.testing.expect(imm.cov_posterior.data[0] > 0);
        var next_estimate = try imm.update(&measurement);
        defer next_estimate.state.destroy();
        defer next_estimate.covariance.destroy();
        const next_probability = expected_cbar[0] * expected_likelihood[0] /
            (expected_cbar[0] * expected_likelihood[0] + expected_cbar[1] * expected_likelihood[1]);
        try std.testing.expectApproxEqRel(next_probability, imm.mu.data[0], 1e-12);
    }
}

fn test_create_filter(allocator: std.mem.Allocator, initial_value: Float) !KalmanFilter {
    var transition = try Matrix.identity(allocator, 1, 1, .row_major);
    errdefer transition.destroy();
    var process_noise = try Matrix.empty(allocator, 1, 1, .row_major);
    errdefer process_noise.destroy();
    var measurement_noise = try Matrix.identity(allocator, 1, 1, .row_major);
    errdefer measurement_noise.destroy();
    var observation = try Matrix.identity(allocator, 1, 1, .row_major);
    errdefer observation.destroy();
    var initial_state = try Matrix.empty(allocator, 1, 1, .row_major);
    defer initial_state.destroy();
    initial_state.data[0] = initial_value;
    var initial_covariance = try Matrix.identity(allocator, 1, 1, .row_major);
    defer initial_covariance.destroy();
    return try KalmanFilter.init(allocator, &transition, &process_noise, &measurement_noise, &observation, &initial_state, &initial_covariance);
}

test "IMM init matches Python constructor with mixing probabilities" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, test_init, .{});
}

fn test_init(allocator: std.mem.Allocator) !void {
    var ownership_transferred = false;
    var mu = try Matrix.empty(allocator, 2, 1, .row_major);
    defer if (!ownership_transferred) mu.destroy();
    mu.data[0] = 2;
    mu.data[1] = 6;
    var transition_matrix = try Matrix.identity(allocator, 2, 2, .row_major);
    defer if (!ownership_transferred) transition_matrix.destroy();
    const filters = try allocator.alloc(KalmanFilter, filter_len);
    defer allocator.free(filters);
    var initialized_count: usize = 0;
    defer if (!ownership_transferred) {
        for (filters[0..initialized_count]) |*filter| filter.deinit();
    };
    for (filters) |*filter| {
        filter.* = try test_create_filter(allocator, 0);
        initialized_count += 1;
    }

    var imm = try init(allocator, filters, &mu, &transition_matrix, 3);
    ownership_transferred = true;
    defer imm.deinit();

    try std.testing.expectEqual(@as(usize, 2), imm.N);
    try std.testing.expect(imm.filters.ptr == filters[0..].ptr);
    try std.testing.expect(imm.transition.data.ptr == transition_matrix.data.ptr);
    try std.testing.expectEqualSlices(Float, &.{ 0.25, 0.75 }, imm.mu.data);
    try std.testing.expect(imm.mu.data.ptr == mu.data.ptr);
    try std.testing.expectEqualSlices(Float, &.{ 0.25, 0.75 }, imm.cbar.data);
    try std.testing.expectEqualSlices(Float, &.{ 1, 0, 0, 1 }, imm.omega.data);
    try std.testing.expectEqual(@as(c_int, 2), imm.omega.rows);
    try std.testing.expectEqual(@as(c_int, 2), imm.omega.cols);
    try std.testing.expect(!imm.prediction_pending);

    inline for (.{ "likelihood", "x_prior", "cov_prior", "x_posterior", "cov_posterior", "x", "cov" }) |field_name| {
        const matrix = @field(imm, field_name);
        const rows: c_int = if (std.mem.eql(u8, field_name, "likelihood")) 2 else 3;
        const cols: c_int = if (std.mem.startsWith(u8, field_name, "cov")) 3 else 1;
        try std.testing.expectEqual(rows, matrix.rows);
        try std.testing.expectEqual(cols, matrix.cols);
        for (matrix.data) |value| {
            try std.testing.expectEqual(@as(Float, 0), value);
        }
    }

    imm.x.data[0] = 1;
    imm.cov.data[0] = 1;
    try std.testing.expectEqual(@as(Float, 0), imm.x_prior.data[0]);
    try std.testing.expectEqual(@as(Float, 0), imm.x_posterior.data[0]);
    try std.testing.expectEqual(@as(Float, 0), imm.cov_prior.data[0]);
    try std.testing.expectEqual(@as(Float, 0), imm.cov_posterior.data[0]);
}
