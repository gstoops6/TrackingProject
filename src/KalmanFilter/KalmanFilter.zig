const std = @import("std");
const KalmanFilter = @This();
const Matrix = @import("Matrix.zig").Matrix;
const Stats = @import("Stats.zig");
pub const Float = @import("root").config.Float;

const logpdf = Stats.multivariate_logpdf;
const pdf = Stats.multivariate_pdf;

const EstimateType = enum{
    posterior,
    prior,
};

const Estimate = struct{
    state: Matrix(Float),
    covariance: Matrix(Float),
};

F: Matrix(Float), // State Transition
Q: Matrix(Float), // Process Noise
R: Matrix(Float), // Measurement Noise
H: Matrix(Float), // Observation
K: Matrix(Float), // Kalman Gain
I: Matrix(Float), // Identity matrix dimensions F.cols

x_posterior: Matrix(Float), // Prior state estimate used during update
cov_posterior: Matrix(Float), // Prior covariance estimate used during update
x_prior: Matrix(Float), // Posterior state estimate used during prediction
cov_prior: Matrix(Float), //Posterior covariance estimate used during prediction

log_likelihood: Float = 0.0,
likelihood: Float = 0.0,

previous_measurement: Matrix(Float) = undefined,
has_previous_measurement: bool = false,
work_buffers: WorkBuffers = undefined,





/// Work matrices are allocated once and owned by the filter.
const WorkBuffers = struct{
    ft: Matrix(Float),
    fp: Matrix(Float),
    fpft: Matrix(Float),
    kh: Matrix(Float),
    kr: Matrix(Float),
    kt: Matrix(Float),
    ikh: Matrix(Float),
    ikh_t: Matrix(Float),
    krkt: Matrix(Float),
    p_left: Matrix(Float),
    p_squared: Matrix(Float),
    hx_prior: Matrix(Float),
    z_hx: Matrix(Float),
    k_z_hx: Matrix(Float),
    ht: Matrix(Float),
    pht: Matrix(Float),
    hp: Matrix(Float),
    hph: Matrix(Float),
    pinv: Matrix(Float),
    hx: Matrix(Float),
    y: Matrix(Float),
    hcov: Matrix(Float),
    hcov_h: Matrix(Float),
    innovation_cov: Matrix(Float),
};


pub fn init(
    smp: std.mem.Allocator,
    F: *Matrix(Float),
    Q: *Matrix(Float),
    R: *Matrix(Float),
    H: *Matrix(Float),
    initial_estimate: *Matrix(Float),
    initial_covariance: *Matrix(Float)
) !KalmanFilter{
    var K: Matrix(Float) = try .empty(smp, F.rows, R.rows, .row_major);
    errdefer K.destroy();
    var I: Matrix(Float) = try .identity(smp, F.rows, F.cols, .row_major);
    errdefer I.destroy();
    var x_prior: Matrix(Float) = try .empty(smp, F.rows, initial_estimate.cols, .row_major);
    errdefer x_prior.destroy();
    var x_posterior: Matrix(Float) = try x_prior.copy();
    errdefer x_posterior.destroy();
    var cov_prior: Matrix(Float) = try .empty(smp, F.rows, Q.cols, .row_major);
    errdefer cov_prior.destroy();
    var cov_posterior: Matrix(Float) = try cov_prior.copy();
    errdefer cov_posterior.destroy();

    var kf: KalmanFilter = .{
        .F = F.*,
        .Q = Q.*,
        .R = R.*,
        .H = H.*,
        .I = I,
        .K = K,
        .cov_prior = cov_prior,
        .cov_posterior = cov_posterior,
        .x_prior = x_prior,
        .x_posterior = x_posterior
    };

    try kf.create_work_buffers(smp);
    errdefer kf.destroy_work_buffers();
    try state_prediction(&kf.x_posterior, F, initial_estimate);
    try covariance_predict(&kf, &kf.cov_posterior, F, initial_covariance, Q);

    return kf;

}


fn create_work_buffers(self: *KalmanFilter, smp: std.mem.Allocator) !void{
    const work_dimensions = .{
        .{ "ft", self.F.cols, self.F.rows, self.F.order },
        .{ "fp", self.F.rows, self.cov_posterior.cols, self.F.order },
        .{ "fpft", self.F.rows, self.F.rows, self.F.order },
        .{ "kh", self.K.rows, self.H.cols, self.K.order },
        .{ "kr", self.K.rows, self.R.cols, self.K.order },
        .{ "kt", self.K.cols, self.K.rows, self.K.order },
        .{ "ikh", self.I.rows, self.I.cols, self.I.order },
        .{ "ikh_t", self.I.cols, self.I.rows, self.I.order },
        .{ "krkt", self.K.rows, self.K.rows, self.K.order },
        .{ "p_left", self.I.rows, self.cov_prior.cols, self.I.order },
        .{ "p_squared", self.I.rows, self.I.rows, self.I.order },
        .{ "hx_prior", self.H.rows, self.x_prior.cols, self.H.order },
        .{ "z_hx", self.H.rows, self.x_prior.cols, self.H.order },
        .{ "k_z_hx", self.K.rows, self.x_prior.cols, self.K.order },
        .{ "ht", self.H.cols, self.H.rows, self.H.order },
        .{ "pht", self.cov_prior.rows, self.H.rows, self.cov_prior.order },
        .{ "hp", self.H.rows, self.cov_prior.cols, self.H.order },
        .{ "hph", self.H.rows, self.H.rows, self.H.order },
        .{ "pinv", self.H.rows, self.H.rows, self.H.order },
        .{ "hx", self.H.rows, self.x_prior.cols, self.H.order },
        .{ "y", self.H.rows, self.x_prior.cols, self.H.order },
        .{ "hcov", self.H.rows, self.cov_prior.cols, self.H.order },
        .{ "hcov_h", self.H.rows, self.H.rows, self.H.order },
        .{ "innovation_cov", self.H.rows, self.H.rows, self.H.order },
    };

    var initialized_count: usize = 0;
    errdefer {
        inline for (work_dimensions, 0..) |dimensions, index| {
            if (index < initialized_count) @field(self.work_buffers, dimensions[0]).destroy();
        }
    }
    inline for (work_dimensions) |dimensions| {
        @field(self.work_buffers, dimensions[0]) = try Matrix(Float).empty(smp, dimensions[1], dimensions[2], dimensions[3]);
        initialized_count += 1;
    }
    self.previous_measurement = try Matrix(Float).empty(smp, self.H.rows, self.x_prior.cols, .row_major);
}

fn destroy_work_buffers(self: *KalmanFilter) void{
    inline for (@typeInfo(WorkBuffers).@"struct".fields) |field| {
        @field(self.work_buffers, field.name).destroy();
    }
    self.previous_measurement.destroy();
}


/// Values passed to Kalman filter should not be destroyed by creator. Freeing upon completion will cause a double free.
pub fn deinit(self: *KalmanFilter) void{
    self.destroy_work_buffers();
    self.F.destroy();
    self.Q.destroy();
    self.H.destroy();
    self.R.destroy();
    self.I.destroy();
    self.K.destroy();
    self.x_prior.destroy();
    self.x_posterior.destroy();
    self.cov_posterior.destroy();
    self.cov_prior.destroy();
}

pub fn predict(self: *KalmanFilter) !void{
    try state_prediction(&self.x_prior, &self.F, &self.x_posterior);
    try covariance_predict(self, &self.cov_prior, &self.F, &self.cov_posterior, &self.Q);
}

pub fn update(self: *KalmanFilter, measurement: ?*Matrix(Float)) !void{
    if (measurement) |z| {
        if (z.rows != self.previous_measurement.rows or z.cols != self.previous_measurement.cols) return error.InvalidMeasurementDimensions;
    }
    try gain_update(self, &self.K, &self.cov_prior, &self.H, &self.R);
    try joseph(self, &self.cov_posterior,  &self.I, &self.K, &self.H, &self.cov_prior, &self.R);

    if(measurement)|z|{
        try state_update(self, &self.x_posterior, &self.x_prior, z, &self.K, &self.H);
        @memcpy(self.previous_measurement.data, z.data);
        self.previous_measurement.order = z.order;
        self.has_previous_measurement = true;
    } else{
        @memcpy(self.x_posterior.data, self.x_prior.data);
    }
}

/// Getter function that returns copy of different estimates from the filter.
/// Direct grabbing of state data should be avoided in case of accidental manipulation of data
/// causing a complete divergence of the filter.
pub fn get_estimate(self: *KalmanFilter, state: EstimateType) !Estimate{
    return .{
        .state = if(state == .posterior) try self.x_posterior.copy() else try self.x_prior.copy(),
        .covariance = if(state == .posterior) try self.cov_posterior.copy() else try self.cov_prior.copy()
    };
}

pub fn compute_log_likelihood(self: *KalmanFilter) !Float{
    try self.compute_innovation();
    self.log_likelihood = try logpdf(Float, &self.work_buffers.y, &self.work_buffers.innovation_cov, 0.0);
    return self.log_likelihood;
}

pub fn compute_likelihood(self: *KalmanFilter) !Float{
    try self.compute_innovation();
    const likelihood = try pdf(Float, &self.work_buffers.y, &self.work_buffers.innovation_cov, 0.0);

    self.likelihood = if (likelihood == 0.0) std.math.floatEps(Float) else likelihood;
    return self.likelihood;
}

fn compute_innovation(self: *KalmanFilter) !void{
    if (!self.has_previous_measurement) return error.MissingMeasurement;
    const work = &self.work_buffers;
    _ = try self.H.multiply(&self.x_prior, &work.hx);
    work.y.order = self.previous_measurement.order;
    _ = try self.previous_measurement.subtract(&work.hx, &work.y);
    try transpose_into(&self.H, &work.ht);
    _ = try self.H.multiply(&self.cov_prior, &work.hcov);
    _ = try work.hcov.multiply(&work.ht, &work.hcov_h);
    _ = try work.hcov_h.add(&self.R, &work.innovation_cov);
}


/// Helper Functions
fn transpose_into(source: *Matrix(Float), out: *Matrix(Float)) !void{
    if (out.rows != source.cols or out.cols != source.rows) return error.InvalidOutputDimensions;
    if (out.order != source.order) return error.OutputOrderInvalid;
    const rows: usize = @intCast(source.rows);
    const cols: usize = @intCast(source.cols);
    for (0..rows) |row| {
        for (0..cols) |col| {
            const source_index = if (source.order == .row_major) row * cols + col else col * rows + row;
            const output_index = if (out.order == .row_major) col * rows + row else row * cols + col;
            out.data[output_index] = source.data[source_index];
        }
    }
}

/// Predicts state based off prior and state transition
/// F - state transition matrix
/// xpost - posterior state estimate
/// xpri - prior state estimate
inline fn state_prediction(xpri: *Matrix(Float), state_transition: *Matrix(Float), xpost: *Matrix(Float)) !void{
    _ = try state_transition.multiply(xpost, xpri);
}

/// Calculates covariance prediction based off posterior estimate
/// F - state transition matrix
/// P - posterior covariance estimate
/// Q - process noise matrix
/// Ppri - prior covariance estimate
inline fn covariance_predict(self: *KalmanFilter, prior_cov: *Matrix(Float), state_transition: *Matrix(Float), covariance: *Matrix(Float), process_noise: *Matrix(Float)) !void{
    const work = &self.work_buffers;
    try transpose_into(state_transition, &work.ft);
    _ = try state_transition.multiply(covariance, &work.fp);
    _ = try work.fp.multiply(&work.ft, &work.fpft);
    _ = try work.fpft.add(process_noise, prior_cov);
}

/// Calculates the Joseph form of the covariance update.
/// This for of calculation is more robust and less prone to numerical errors, like floating-point, than
///  the standard form at the cost of more calculation time
/// I - identity matrix
/// K - Kalman gain
/// H - observation matrix
/// P - prior covariance
/// R - measurement noise
/// Ppost - posterior covariance estimate
inline fn joseph(self: *KalmanFilter, posterior_cov: *Matrix(Float), identity: *Matrix(Float), gain: *Matrix(Float), observation: *Matrix(Float), prior_cov: *Matrix(Float), measurement_noise: *Matrix(Float)) !void{
    const work = &self.work_buffers;
    _ = try gain.multiply(observation, &work.kh);
    _ = try gain.multiply(measurement_noise, &work.kr);
    try transpose_into(gain, &work.kt);
    _ = try identity.subtract(&work.kh, &work.ikh);
    try transpose_into(&work.ikh, &work.ikh_t);
    _ = try work.kr.multiply(&work.kt, &work.krkt);
    _ = try work.ikh.multiply(prior_cov, &work.p_left);
    _ = try work.p_left.multiply(&work.ikh_t, &work.p_squared);
    _ = try work.p_squared.add(&work.krkt, posterior_cov);
}

/// Calculates the state estimate based on the update to the Kalman Filter
/// xprior is the prior state
/// z - measurement
/// K - Kalman gain
/// H - observation matrix
/// xpost - posterior state estimate
inline fn state_update(self: *KalmanFilter, posterior_state: *Matrix(Float), prior_state: *Matrix(Float), measurement: *Matrix(Float), gain: *Matrix(Float), observation: *Matrix(Float)) !void{
    const work = &self.work_buffers;
    _ = try observation.multiply(prior_state, &work.hx_prior);
    work.z_hx.order = measurement.order;
    _ = try measurement.subtract(&work.hx_prior, &work.z_hx);
    _ = try gain.multiply(&work.z_hx, &work.k_z_hx);
    _ = try prior_state.add(&work.k_z_hx, posterior_state);
}

/// Updates the Kalman gain during the update step of the Kalman Filter
/// P - prior covariance
/// H - observation matrix
/// R - measurement noise
/// K - Kalman gain
inline fn gain_update(self: *KalmanFilter, gain: *Matrix(Float), prior_cov: *Matrix(Float), observation: *Matrix(Float), measurement_noise: *Matrix(Float)) !void{
    //P @ H.T @ np.linalg.pinv(H @ P @ H.T + R))

    const work = &self.work_buffers;
    try transpose_into(observation, &work.ht);
    _ = try prior_cov.multiply(&work.ht, &work.pht);
    _ = try observation.multiply(prior_cov, &work.hp);
    _ = try work.hp.multiply(&work.ht, &work.hph);
    _ = try work.hph.add(measurement_noise, &work.pinv);
    _ = try work.pinv.inv(false);
    _ = try work.pht.multiply(&work.pinv, gain);
}
