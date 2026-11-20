const std = @import("std");
const math = std.math;
const mat = @import("Matrix.zig");
const Matrix = mat.Matrix;

pub fn multivariate_logpdf(comptime T: type, x: *Matrix(T), cov: *Matrix(T), mean: T) !T{
    return @log(try multivariate_pdf(T, x, cov, mean));
}

pub fn multivariate_pdf(comptime T: type, x: *Matrix(T), cov: *Matrix(T), mean: T) !T{
    const verify = Matrix(T).verify;
    var xcopy = try x.copy();
    defer xcopy.destroy();

    var cov_copy = try cov.copy();
    defer cov_copy.destroy();

    if(try cov_copy.is_singular()){
        return error.SingularMatrix;
    }
    const cov_det = cov_copy.det orelse return error.NullDeterminant;
    const cov_rank: T = @floatFromInt(try cov_copy.rank());

    _ = try cov_copy.inv(false);

    const scalar: T = 1 / @sqrt(math.pow(T, 2.0 * math.pi, cov_rank) * cov_det);

    xcopy.scalar_subtract(mean);

    var xcopy_t = try verify(try xcopy.transpose(true));
    defer xcopy_t.destroy();

    var xt_sigma = try verify(try xcopy_t.multiply(&cov_copy, null));
    defer xt_sigma.destroy();

    var x_square_sigma_mat = try verify(try xt_sigma.multiply(&xcopy, null));
    defer x_square_sigma_mat.destroy();

    const exp_val: T = -0.5 * x_square_sigma_mat.data[0];

    return scalar * @as(T, @exp(exp_val));
}

test "multivariate matches known output" {
    const allocator = std.testing.allocator;
    var x = try Matrix(f64).create(allocator, try allocator.dupe(f64, &.{ 3, 2 }), 2, 1, .row_major);
    defer x.destroy();

    var cov = try Matrix(f64).create(allocator, try allocator.dupe(f64, &.{
        4, 2,
        2, 3,
    }), 2, 2, .row_major);
    defer cov.destroy();

    const expected_pdf: f64 = 0.03412934057430108;
    const actual_pdf = try multivariate_pdf(f64, &x, &cov, 1.0);
    try std.testing.expectApproxEqRel(expected_pdf, actual_pdf, 1e-12);

    const expected_logpdf: f64 = @log(expected_pdf);
    const actual_logpdf = try multivariate_logpdf(f64, &x, &cov, 1.0);
    try std.testing.expectApproxEqRel(expected_logpdf, actual_logpdf, 1e-12);


    std.debug.print("Actual log PDF: {d}. Expected log PDF: {d}\n", .{expected_logpdf, actual_logpdf});

}
