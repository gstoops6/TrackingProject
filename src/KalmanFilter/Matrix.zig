const std = @import("std");
const cblas = @cImport({
    @cInclude("cblas.h");
    @cInclude("openblas_config.h");
    @cInclude("lapack.h");
    @cInclude("lapacke.h");
    @cInclude("lapacke_utils.h");
});

const RowMajorT = cblas.CblasRowMajor;
const ColumnMajorT = cblas.CblasColMajor;
const NoTranspose = cblas.CblasNoTrans;
const DoTranspose = cblas.CblasTrans;
const RowMajor = cblas.LAPACK_ROW_MAJOR;
const ColMajor = cblas.LAPACK_COL_MAJOR;

const MatrixOrder = enum(c_int) { row_major = RowMajor, col_major = ColMajor };

const Size = struct {
    rows: usize,
    cols: usize,
};

pub fn Matrix(comptime T: type) type {

    if(T != f64 and T != f32) @compileError("Matrix must be either single or double precision floating-point");

    return struct {
        const Self = @This();

        const l1 = if(T == f64) cblas.LAPACKE_dlange else cblas.LAPACKE_slange;
        const lu = if(T == f64) cblas.LAPACKE_dgetrf else cblas.LAPACKE_sgetrf;
        const gemm = if(T == f64) cblas.cblas_dgemm else cblas.cblas_sgemm;
        const trans = if(T == f64) cblas.LAPACKE_dge_trans else cblas.LAPACKE_sge_trans;
        const getrf = if(T == f64) cblas.LAPACKE_dgetrf else cblas.LAPACKE_sgetrf;
        const getri = if(T == f64) cblas.LAPACKE_dgetri else cblas.LAPACKE_sgetri;
        const gesdd = if(T == f64) cblas.LAPACKE_dgesdd else cblas.LAPACKE_sgesdd;
        const lapack_int = cblas.lapack_int;

        smp: std.mem.Allocator,
        rows: lapack_int,
        cols: lapack_int,
        data: []T,
        det: ?T = null, // Must call is_singular() before getting determinant
        order: MatrixOrder,

        /// Lifetime functions for matrix
        pub fn create(smp: std.mem.Allocator, data: []T, r: anytype, c: anytype, t: MatrixOrder) !Self {
            const rtype = @TypeOf(r);
            const ctype = @TypeOf(c);

            if (rtype != usize and rtype != c_int and rtype != c_uint and rtype != i32 and rtype != comptime_int) return error.InvalidType;
            if (ctype != usize and ctype != c_int and ctype != c_uint and ctype != i32 and ctype != comptime_int) return error.InvalidType;

            if (r * c != data.len) return error.InvalidMatrixInitialization;
            return .{
                .smp = smp,
                .cols = @intCast(c),
                .rows = @intCast(r),
                .data = data,
                .order = t,
            };
        }

        pub fn destroy(self: *Self) void {
            self.smp.free(self.data);
        }

        pub fn copy(self: *Self) !Self {
            return .{
                .rows = self.rows,
                .cols = self.cols,
                .data = try self.smp.dupe(T, self.data),
                .order = self.order,
                .smp = self.smp,
            };
        }

        pub fn verify(mat: ?Self) !Self {
            return mat orelse error.NullMatrix;
        }

        pub fn identity(smp: std.mem.Allocator, r: anytype, c: anytype, t: MatrixOrder) !Self {
            const rtype = @TypeOf(r);
            const ctype = @TypeOf(c);

            if (rtype != usize and rtype != c_int and rtype != c_uint and rtype != i32 and rtype != comptime_int) return error.InvalidType;
            if (ctype != usize and ctype != c_int and ctype != c_uint and ctype != i32 and ctype != comptime_int) return error.InvalidType;

            const data = try smp.alloc(T, @intCast(r * c));
            errdefer smp.free(data);
            @memset(data, 0);

            const m: usize = @intCast(r);
            const n: usize = @intCast(c);

            for (0..m) |i| {
                for (0..n) |j| {
                    if (i == j) {
                        if (t == .row_major) {
                            data[i * n + j] = 1.0;
                        } else {
                            data[j * m + i] = 1.0;
                        }
                    }
                }
            }

            return .{ .rows = @intCast(r), .cols = @intCast(c), .data = data, .order = t, .smp = smp };
        }

        pub fn empty(smp: std.mem.Allocator, r: anytype, c: anytype, t: MatrixOrder) !Self {
            const rtype = @TypeOf(r);
            const ctype = @TypeOf(c);

            if (rtype != usize and rtype != c_int and rtype != c_uint and rtype != i32 and rtype != comptime_int) return error.InvalidType;
            if (ctype != usize and ctype != c_int and ctype != c_uint and ctype != i32 and ctype != comptime_int) return error.InvalidType;

            const data = try smp.alloc(T, @intCast(r * c));
            errdefer smp.free(data);
            @memset(data, 0);

            return .{
                .rows = @intCast(r),
                .cols = @intCast(c),
                .data = data,
                .order = t,
                .smp = smp,
            };
        }

        /// Public wrapper Functions for Matrix operations
        pub fn inv(self: *Self, create_copy: bool) !?Self {
            if (create_copy) {
                var cp = try self.copy();
                errdefer cp.destroy();
                try cp.invert();
                return cp;
            } else {
                try self.invert();
                return null;
            }
        }

        pub fn add(self: *Self, b: *Self, out: ?*Self) !?Self {
            if (out) |o| {
                try self.addition(b, o);
                return null;
            }

            var o = try self.copy();
            errdefer o.destroy();
            try self.addition(b, &o);

            return o;
        }

        pub fn subtract(self: *Self, b: *Self, out: ?*Self) !?Self {
            if (out) |o| {
                try self.subtraction(b, o);
                return null;
            }

            var o = try self.copy();
            errdefer o.destroy();
            try self.subtraction(b, &o);
            return o;
        }

        pub fn transpose(self: *Self, create_copy: bool) !?Self {
            if (create_copy) {
                var cp = try self.copy();
                errdefer cp.destroy();
                try cp.transposition();
                return cp;
            } else {
                try self.transposition();
                return null;
            }
        }

        pub fn multiply(self: *Self, b: *Self, out: ?*Self) !?Self {
            if (out) |o| {
                try self.multiplication(b, o);
                return null;
            }

            const data = try self.smp.alloc(T, @intCast(self.rows * b.cols));
            errdefer self.smp.free(data);
            var o: Self = try .create(self.smp, data, self.rows, b.cols, self.order);

            try self.multiplication(b, &o);
            return o;
        }

        pub fn scalar_multiply(self: *Self, scalar: T) void {
            for (0..self.rows * self.cols) |i| {
                self.data[i] *= scalar;
            }
        }

        pub fn scalar_add(self: *Self, scalar: T) void {
            for (0..self.rows * self.cols) |i| {
                self.data[i] += scalar;
            }
        }

        pub fn scalar_subtract(self: *Self, scalar: T) void {
            for (0..@intCast(self.rows * self.cols)) |i| {
                self.data[i] -= scalar;
            }
        }

        pub fn size(self: *Self) Size {
            return .{ .rows = @intCast(self.rows), .cols = @intCast(self.cols) };
        }

        pub fn rank(self: *Self) !lapack_int{
            if (self.rows == 0 or self.cols == 0) return error.InvalidMatrixDimensions;

            const max_rc: lapack_int = @max(self.rows, self.cols);
            const min_rc: lapack_int = @min(self.rows, self.cols);

            const dupe = try self.smp.dupe(T, self.data);
            defer self.smp.free(dupe);
            const lda = if (self.order == .row_major) self.cols else self.rows;

            // Perform singular value decomposition on matrix
            const s = try self.smp.alloc(T, @intCast(min_rc));
            defer self.smp.free(s);


            const info = gesdd(@intFromEnum(self.order), 'N', self.rows, self.cols, dupe.ptr, lda, s.ptr, null, 1, null, 1);

            if (info > 0) {
                return error.SVDFailure;
            } else if (info < 0) {
                return error.MatixContainsIllegalValue;
            }

            const tolerance: T = @as(T, @floatFromInt(max_rc)) * s[0] * std.math.floatEps(T);
            var _rank: lapack_int = 0;
            for(0..@intCast(min_rc))|i|{
                if(s[i] > tolerance){
                    _rank += 1;
                } else {
                    break;
                }
            }

            return _rank;
        }

        pub fn is_singular(self: *Self) !bool {
            const data = try self.smp.dupe(T, self.data);
            defer self.smp.free(data);

            const ipiv = try self.smp.alloc(lapack_int, @intCast(self.rows * self.cols));
            defer self.smp.free(ipiv);

            const lda = if (self.order == .col_major) self.rows else self.cols;

            const info = getrf(@intFromEnum(self.order), self.rows, self.cols, data.ptr, lda, ipiv.ptr);

            if(info == 0){
                var _det: T = 1.0;
                for(0..@intCast(self.cols))|i|{
                    if (self.order == .row_major) {
                        _det *= data[i * @as(usize, @intCast(self.cols)) + i];
                    } else {
                        _det *= data[i * @as(usize, @intCast(self.rows)) + i];
                    }

                    if(ipiv[i] != @as(lapack_int, @intCast(i + 1))){
                        _det = -_det;
                    }
                }
                self.det = _det;
                return false;
            }else if (info > 0) {
                return true;
            } else {
                return error.MatixContainsIllegalValue;
            }
        }

        /// Private helper functions for matrix
        /// Transposes data using a separate output buffer, preserving storage order.
        fn transposition(self: *Self) !void {
            const transposed_data = try self.smp.alloc(T, self.data.len);
            const ldin = if (self.order == .col_major) self.rows else self.cols;
            const ldout = if (self.order == .col_major) self.cols else self.rows;

            trans(@intFromEnum(self.order), self.rows, self.cols, self.data.ptr, ldin, transposed_data.ptr, ldout);

            self.smp.free(self.data);
            self.data = transposed_data;
            std.mem.swap(lapack_int, &self.rows, &self.cols);
        }

        fn invert(self: *Self) !void {
            if (self.rows == self.cols) {
                return invert_square(self);
            } else {
                return psuedo_inverse(self);
            }
        }

        fn invert_square(self: *Self) !void {
            const ipiv = try self.smp.alloc(lapack_int, @intCast(self.rows * self.cols));
            defer self.smp.free(ipiv);

            const lda = self.rows;

            var info = getrf(@intFromEnum(self.order), self.rows, self.cols, self.data.ptr, lda, ipiv.ptr);

            if (info > 0) {
                return error.SingularMatrix;
            } else if (info < 0) {
                return error.MatixContainsIllegalValue;
            }

            info = getri(@intFromEnum(self.order), self.rows, self.data.ptr, lda, ipiv.ptr);

            if (info != 0) return error.MatrixInversionFailure;
        }

        fn psuedo_inverse(self: *Self) !void {
            if (self.rows == 0 or self.cols == 0) return error.InvalidMatrixDimensions;
            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            const max_rc: lapack_int = @max(self.rows, self.cols);
            const min_rc: lapack_int = @min(self.rows, self.cols);

            const dupe = try self.smp.dupe(T, self.data);
            defer self.smp.free(dupe);
            const lda = if (self.order == .row_major) self.cols else self.rows;
            const ldu = self.rows;
            const ldvt = self.cols;
            const ld_pinv = if (self.order == .row_major) self.rows else self.cols;
            const blas_order: cblas.enum_CBLAS_ORDER = if (self.order == .row_major) RowMajorT else ColumnMajorT;

            // Perform singular value decomposition on matrix
            const s = try self.smp.alloc(T, @intCast(min_rc));
            defer self.smp.free(s);
            const u = try self.smp.alloc(T, @intCast(self.rows * self.rows));
            defer self.smp.free(u);
            const vt = try self.smp.alloc(T, @intCast(self.cols * self.cols));
            defer self.smp.free(vt);

            const info = gesdd(@intFromEnum(self.order), 'A', self.rows, self.cols, dupe.ptr, lda, s.ptr, u.ptr, ldu, vt.ptr, ldvt);

            if (info > 0) {
                return error.SVDFailure;
            } else if (info < 0) {
                return error.MatixContainsIllegalValue;
            }

            const eps = std.math.floatEps(T);
            const tolerance = @as(T, @floatFromInt(max_rc)) * eps * s[0];
            const dimension_max = n;

            const tmp_pinv = try self.smp.alloc(T, @intCast(self.rows * self.cols));
            @memset(tmp_pinv, 0);
            defer self.smp.free(tmp_pinv);

            for (0..@intCast(min_rc)) |i| {
                if (s[i] > tolerance) {
                    const reciprocal = 1.0 / s[i];
                    for (0..dimension_max) |j| {
                        if (self.order == .row_major) {
                            tmp_pinv[j * m + i] = vt[i * n + j] * reciprocal;
                        } else {
                            tmp_pinv[i * n + j] = vt[j * n + i] * reciprocal;
                        }
                    }
                }
            }

            const pinv = try self.smp.alloc(T, @intCast(self.rows * self.cols));
            gemm(
                blas_order,
                NoTranspose,
                DoTranspose,
                self.cols,
                self.rows,
                self.rows,
                1.0,
                tmp_pinv.ptr,
                ld_pinv,
                u.ptr,
                ldu,
                0.0,
                pinv.ptr,
                ld_pinv,
            );

            self.smp.free(self.data);
            self.data = pinv;
            std.mem.swap(lapack_int, &self.rows, &self.cols);
        }

        fn addition(self: *Self, b: *Self, out: *Self) !void {
            if (self.order != out.order) return error.OutputOrderInvalid;
            if (self.rows <= 0 or self.cols <= 0 or b.rows <= 0 or b.cols <= 0 or out.rows <= 0 or out.cols <= 0) {
                return error.CannotAddZeroDimensionMatrix;
            }
            if (out.rows != self.rows or out.cols != self.cols) return error.InvalidOutputDimensions;

            if (b.cols == 1 and b.rows == self.rows) {
                return add_column_vector(self, b, out);
            } else if (b.rows == 1 and b.cols == self.cols) {
                return add_row_vector(self, b, out);
            }
            if (b.rows != self.rows or b.cols != self.cols) return error.InvalidAddDimensions;

            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major and b.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] + b.data[i * n + j];
                    } else if (self.order == .row_major and b.order == .col_major) {
                        out.data[i * n + j] = self.data[i * n + j] + b.data[j * m + i];
                    } else if (self.order == .col_major and b.order == .row_major) {
                        out.data[j * m + i] = self.data[j * m + i] + b.data[i * n + j];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] + b.data[j * m + i];
                    }
                }
            }
        }

        fn add_column_vector(self: *Self, b: *Self, out: *Self) void {
            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] + b.data[i];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] + b.data[i];
                    }
                }
            }
        }

        fn add_row_vector(self: *Self, b: *Self, out: *Self) void {
            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] + b.data[j];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] + b.data[j];
                    }
                }
            }
        }

        fn subtraction(self: *Self, b: *Self, out: *Self) !void {
            if (self.order != out.order) return error.OutputOrderInvalid;
            if (self.rows <= 0 or self.cols <= 0 or b.rows <= 0 or b.cols <= 0 or out.rows <= 0 or out.cols <= 0) {
                return error.CannotAddZeroDimensionMatrix;
            }
            if (out.rows != self.rows or out.cols != self.cols) return error.InvalidOutputDimensions;

            if (b.cols == 1 and b.rows == self.rows) {
                return subtract_column_vector(self, b, out);
            } else if (b.rows == 1 and b.cols == self.cols) {
                return subtract_row_vector(self, b, out);
            }
            if (b.rows != self.rows or b.cols != self.cols) return error.InvalidAddDimensions;

            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major and b.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] - b.data[i * n + j];
                    } else if (self.order == .row_major and b.order == .col_major) {
                        out.data[i * n + j] = self.data[i * n + j] - b.data[j * m + i];
                    } else if (self.order == .col_major and b.order == .row_major) {
                        out.data[j * m + i] = self.data[j * m + i] - b.data[i * n + j];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] - b.data[j * m + i];
                    }
                }
            }
        }

        fn subtract_column_vector(self: *Self, b: *Self, out: *Self) void {
            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] - b.data[i];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] - b.data[i];
                    }
                }
            }
        }

        fn subtract_row_vector(self: *Self, b: *Self, out: *Self) void {
            const m: usize = @intCast(self.rows);
            const n: usize = @intCast(self.cols);
            for (0..m) |i| {
                for (0..n) |j| {
                    if (self.order == .row_major) {
                        out.data[i * n + j] = self.data[i * n + j] - b.data[j];
                    } else {
                        out.data[j * m + i] = self.data[j * m + i] - b.data[j];
                    }
                }
            }
        }

        fn multiplication(self: *Self, b: *Self, out: *Self) !void {
            if (self.cols != b.rows) return error.InvalidMultiplyDimensions;
            if (self.order != out.order) return error.OutputMatrixOrderInvalid;
            if (out.rows != self.rows or out.cols != b.cols) return error.InvalidOutputDimensions;

            const trans_b = if (b.order != self.order) DoTranspose else NoTranspose;
            const lda = if (self.order == .row_major) self.cols else self.rows;
            const ldb = if (b.order == .row_major) b.cols else b.rows;
            const ldc = if (out.order == .row_major) out.cols else out.rows;

            gemm(
                @intCast(@intFromEnum(self.order)),
                NoTranspose,
                @intCast(trans_b),
                self.rows,
                b.cols,
                self.cols,
                1.0,
                self.data.ptr,
                lda,
                b.data.ptr,
                ldb,
                0.0,
                out.data.ptr,
                ldc,
            );
        }

        fn swap_order(self: *Self) void {
            if (self.order == .col_major) {
                self.order = .row_major;
            } else {
                self.order = .col_major;
            }
            std.mem.swap(lapack_int, &self.rows, &self.cols);
        }
    };
}

// Tests
test "simple vector add" {
    const Float = f64;
    const TestMatrix = Matrix(Float);

    const ra = 3;
    const ca = 3;
    var data_a = [ra * ca]Float{ 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

    const rb = 3;
    const cb = 1;
    var data_b = [rb * cb]Float{ //Row Major
        1.0,
        0.0,
        1.0,
    };

    const alloc_a = try std.testing.allocator.dupe(Float, &data_a);
    const alloc_b = try std.testing.allocator.dupe(Float, &data_b);

    var a = try TestMatrix.create(std.testing.allocator, alloc_a, ra, ca, .row_major);
    defer a.destroy();

    var b = try TestMatrix.create(std.testing.allocator, alloc_b, rb, cb, .row_major);
    defer b.destroy();

    var c = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.alloc(Float, ra * ca), ra, ca, .row_major);
    defer c.destroy();

    try a.addition(&b, &c);

    try std.testing.expectEqualSlices(Float, &.{ 2, 1, 2, 0, 1, 1, 2, 2, 2 }, c.data);
}

test "matrix multiply different order" {
    const Float = f32;
    const TestMatrix = Matrix(Float);

    const expected = [_]Float{ 1, 0, 0, 0, 1, 0, 1, 1, 0 };

    const ra = 3;
    const ca = 2;
    var data_a = [ra * ca]Float{ 1.0, 0.0, 0.0, 1.0, 1.0, 1.0 };

    const rb = 2;
    const cb = 3;
    var data_b = [rb * cb]Float{ //Column Major
        1.0, 0.0,
        0.0, 1.0,
        0.0, 0.0,
    };

    const alloc_a = try std.testing.allocator.dupe(Float, &data_a);
    const alloc_b = try std.testing.allocator.dupe(Float, &data_b);

    var a = try TestMatrix.create(std.testing.allocator, alloc_a, ra, ca, .row_major);
    defer a.destroy();

    var b = try TestMatrix.create(std.testing.allocator, alloc_b, rb, cb, .col_major);
    defer b.destroy();

    var c = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.alloc(Float, ra * cb), ra, cb, .row_major);
    defer c.destroy();

    try a.multiplication(&b, &c);

    for (0..c.data.len) |i| {
        //std.debug.print("Expected: {d}. Actual: {d}\n", .{expected[i], c.data[i]});
        try std.testing.expectApproxEqAbs(expected[i], c.data[i], 0.0001);
    }
}

test "matrix multiply same order" {
    const Float = f64;
    const TestMatrix = Matrix(Float);

    const expected = [_]Float{ 1, 0, 0, 0, 1, 0, 1, 1, 0 };

    const ra = 3;
    const ca = 2;
    var data_a = [ra * ca]Float{ 1.0, 0.0, 0.0, 1.0, 1.0, 1.0 };

    const rb = 2;
    const cb = 3;
    var data_b = [rb * cb]Float{ //Row Major
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    };

    const alloc_a = try std.testing.allocator.dupe(Float, &data_a);
    const alloc_b = try std.testing.allocator.dupe(Float, &data_b);

    var a = try TestMatrix.create(std.testing.allocator, alloc_a, ra, ca, .row_major);
    defer a.destroy();

    var b = try TestMatrix.create(std.testing.allocator, alloc_b, rb, cb, .row_major);
    defer b.destroy();

    var c = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.alloc(Float, ra * cb), ra, cb, .row_major);
    defer c.destroy();

    try a.multiplication(&b, &c);

    for (0..c.data.len) |i| {
        try std.testing.expectApproxEqAbs(expected[i], c.data[i], 0.0001);
    }
}

test "matrix multiplied by its inverse produces identity" {
    const Float = f32;
    const TestMatrix = Matrix(Float);

    const data = [_]Float{
        4, 7,
        2, 6,
    };
    const expected = [_]Float{
        1, 0,
        0, 1,
    };

    var matrix = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.dupe(Float, &data), 2, 2, .row_major);
    defer matrix.destroy();

    var inverse = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.dupe(Float, &data), 2, 2, .row_major);
    defer inverse.destroy();
    try inverse.invert();

    var product = try TestMatrix.create(std.testing.allocator, try std.testing.allocator.alloc(Float, 4), 2, 2, .row_major);
    defer product.destroy();
    try matrix.multiplication(&inverse, &product);

    for (expected, product.data) |expected_value, actual_value| {
        try std.testing.expectApproxEqAbs(expected_value, actual_value, 0.000001);
    }
}

test "pinv handles non-square matrix" {
    const Float = f64;
    const TestMatrix = Matrix(Float);

    const r = 3;
    const c = 2;
    var data = [r * c]Float{ 1, 0, 0, 1, 1, 1 };

    const d = try std.testing.allocator.dupe(Float, &data);

    const expected = [r * c]Float{ 2.0 / 3.0, -1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0 };

    var m = try TestMatrix.create(std.testing.allocator, d, r, c, .row_major);
    defer m.destroy();
    try m.invert();

    const pinv = m.data;

    for (0..pinv.len) |i| {
        try std.testing.expectApproxEqRel(expected[i], pinv[i], 0.000001);
    }
}

test "invert handles square matrix" {
    const Float = f32;
    const TestMatrix = Matrix(Float);

    const r = 2;
    const c = 2;
    var data = [r * c]Float{
        4, 7,
        2, 6,
    };

    const d = try std.testing.allocator.dupe(Float, &data);

    const expected = [r * c]Float{
        6.0 / 10.0,  -7.0 / 10.0,
        -2.0 / 10.0, 4.0 / 10.0,
    };

    var m = try TestMatrix.create(std.testing.allocator, d, r, c, .row_major);
    defer m.destroy();
    try m.invert();

    const pinv = m.data;

    for (0..pinv.len) |i| {
        try std.testing.expectApproxEqRel(expected[i], pinv[i], 0.000001);
    }
}

test "is_singular returns true for linearly dependent rows" {
    const Float = f64;
    const TestMatrix = Matrix(Float);

    const data = try std.testing.allocator.dupe(Float, &.{
        1, 2,
        2, 4,
    });
    var matrix = try TestMatrix.create(std.testing.allocator, data, 2, 2, .row_major);
    defer matrix.destroy();

    try std.testing.expect(try matrix.is_singular());
    try std.testing.expectEqualSlices(Float, &.{ 1, 2, 2, 4 }, matrix.data);
}

test "is_singular returns false for an invertible matrix requiring pivoting" {
    const Float = f32;
    const TestMatrix = Matrix(Float);

    const data = try std.testing.allocator.dupe(Float, &.{
        0, 1,
        1, 0,
    });
    var matrix = try TestMatrix.create(std.testing.allocator, data, 2, 2, .row_major);
    defer matrix.destroy();

    try std.testing.expect(!try matrix.is_singular());
    try std.testing.expectEqualSlices(Float, &.{ 0, 1, 1, 0 }, matrix.data);
}


