const std = @import("std");
const assert = std.debug.assert;
const print = std.debug.print;


const vector_length = std.simd.suggestVectorLength(f32) orelse 1;
const VecF32 = @Vector(vector_length, f32);
const VecU8 = @Vector(vector_length, u8);
const rgb_pixel_len = 3;
const hsv_pixel_len = 3;
const rgb_weights: [3]f32 = .{0.2126, 0.7152, 0.0722};



const InterleaveOperation = enum{
    interleave,
    deinterleave,
};

fn interleave(smp: std.mem.Allocator, input_buffer: []u8, height: usize, width: usize, comptime op: InterleaveOperation) ![]u8{
    const output_buffer = try smp.alloc(u8, input_buffer.len);
    const frame_size = height * width;

    for(0..frame_size)|i|{
        const outer_idx = rgb_pixel_len * i;
        inline for(0..rgb_pixel_len)|j|{
            const inner_idx = (j * frame_size) + i;
            if(op == .interleave){
                output_buffer[outer_idx + j] = input_buffer[inner_idx];
            } else if(op == .deinterleave){
                output_buffer[inner_idx] = input_buffer[outer_idx + j];
            }
        }
    }
    return output_buffer;
}





pub fn rgb_to_gray(rgb_buf: []u8, gray_buf: []u8, height: usize, width: usize) !void{
    assert(width * height == gray_buf.len);
    assert(height * width * rgb_pixel_len == rgb_buf.len);

    const frame_size = width * height;

    const r_weight: VecF32 = @splat(rgb_weights[0]);
    const g_weight: VecF32 = @splat(rgb_weights[1]);
    const b_weight: VecF32 = @splat(rgb_weights[2]);

    const length = gray_buf.len;
    const iters: usize = @divFloor(length, vector_length);

    var i: usize = 0;
    var idx_end: usize = 0;
    while(i < iters) : (i += 1){
        const idx_start = i * vector_length;

        const g_start = frame_size + idx_start;
        const b_start = (2 * frame_size) + idx_start;

        const r_vec: VecU8 = rgb_buf[idx_start..][0..vector_length].*;
        const g_vec: VecU8 = rgb_buf[g_start..][0..vector_length].*;
        const b_vec: VecU8 = rgb_buf[b_start..][0..vector_length].*;


        const r_weighted: VecF32 = @as(VecF32, @floatFromInt(r_vec)) * r_weight;
        const g_weighted: VecF32 = @as(VecF32, @floatFromInt(g_vec)) * g_weight;
        const b_weighted: VecF32 = @as(VecF32, @floatFromInt(b_vec)) * b_weight;

        const gray_float: VecF32 = r_weighted + g_weighted + b_weighted;
        const gray_u8: VecU8 = @intFromFloat(@round(gray_float));

        gray_buf[idx_start..][0..vector_length].* = gray_u8;
        idx_end = idx_start + vector_length;
    }

    var pixel: [rgb_pixel_len]u8 = .{0} ** rgb_pixel_len;

    for(idx_end..gray_buf.len)|j|{
        inline for(0..rgb_pixel_len)|k|{
            pixel[k] = rgb_buf[(k * frame_size) + j];
        }
        gray_buf[j] = rgb_pixel_to_gray(&pixel);
    }
}

inline fn rgb_pixel_to_gray(pixel: []u8) u8{
    var pix: f32 = 0;

    inline for(0..rgb_weights.len)|i|{
        pix += rgb_weights[i] * @as(f32, @floatFromInt(pixel[i]));
    }
    return @intFromFloat(@round(pix));
}








test "interleave converts between planar and packed RGB across multiple rows" {
    const allocator = std.testing.allocator;
    // A 3-by-2 image stored as separate red, green, and blue planes.
    var planar = [_]u8{
        0,   1,   2,   3,   4,   5,
        10,  11,  12,  13,  14,  15,
        250, 251, 252, 253, 254, 255,
    };
    var packed_rgb = [_]u8{
        0, 10, 250,
        1, 11, 251,
        2, 12, 252,
        3, 13, 253,
        4, 14, 254,
        5, 15, 255,
    };

    const interleaved = try interleave(allocator, &planar, 2, 3, .interleave);
    defer allocator.free(interleaved);
    try std.testing.expectEqualSlices(u8, &packed_rgb, interleaved);

    const deinterleaved = try interleave(allocator, &packed_rgb, 2, 3, .deinterleave);
    defer allocator.free(deinterleaved);
    try std.testing.expectEqualSlices(u8, &planar, deinterleaved);
}

test "rgb_to_hue converts planar RGB to degrees across multiple rows" {
    // A 4-by-3 image: red, yellow, green, cyan, blue, magenta,
    // black, white, gray, orange, pink, and a mixed blue color.
    var rgb_buf = [_]u8{
        255, 255, 0,   0,   0,   255, 0, 255, 128, 255, 255, 12, // Red
        0,   255, 255, 255, 0,   0,   0, 255, 128, 128, 0,   34, // Green
        0,   0,   0,   255, 255, 255, 0, 255, 128, 0,   128, 56, // Blue
    };
    var hue_buf: [12]f32 = undefined;
    const expected = [_]f32{
        0, 60, 120, 180, 240, 300,
        0, 0,  0,   30.117647, 329.882353, 210,
    };

    try rgb_to_hue(&rgb_buf, &hue_buf, 3, 4);

    // Achromatic pixels use the function's zero-hue convention.
    // Fractional angles remain unquantized; negative red-sector angles wrap.
    for (expected, hue_buf) |expected_hue, actual_hue| {
        try std.testing.expectApproxEqAbs(expected_hue, actual_hue, @as(f32, 0.001));
    }
}

test "rgb_to_gray converts planar RGB across multiple rows" {
    // A 4-by-2 image with separate red, green, and blue planes.
    var rgb_buf = [_]u8{
        0, 255, 255, 0,   0,   12, 128, 1, // Red
        0, 255, 0,   255, 0,   34, 128, 1, // Green
        0, 255, 0,   0,   255, 56, 128, 1, // Blue
    };
    var gray_buf: [8]u8 = undefined;

    print("Vector Length: {d}\n", .{vector_length});

    try rgb_to_gray(&rgb_buf, &gray_buf, 2, 4);

    // Sum the weighted channels, then round once to the nearest integer.
    const expected = [_]u8{ 0, 255, 54, 182, 18, 31, 128, 1 };
    try std.testing.expectEqualSlices(u8, &expected, &gray_buf);
}

test "rgb_to_gray handles pixel counts around the selected vector length" {
    const red = [_]u8{ 0, 255, 255, 0, 0, 12, 128, 1 };
    const green = [_]u8{ 0, 255, 0, 255, 0, 34, 128, 1 };
    const blue = [_]u8{ 0, 255, 0, 0, 255, 56, 128, 1 };
    const gray = [_]u8{ 0, 255, 54, 182, 18, 31, 128, 1 };

    // Empty input, a scalar-only input, full vectors, and a remainder.
    // With the one-lane fallback, every nonempty input uses full vectors.
    inline for (.{ 0, vector_length - 1, vector_length, 2 * vector_length, 2 * vector_length + 3 }) |pixel_count| {
        var rgb_buf: [3 * pixel_count]u8 = undefined;
        var gray_buf: [pixel_count]u8 = undefined;
        var expected: [pixel_count]u8 = undefined;

        for (0..pixel_count) |i| {
            const color = i % gray.len;
            rgb_buf[i] = red[color];
            rgb_buf[pixel_count + i] = green[color];
            rgb_buf[2 * pixel_count + i] = blue[color];
            expected[i] = gray[color];
        }

        try rgb_to_gray(&rgb_buf, &gray_buf, 1, pixel_count);
        try std.testing.expectEqualSlices(u8, &expected, &gray_buf);
    }
}


