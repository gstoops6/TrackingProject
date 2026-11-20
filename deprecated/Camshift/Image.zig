const Self = @This();
const std = @import("std");
const root = @import("root");
const print = std.debug.print;

const vector_length = std.simd.suggestVectorLength(f32) orelse 1;
const VecF32 = @Vector(vector_length, f32);
const VecU8 = @Vector(vector_length, u8);
pub const rgb_pixel_len = 3;
const hsv_pixel_len = 3;
const rgb_weights: [3]f32 = .{ 0.2126, 0.7152, 0.0722 };

const InterleaveOperation = enum {
    interleave,
    deinterleave,
};


/// Only works for RGB images / video frames
smp: std.mem.Allocator,
width: usize,
height: usize,
stride: usize,
data: []u8,
data_deinterleaved: []u8,
data_gray: []u8,
gray_histogram: []usize,
data_hue: []f32,

/// Width and height are the frame size not the height and stride. All frames are assumed to be RGB
/// Width and height should be the total size of the frame
pub fn init(smp: std.mem.Allocator, width: usize, height: usize) !Self {
    const frame_size = width * height * rgb_pixel_len;
    return .{
        .smp = smp,
        .height = height,
        .width = width,
        .stride = width * rgb_pixel_len,
        .data = try smp.alloc(u8, frame_size),
        .data_deinterleaved = try smp.alloc(u8, frame_size),
        .data_gray = try smp.alloc(u8, width * height),
        .gray_histogram = try smp.alloc(usize, 256),
        .data_hue = try smp.alloc(f32, width * height)
    };
}

pub fn deinit(self: *Self) void {
    self.smp.free(self.data);
    self.smp.free(self.data_deinterleaved);
    self.smp.free(self.data_gray);
    self.smp.free(self.data_hue);
    self.smp.free(self.gray_histogram);
}

pub fn update(self: *Self, buffer: []u8, width: usize, height: usize) !void {
    if(width * height * rgb_pixel_len != buffer.len) return error.BufferMetadataMismatch;

    if(height != self.height or width != self.width){
        self.data = try self.smp.realloc(self.data, buffer.len);
        self.data_deinterleaved = try self.smp.realloc(self.data_deinterleaved, buffer.len);
        self.data_gray = try self.smp.realloc(self.data_gray, width * height);
        self.data_hue = try self.smp.realloc(self.data_hue, width * height);
        self.width = width;
        self.height = height;
        self.stride = width * rgb_pixel_len;
    }

    @memcpy(self.data, buffer);

    self.interleave(.deinterleave);
    self.rgb_to_gray();
    self.rgb_to_hue();
}


pub fn pad_gray(self: *Self, padding: usize) ![]u8 {
    const new_width = self.width + (2 * padding);
    const new_height = self.height + (2 * padding);

    const new_frame = try self.smp.alloc(u8, new_height * new_width);
    @memset(new_frame, 0);

    for (0..self.height) |x| {
        for (0..self.width) |y| {
            const new_idx: usize = (x + padding) * new_width + (y + padding);
            const src_idx: usize = x * self.width + y;
            new_frame[new_idx] = self.data_gray[src_idx];
        }
    }
    return new_frame;
}

pub fn interleave(self: *Self, comptime op: InterleaveOperation) void {
    const image_size = self.height * self.width;

    for (0..image_size) |i| {
        const outer_idx = rgb_pixel_len * i;
        inline for (0..rgb_pixel_len) |j| {
            const inner_idx = (j * image_size) + i;
            if (op == .interleave) {
                self.data_deinterleaved[outer_idx + j] = self.data[inner_idx];
            } else if (op == .deinterleave) {
                self.data_deinterleaved[inner_idx] = self.data[outer_idx + j];
            }
        }
    }
}

pub fn histogram(self: *Self) !void{
    @memset(self.gray_histogram, 0);
    for (0..self.data_gray.len) |i| {
        self.gray_histogram[@intCast(self.data_gray[i])] += 1;
    }
}

pub fn rgb_to_gray(self: *Self) void {
    const rgb_buf = self.data_deinterleaved;

    const frame_size = self.height * self.width;

    const r_weight: VecF32 = @splat(rgb_weights[0]);
    const g_weight: VecF32 = @splat(rgb_weights[1]);
    const b_weight: VecF32 = @splat(rgb_weights[2]);

    const length = self.data_gray.len;
    const iters: usize = @divFloor(length, vector_length);

    var i: usize = 0;
    var idx_end: usize = 0;
    while (i < iters) : (i += 1) {
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

        self.data_gray[idx_start..][0..vector_length].* = gray_u8;
        idx_end = idx_start + vector_length;
    }

    var pixel: [rgb_pixel_len]u8 = .{0} ** rgb_pixel_len;

    for (idx_end..self.data_gray.len) |j| {
        inline for (0..rgb_pixel_len) |k| {
            pixel[k] = rgb_buf[(k * frame_size) + j];
        }
        self.data_gray[j] = rgb_pixel_to_gray(&pixel);
    }
}

inline fn rgb_pixel_to_gray(pixel: []u8) u8 {
    var pix: f32 = 0;

    inline for (0..rgb_weights.len) |i| {
        pix += rgb_weights[i] * @as(f32, @floatFromInt(pixel[i]));
    }
    return @intFromFloat(@round(pix));
}


/// Conversions require the pixels be planar not packed.
pub fn rgb_to_hue(self: *Self) void{
    const MaxComponent = enum{
        red,
        green,
        blue
    };

    const frame_size = self.width * self.height;

    for(0..frame_size)|i|{
        const r = @as(f32, @floatFromInt(self.data_deinterleaved[i])) / 255.0;
        const g = @as(f32, @floatFromInt(self.data_deinterleaved[frame_size + i])) / 255.0;
        const b = @as(f32, @floatFromInt(self.data_deinterleaved[(2 * frame_size) + i])) / 255.0;

        const max_value: f32 = @max(r, g, b);
        const min_value: f32 = @min(r, g, b);
        const diff = max_value - min_value;

        if(diff == 0){
            self.data_hue[i] = 0;
            continue;
        }

        const max_component: MaxComponent = blk: {
            if(max_value == r) {
                break :blk .red;
            }else if(max_value == g) {
                break :blk .green;
            }else {
                break :blk .blue;
            }
        };

        self.data_hue[i] = switch(max_component){
            .red => (60 * (g - b)) / diff,
            .green => 120 + (60 * (b - r) / diff),
            .blue => 240 + (60 * (r - g) / diff)
        };
        // Wrap negative red-sector angles into [0, 360).
        if (self.data_hue[i] < 0) self.data_hue[i] += 360.0;
    }
}
