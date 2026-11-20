const std = @import("std");
const BoundingBox = @import("AKAZE.zig").BoundingBox;

pub const rgb_pixel_len = 3;

pub const CropOrigin = struct{
    x: usize,
    y: usize,
};


pub fn Frame(T: type) type{
    return struct{
        const Self = @This();
        height: usize,
        width: usize,
        stride: usize,
        data: []T,


        pub fn init(height: usize, width: usize, stride: usize, data: []T) Self{
            return .{
                .height = height,
                .width = width,
                .stride = stride,
                .data = data
            };
        }

        pub fn deinit(self: *Self, smp: std.mem.Allocator) void{
            smp.free(self.data);
        }
    };
}

pub inline fn copy_to_frame(T: type, f: *Frame(T), buffer: []T) void{
    @memcpy(f.data, buffer);
}


pub fn translate_box_to_frame(bb: *BoundingBox, origin: CropOrigin) void{
    if(!bb.valid) return;
    const offset_x: f32 = @floatFromInt(origin.x);
    const offset_y: f32 = @floatFromInt(origin.y);
    for([_]*@TypeOf(bb.ul){&bb.ul, &bb.ur, &bb.lr, &bb.ll, &bb.centroid}) |point|{
        point.x += offset_x;
        point.y += offset_y;
    }
}


pub fn copy_region(f: *Frame(u8), region: *Frame(u8), bb: *const BoundingBox, expand_by: usize) !CropOrigin{
    if(!bb.valid) return error.InvalidBoundingBox;
    if(f == region) return error.InvalidRegion;
    if(f.width == 0 or f.height == 0) return error.InvalidFrame;

    const frame_stride = try std.math.mul(usize, f.width, rgb_pixel_len);
    const frame_size = try std.math.mul(usize, f.height - 1, f.stride);
    const required_size = try std.math.add(usize, frame_size, frame_stride);
    if(f.stride < frame_stride or f.data.len < required_size) return error.InvalidFrame;
    const region_capacity = try std.math.mul(usize, f.height, frame_stride);
    if(region.data.len < region_capacity) return error.RegionTooSmall;

    var min_x: f64 = std.math.inf(f64);
    var min_y: f64 = std.math.inf(f64);
    var max_x: f64 = -std.math.inf(f64);
    var max_y: f64 = -std.math.inf(f64);
    for([_]@TypeOf(bb.ul){bb.ul, bb.ur, bb.lr, bb.ll}) |point|{
        if(!std.math.isFinite(point.x) or !std.math.isFinite(point.y)) return error.InvalidBoundingBox;
        min_x = @min(min_x, point.x);
        min_y = @min(min_y, point.y);
        max_x = @max(max_x, point.x);
        max_y = @max(max_y, point.y);
    }

    const padding: f64 = @floatFromInt(expand_by);
    const frame_width: f64 = @floatFromInt(f.width);
    const frame_height: f64 = @floatFromInt(f.height);
    const x_start: usize = @intFromFloat(std.math.clamp(@floor(min_x) - padding, 0, frame_width));
    const y_start: usize = @intFromFloat(std.math.clamp(@floor(min_y) - padding, 0, frame_height));
    const x_end: usize = @intFromFloat(std.math.clamp(@floor(max_x) + 1 + padding, 0, frame_width));
    const y_end: usize = @intFromFloat(std.math.clamp(@floor(max_y) + 1 + padding, 0, frame_height));
    if(x_start >= x_end or y_start >= y_end) return error.EmptyRegion;

    const region_width = x_end - x_start;
    const region_height = y_end - y_start;
    const region_stride = region_width * rgb_pixel_len;
    const data = region.data;
    for(0..region_height) |row|{
        const source_start = (y_start + row) * f.stride + x_start * rgb_pixel_len;
        const destination_start = row * region_stride;
        @memcpy(data[destination_start..][0..region_stride], f.data[source_start..][0..region_stride]);
    }
    region.* = .init(region_height, region_width, region_stride, data);
    return .{.x = x_start, .y = y_start};
}


fn make_test_box() BoundingBox {
    return .{
        .ul = .{.x = 1, .y = 1},
        .ur = .{.x = 2, .y = 1},
        .ll = .{.x = 1, .y = 2},
        .lr = .{.x = 2, .y = 2},
        .centroid = .{.x = 1.5, .y = 1.5},
        .valid = true,
        .crit = 0,
        .rejection_value = 0,
    };
}

test "RGB crop respects padded source rows and includes corner pixels" {
    var pixels: [56]u8 = undefined;
    for(&pixels, 0..) |*pixel, index| pixel.* = @intCast(index);
    var frame = Frame(u8).init(4, 4, 14, &pixels);
    var region_pixels = [_]u8{255} ** 48;
    var region = Frame(u8).init(4, 4, 12, &region_pixels);
    const box = make_test_box();
    const origin = try copy_region(&frame, &region, &box, 0);
    try std.testing.expectEqual(CropOrigin{.x = 1, .y = 1}, origin);
    try std.testing.expectEqual(@as(usize, 2), region.width);
    try std.testing.expectEqual(@as(usize, 2), region.height);
    try std.testing.expectEqual(@as(usize, 6), region.stride);
    try std.testing.expectEqualSlices(u8, pixels[17..23], region.data[0..6]);
    try std.testing.expectEqualSlices(u8, pixels[31..37], region.data[6..12]);
    try std.testing.expectEqual(@as(usize, 48), region.data.len);
    try std.testing.expect(region.data.ptr == region_pixels[0..].ptr);
    try std.testing.expectEqualSlices(u8, &([_]u8{255} ** 36), region.data[12..]);
    region.data[0] = 255;
    try std.testing.expectEqual(@as(u8, 17), pixels[17]);
}

test "padding clamps to frame edges and reuses the fixed region buffer" {
    var pixels: [56]u8 = undefined;
    for(&pixels, 0..) |*pixel, index| pixel.* = @intCast(index);
    var frame = Frame(u8).init(4, 4, 14, &pixels);
    var region_pixels: [48]u8 = undefined;
    var region = Frame(u8).init(4, 4, 12, &region_pixels);
    const box = make_test_box();
    _ = try copy_region(&frame, &region, &box, 0);
    const origin = try copy_region(&frame, &region, &box, std.math.maxInt(usize));
    try std.testing.expectEqual(CropOrigin{.x = 0, .y = 0}, origin);
    try std.testing.expectEqual(@as(usize, 4), region.width);
    try std.testing.expectEqual(@as(usize, 4), region.height);
    for(0..4) |row| {
        try std.testing.expectEqualSlices(u8, pixels[row * 14..][0..12], region.data[row * 12..][0..12]);
    }
    _ = try copy_region(&frame, &region, &box, 0);
    try std.testing.expectEqual(@as(usize, 2), region.width);
    try std.testing.expectEqual(@as(usize, 2), region.height);
    try std.testing.expectEqual(@as(usize, 48), region.data.len);
    try std.testing.expect(region.data.ptr == region_pixels[0..].ptr);
}

test "invalid boxes return an error without changing region and nonfinite coordinates are rejected" {
    var pixels = [_]u8{0} ** 48;
    var frame = Frame(u8).init(4, 4, 12, &pixels);
    var region_pixels = [_]u8{255} ** 48;
    var region = Frame(u8).init(4, 4, 12, &region_pixels);
    var box = make_test_box();
    box.valid = false;
    try std.testing.expectError(error.InvalidBoundingBox, copy_region(&frame, &region, &box, 1));
    try std.testing.expectEqual(@as(usize, 48), region.data.len);
    try std.testing.expectEqual(@as(usize, 4), region.width);
    try std.testing.expectEqual(@as(usize, 4), region.height);
    try std.testing.expectEqual(@as(usize, 12), region.stride);
    try std.testing.expectEqualSlices(u8, &([_]u8{255} ** 48), region.data);
    box.valid = true;
    box.ul.x = std.math.nan(f32);
    try std.testing.expectError(error.InvalidBoundingBox, copy_region(&frame, &region, &box, 0));
}

test "rotated boxes use all corners and reject short source buffers" {
    var pixels = [_]u8{0} ** 48;
    var frame = Frame(u8).init(4, 4, 12, &pixels);
    var region_pixels: [48]u8 = undefined;
    var region = Frame(u8).init(4, 4, 12, &region_pixels);
    var box = make_test_box();
    box.ul = .{.x = 1.5, .y = 0};
    box.ur = .{.x = 3, .y = 1.5};
    box.lr = .{.x = 1.5, .y = 3};
    box.ll = .{.x = 0, .y = 1.5};
    _ = try copy_region(&frame, &region, &box, 0);
    try std.testing.expectEqual(@as(usize, 4), region.width);
    try std.testing.expectEqual(@as(usize, 4), region.height);
    frame.data = pixels[0..47];
    try std.testing.expectError(error.InvalidFrame, copy_region(&frame, &region, &box, 0));
}

test "undersized region buffers are rejected without changing the destination" {
    var pixels = [_]u8{0} ** 48;
    var frame = Frame(u8).init(4, 4, 12, &pixels);
    var region_pixels = [_]u8{255} ** 47;
    var region = Frame(u8).init(4, 4, 12, &region_pixels);
    const box = make_test_box();
    try std.testing.expectError(error.RegionTooSmall, copy_region(&frame, &region, &box, 0));
    try std.testing.expectEqual(@as(usize, 4), region.width);
    try std.testing.expectEqual(@as(usize, 4), region.height);
    try std.testing.expectEqual(@as(usize, 12), region.stride);
    try std.testing.expectEqualSlices(u8, &([_]u8{255} ** 47), region.data);
}

test "crop translation moves every corner and centroid while preserving metadata" {
    var box = make_test_box();
    box.rejection_value = 0.25;
    var expected = box;
    expected.ul = .{.x = 11, .y = 21};
    expected.ur = .{.x = 12, .y = 21};
    expected.ll = .{.x = 11, .y = 22};
    expected.lr = .{.x = 12, .y = 22};
    expected.centroid = .{.x = 11.5, .y = 21.5};
    translate_box_to_frame(&box, .{.x = 10, .y = 20});
    try std.testing.expectEqualDeep(expected, box);
}

test "crop translation leaves invalid boxes and zero-origin boxes unchanged" {
    var box = make_test_box();
    const original = box;
    translate_box_to_frame(&box, .{.x = 0, .y = 0});
    try std.testing.expectEqualDeep(original, box);
    box.valid = false;
    const invalid = box;
    translate_box_to_frame(&box, .{.x = 10, .y = 20});
    try std.testing.expectEqualDeep(invalid, box);
}
