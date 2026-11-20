const std = @import("std");
const BoundingBox = @import("../VideoTracker.zig").BoundingBox;
const bbPoint = @import("../VideoTracker.zig").bbPoint;

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

        pub fn clone(self: *Self, smp: std.mem.Allocator) !Self{
            return .{
                .height = self.height,
                .width = self.width,
                .stride = self.stride,
                .data = try smp.dupe(T, self.data)
            };
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
    for([_]*bbPoint{&bb.ul, &bb.ur, &bb.lr, &bb.ll, &bb.centroid}) |point|{
        point.x += offset_x;
        point.y += offset_y;
    }
}


/// expand_by scales the crop width and height about its center. 1 means no expansion.
pub fn copy_region(f: *Frame(u8), region: *Frame(u8), bb: *const BoundingBox, expand_by: f64) !CropOrigin{
    if(!bb.valid) return error.InvalidBoundingBox;
    if(!std.math.isFinite(expand_by) or expand_by < 1) return error.InvalidExpansionFactor;
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

    const crop_min_x = @floor(min_x);
    const crop_min_y = @floor(min_y);
    const crop_max_x = @floor(max_x) + 1;
    const crop_max_y = @floor(max_y) + 1;
    const padding_x = (expand_by - 1) * (crop_max_x - crop_min_x) / 2;
    const padding_y = (expand_by - 1) * (crop_max_y - crop_min_y) / 2;
    const frame_width: f64 = @floatFromInt(f.width);
    const frame_height: f64 = @floatFromInt(f.height);
    const x_start: usize = @intFromFloat(std.math.clamp(@floor(crop_min_x - padding_x), 0, frame_width));
    const y_start: usize = @intFromFloat(std.math.clamp(@floor(crop_min_y - padding_y), 0, frame_height));
    const x_end: usize = @intFromFloat(std.math.clamp(@ceil(crop_max_x + padding_x), 0, frame_width));
    const y_end: usize = @intFromFloat(std.math.clamp(@ceil(crop_max_y + padding_y), 0, frame_height));
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


