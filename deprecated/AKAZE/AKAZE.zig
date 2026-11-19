const Self = @This();
const root = @import("root");
const std = @import("std");
const frame_op = @import("FrameOperations.zig");
pub const ak = @cImport({
    @cInclude("OpenCVAKAZE.h");
});

pub const Frame = frame_op.Frame;
pub const FrameDescriptor = ak.FrameDescriptor;
pub const FeatureMatch = ak.FeatureMatch;
pub const BoundingBox = ak.BoundingBox;
const CropOrigin = frame_op.CropOrigin;

const destroy = ak.akaze_destroy;
const print = std.debug.print;
const compute_akaze = ak.akaze_compute;
const copy_to_frame = frame_op.copy_to_frame;
const copy_region = frame_op.copy_region;
const translate_box_to_frame = frame_op.translate_box_to_frame;
const rgb_pixel_len = frame_op.rgb_pixel_len;

color: Frame(u8),
region: Frame(u8),
smp: std.mem.Allocator,
bounding_box: BoundingBox,
tracker: ?*ak.AkazeTracker = null,

pub fn init(smp: std.mem.Allocator, first_frame: []u8, bb: BoundingBox, width: usize, height: usize) !Self{

    const color_stride = width * rgb_pixel_len;
    const color = try smp.alloc(u8, height * color_stride);
    const region = try smp.alloc(u8, height * color_stride);


    var s: Self =  .{
        .color = .init(height, width, color_stride, color),
        .region = .init(height, width, color_stride, region),
        .smp = smp,
        .bounding_box = bb
    };
    s.update(first_frame);
    try s.create_tracker();
    return s;
}

pub fn deinit(self: *Self) void{
    self.color.deinit(self.smp);
    self.region.deinit(self.smp);
    if(self.tracker) |t| destroy(t);
}

pub fn update(self: *Self, data: []u8) void{
    copy_to_frame(u8, &self.color, data);
}

pub fn create_tracker(self: *Self) !void{
    const desc = to_descriptor(&self.color);
    self.tracker = ak.akaze_init(&desc, &self.bounding_box) orelse return error.TrackerInitializationFailed;
}

pub fn compute(self: *Self) !void{
    const crop_origin: ?frame_op.CropOrigin = copy_region(
        &self.color,
        &self.region,
        &self.bounding_box,
        6
    ) catch null;

    var desc = to_descriptor(if(crop_origin != null) &self.region else &self.color);
    const t = self.tracker orelse return error.NullTracker;
    self.bounding_box = compute_akaze(t, &desc);
    if(crop_origin) |origin| translate_box_to_frame(&self.bounding_box, origin);
}

pub fn get_bounding_box(self: *Self) BoundingBox{
    return self.bounding_box;
}

fn to_descriptor(f: *Frame(u8)) FrameDescriptor{
    return .{
        .height = f.height,
        .width = f.width,
        .stride = f.stride,
        .data = @ptrCast(f.data.ptr)
    };
}

