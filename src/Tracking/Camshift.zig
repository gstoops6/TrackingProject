const Self = @This();
const std = @import("std");
const tracking = @import("../VideoTracker.zig");
const frame_op = @import("FrameOperations.zig");
const BoundingBox = tracking.BoundingBox;
const Frame = frame_op.Frame;


color: Frame(u8),
region: Frame(u8),
smp: std.mem.Allocator,
bounding_box: BoundingBox,
tracker: ?*tracking.CamshiftTracker = null,




