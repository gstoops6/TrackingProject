pub const ui = @import("GUI/UI.zig");
//pub const dispbuf = @import("GUI/DisplayBuffer.zig");
pub const rl = @import("raylib");
pub const rg = @import("raygui");

pub const Settings = @import("root").Settings;
pub const debug_mode = @import("root").debug_mode;

pub const default_max_radius: f32 = 50;
pub const default_max_rand: i32 = 16384;

//GUI Settings Constants
pub const min_frames: f32 = 2;
pub var max_frames: f32 = 128;
pub const min_radius: f32 = 1;
pub var max_radius: f32 = 50;
pub const min_mainfloor: f32 = 0;
pub const max_mainfloor: f32 = 255;
pub const min_mainceil: f32 = 0;
pub const max_mainceil: f32 = 255;
pub var max_rand: i32 = 16384;
pub const min_rand: i32 = 1;

//GUI UI Constants
pub const margin: f32 = 10;
pub const gap: f32 = 12;
pub const group_padding: f32 = 8;
pub const right_panel_width: f32 = 500;
pub const viewer_aspect_ratio: f32 = 4.0 / 3.0;

pub const AppState = struct {
    show_track: bool = false,
    track_ready: bool = false,
    freeze: bool = false,
    region_start: ?rl.Vector2 = null,
    region_end: ?rl.Vector2 = null,
    region_set: bool = false,
    drag_start: ?rl.Vector2 = null,
    drag_end: ?rl.Vector2 = null,
    pending_bounding_box: ?@import("VideoTracker.zig").BoundingBox = null,
};
