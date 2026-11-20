const gui = @import("../GUI.zig");
const rg = gui.rg;
const rl = gui.rl;
const root = @import("root");
const std = @import("std");

const AppState = gui.AppState;
const BoundingBox = @import("../VideoTracker.zig").BoundingBox;
const opencvIMM = @import("../VideoTracker.zig").opencv.IMM;

pub fn init_state() AppState {
    return .{};
}

/// Converts a screen point into normalized coordinates within the displayed image.
fn screen_point_to_frame_point(point: rl.Vector2, frame: rl.Rectangle) rl.Vector2 {
    return .{
        .x = @min(@max((point.x - frame.x) / frame.width, 0.0), 1.0),
        .y = @min(@max((point.y - frame.y) / frame.height, 0.0), 1.0),
    };
}

fn bounding_box_from_drag(start: rl.Vector2, end: rl.Vector2, width: c_int, height: c_int) ?BoundingBox {
    if (width <= 0 or height <= 0) return null;
    const frame_width: f32 = @floatFromInt(width);
    const frame_height: f32 = @floatFromInt(height);
    if (@abs(end.x - start.x) * frame_width < 1.0 or
        @abs(end.y - start.y) * frame_height < 1.0) return null;

    const left = @max(0, @floor(@min(start.x, end.x) * frame_width));
    const top = @max(0, @floor(@min(start.y, end.y) * frame_height));
    const right = @min(frame_width - 1, @ceil(@max(start.x, end.x) * frame_width) - 1);
    const bottom = @min(frame_height - 1, @ceil(@max(start.y, end.y) * frame_height) - 1);
    if (left >= right or top >= bottom) return null;

    return .{
        .ul = .{.x = left, .y = top},
        .ur = .{.x = right, .y = top},
        .ll = .{.x = left, .y = bottom},
        .lr = .{.x = right, .y = bottom},
        .centroid = .{.x = (left + right) / 2, .y = (top + bottom) / 2},
        .valid = true,
        .crit = 0,
        .rejection_value = 0,
    };
}

/// Left-drag selects the tracking box; right-click clears the selection.
fn update_mouse_drag(state: *AppState, frame: rl.Rectangle, width: c_int, height: c_int) void {
    const mouse = rl.getMousePosition();
    const inside = rl.checkCollisionPointRec(mouse, frame);

    if (inside and rl.isMouseButtonPressed(.right)) {
        state.region_start = null;
        state.region_end = null;
        state.region_set = false;
        state.drag_start = null;
        state.drag_end = null;
        state.pending_bounding_box = null;
        state.track_ready = false;
        return;
    }

    if (inside and rl.isMouseButtonPressed(.left)) {
        state.drag_start = screen_point_to_frame_point(mouse, frame);
    }

    const start = state.drag_start orelse return;
    const end = screen_point_to_frame_point(mouse, frame);
    state.drag_end = end;

    if (rl.isMouseButtonReleased(.left)) {
        if (bounding_box_from_drag(start, end, width, height)) |box| {
            state.pending_bounding_box = box;
            state.region_start = .{.x = @min(start.x, end.x), .y = @min(start.y, end.y)};
            state.region_end = .{.x = @max(start.x, end.x), .y = @max(start.y, end.y)};
            state.region_set = true;
        }

        state.drag_start = null;
        state.drag_end = null;
    }
}

fn draw_dragged_rectangle(state: *const AppState, frame: rl.Rectangle) void {
    const start = state.drag_start orelse return;
    const end = state.drag_end orelse return;
    rl.drawRectangleLinesEx(.{
        .x = frame.x + @min(start.x, end.x) * frame.width,
        .y = frame.y + @min(start.y, end.y) * frame.height,
        .width = @abs(end.x - start.x) * frame.width,
        .height = @abs(end.y - start.y) * frame.height,
    }, 1, .red);
}

/// Fits the image into the viewer without changing its aspect ratio.
fn fit_texture_rect(bounds: rl.Rectangle, width: c_int, height: c_int) rl.Rectangle {
    const aspect_ratio = @as(f32, @floatFromInt(width)) / @as(f32, @floatFromInt(height));
    var fitted_width = bounds.width;
    var fitted_height = fitted_width / aspect_ratio;

    if (fitted_height > bounds.height) {
        fitted_height = bounds.height;
        fitted_width = fitted_height * aspect_ratio;
    }

    return .{
        .x = bounds.x + (bounds.width - fitted_width) / 2,
        .y = bounds.y + (bounds.height - fitted_height) / 2,
        .width = fitted_width,
        .height = fitted_height,
    };
}

fn draw_bounding_box(bounding_box: BoundingBox, texture: rl.Texture, image_frame: rl.Rectangle) void {
    if (!bounding_box.valid and bounding_box.matcher != opencvIMM) return;

    //if(bounding_box.matcher == opencvIMM) std.debug.print("Displaying IMM Bounding Box\n", .{});

    const frame_width: f32 = @floatFromInt(texture.width);
    const frame_height: f32 = @floatFromInt(texture.height);
    const corners = .{ bounding_box.ul, bounding_box.ur, bounding_box.lr, bounding_box.ll };
    var points: [4]rl.Vector2 = undefined;

    inline for (corners, 0..) |corner, i| {
        points[i] = .{
            .x = image_frame.x + corner.x / frame_width * image_frame.width,
            .y = image_frame.y + corner.y / frame_height * image_frame.height,
        };
    }

    rl.beginScissorMode(
        @intFromFloat(image_frame.x),
        @intFromFloat(image_frame.y),
        @intFromFloat(image_frame.width),
        @intFromFloat(image_frame.height),
    );
    defer rl.endScissorMode();

    for (points, 0..) |point, i| {
        rl.drawLineEx(point, points[(i + 1) % points.len], 2, .green);
    }
}

fn draw_viewer_pane(bounds: rl.Rectangle, state: *AppState, frame_texture: ?rl.Texture, bounding_box: ?BoundingBox) void {
    _ = rg.groupBox(bounds, "Viewer");

    const padding = gui.group_padding;
    const x_bound = bounds.x + bounds.width - padding - 88;
    const y_bound = bounds.y + 12;
    _ = rg.toggle(
        .init(x_bound, y_bound, 88, 22),
        "show track",
        &state.show_track,
    );
    _ = rg.toggle(
        .init(x_bound - gui.gap - 88, y_bound, 88, 22),
        "freeze",
        &state.freeze,
    );

    rl.drawFPS(@intFromFloat(bounds.x + 100), @intFromFloat(bounds.y + 11));

    const surface = rl.Rectangle.init(
        bounds.x + padding,
        bounds.y + 44,
        bounds.width - padding * 2,
        bounds.height - 44 - padding,
    );
    if (surface.width <= 0 or surface.height <= 0) return;
    rl.drawRectangleRec(surface, .{ .r = 10, .g = 12, .b = 18, .a = 255 });

    if (frame_texture) |texture| {
        if (texture.width > 0 and texture.height > 0 and rl.isTextureValid(texture)) {
            _ = rg.label(
                .init(bounds.x + padding, bounds.y + 12, @max(0, bounds.width - padding * 2 - 188 - gui.gap), 22),
                if (state.track_ready)
                    rl.textFormat("%d x %d", .{ texture.width, texture.height })
                else
                    "Drag a rectangle to select the tracking object",
            );

            const image_frame = fit_texture_rect(surface, texture.width, texture.height);
            texture.drawPro(
                .init(0, 0, @floatFromInt(texture.width), @floatFromInt(texture.height)),
                image_frame,
                .init(0, 0),
                0,
                .ray_white,
            );
            if (state.show_track) {
                if (bounding_box) |box| draw_bounding_box(box, texture, image_frame);
            }
            update_mouse_drag(state, image_frame, texture.width, texture.height);
            draw_dragged_rectangle(state, image_frame);
            return;
        }
    }

    _ = rg.label(.init(bounds.x + padding, bounds.y + 12, 120, 22), "No image");
}

/// Draws the viewer, resolution, and the track visibility and freeze toggles.
/// The caller owns the texture and keeps it valid for the duration of this call.
pub fn draw_gui(state: *AppState, background_color: i32, frame_texture: ?rl.Texture, bounding_box: ?BoundingBox) void {
    rl.beginDrawing();
    defer rl.endDrawing();
    rl.clearBackground(rl.getColor(@bitCast(background_color)));

    const screen_width: f32 = @floatFromInt(rl.getScreenWidth());
    const screen_height: f32 = @floatFromInt(rl.getScreenHeight());
    draw_viewer_pane(.init(
        gui.margin,
        gui.margin,
        screen_width - gui.margin * 2,
        screen_height - gui.margin * 2,
    ), state, frame_texture, bounding_box);
}
