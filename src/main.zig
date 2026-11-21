const rg = @import("raygui");
const rl = @import("raylib");
const std = @import("std");
const builtin = @import("builtin");
const Optimization = @import("Optimization");
const gui = @import("GUI.zig");
const recording = @import("recording");
const track = @import("VideoTracker.zig");
const frame_op = @import("Tracking/FrameOperations.zig");
pub const config = @import("Config.zig");
const mat = @import("KalmanFilter/Matrix.zig");
const Matrix = mat.Matrix(config.Float);
const IMM = @import("KalmanFilter/IMM.zig");

const json = std.json;
const AppState = gui.AppState;
const DisplayBuffer = gui.dispbuf.DisplayBuffer;
const BoundingBox = track.BoundingBox;
const VideoMemory = recording.VideoMemory;
const Tracker = track.VideoTracker;

pub const tracking_type: MatchingAlgorithm = .KCF;
const debug_mode = false;
const video_format: recording.VideoFormats = .rgb;

const print = std.debug.print;


const FileJSON = struct{
    filename: [:0]const u8
};


const MatchingAlgorithm = enum{
    Camshift,
    Akaze,
    IMM,
    KCF,
    Unknown
};

fn normalize_box(bb: *const BoundingBox) BoundingBox {
    var adjusted: BoundingBox = bb.*;

    adjusted.ll = .{.x = bb.ul.x, .y = bb.ll.y};
    adjusted.lr = .{.x = bb.ur.x, .y = bb.ll.y};

    return adjusted;
}

fn name_matcher(m: c_int) MatchingAlgorithm{
    return switch(m){
        track.opencv.AKAZE => .Akaze,
        track.opencv.CAMSHIFT => .Camshift,
        track.opencv.IMM => .IMM,
        track.opencv.KCF => .KCF,
        else => .Unknown
    };
}

fn name_criteria(criteria: track.opencv.BoxRejectionCriteria) []const u8{
    return switch(criteria){
        track.opencv.Accepted => "Accepted",
        track.opencv.EmptyInlierMask => "EmptyInlierMask",
        track.opencv.EmptyHomography => "EmptyHomography",
        track.opencv.InsufficientMatches => "InsufficientMatches",
        track.opencv.InsufficientValidInlier => "InsufficientValidInlier",
        track.opencv.InsufficientRejectionRatio => "InsufficientRejectionRatio",
        track.opencv.IncorrectBounds => "IncorrectBounds",
        track.opencv.BoxConcave => "BoxConcave",
        track.opencv.BhattacharyyaInsufficient => "BhattacharyyaInsufficient",
        track.opencv.CamshiftHeightRatio => "CamshiftHeightRatio",
        track.opencv.CamshiftWidthRatio => "CamshiftWidthRatio",
        track.opencv.ExceedsWindow => "ExceedsWindow",
        track.opencv.CenterDisplacementExcessive => "CenterDisplacementExcessive",
        track.opencv.OverlappedAreaInsufficient => "OverlappedAreaInsufficient",
        track.opencv.KCFInvalid => "KCFInvalid",
        else => "Unknown"
    };
}


pub fn main(init: std.process.Init) !void {
    const smp = init.gpa;
    const io = init.io;
    var cwd = std.Io.Dir.cwd();

    var fjson = try parse_filename_json(&cwd, smp, io);
    defer fjson.deinit();

    const file_path = fjson.value;

    var video_memory: VideoMemory = undefined;

    if(builtin.os.tag == .macos){
        const abs_path = try cwd.realPathFileAlloc(
            io,
            file_path.filename,
            smp
        );
        print("File Path: {s}\n", .{abs_path});
        defer smp.free(abs_path);

        video_memory = try .init(smp, abs_path, video_format);

    } else {
        video_memory = try .init(smp, file_path.filename, video_format);
    }
    defer video_memory.deinit();


    const window_width: i32 = 1280;
    const window_height: i32 = 720;

    const fps_rational = video_memory.video_stream.?.avg_frame_rate;
    const fps_num: f64 = @floatFromInt(fps_rational.num);
    const fps_den: f64 = @floatFromInt(fps_rational.den);
    const fps = fps_num / fps_den;
    std.debug.print("Video playback at {d} FPS\n", .{@as(usize, @intFromFloat(@round(fps)))});

    rl.setConfigFlags(.{ .window_resizable = true });
    rl.initWindow(window_width, window_height, "Object Tracker");
    rl.setWindowMinSize(1280, 720);
    rl.setTargetFPS(@intFromFloat(fps));
    defer rl.closeWindow();


    const video_basename = std.fs.path.basename(file_path.filename);
    const video_ext = std.fs.path.extension(file_path.filename);
    const video_name = std.mem.trimEnd(u8, video_basename, video_ext);
    const log_name= try std.fmt.allocPrint(smp,"{s}_BoundingBox.log", .{video_name});

    defer smp.free(log_name);

    var log_file = try cwd.createFile(io, log_name, .{});
    defer log_file.close(io);

    var file_buffer: [1024]u8 = .{0} ** 1024;
    var writer = log_file.writerStreaming(io, &file_buffer);
    var string: [1024]u8 = undefined;


    const background_color = rg.getStyle(.default, .{ .default = .background_color });
    var state = gui.ui.init_state();

    const buf = try video_memory.get_frame_buffer(smp, null);
    defer smp.free(buf);

    const image: rl.Image = .{
        .data = buf.ptr,
        .format = if(video_format == .rgb) .uncompressed_r8g8b8 else .uncompressed_grayscale,
        .height = video_memory.metadata.height,
        .width = video_memory.metadata.width,
        .mipmaps = 1
    };

    try video_memory.get_frame(buf);
    const frame_texture = try rl.loadTextureFromImage(image);
    defer rl.unloadTexture(frame_texture);

    var bb: ?BoundingBox = null;
    var tracker: ?Tracker = null;
    defer if (tracker) |*active_tracker| active_tracker.deinit();

    var frame_num: usize = 0;

    while (!rl.windowShouldClose()) {
        gui.ui.draw_gui(
            &state,
            background_color,
            frame_texture,
            bb
        );

        if (state.pending_bounding_box) |selected_box| {
            const next_tracker = try Tracker.init(
                smp,
                io,
                buf,
                selected_box,
                fps,
                @intCast(video_memory.metadata.width),
                @intCast(video_memory.metadata.height)
            );
            if (tracker) |*active_tracker| active_tracker.deinit();
            tracker = next_tracker;
            bb = selected_box;
            state.pending_bounding_box = null;
            state.track_ready = true;
            state.show_track = true;
            continue;
        }

        if (!state.track_ready) {
            if (tracker) |*active_tracker| active_tracker.deinit();
            tracker = null;
            bb = null;
        }

        if (!state.freeze and state.track_ready and state.drag_start == null) {
            const active_tracker = if (tracker) |*value| value else continue;
            try video_memory.get_frame(buf);

            active_tracker.update(buf);
            try active_tracker.compute();

            const tracked_box = active_tracker.get_bounding_box();
            bb = tracked_box;//normalize_box(&tracked_box);

            const line = try std.fmt.bufPrint(
                &string,
                "Frame Number {d}: ul: ({d:.2}, {d:.2}), ur: ({d:.2}, {d:.2}), lr: ({d:.2}, {d:.2}), ll: ({d:.2}, {d:.2}), centroid: ({d:.2}, {d:.2}), valid: {}, matcher: {s}, criteria: {s}, IMM mu: {any}\n",
                .{
                    frame_num,
                    tracked_box.ul.x,
                    tracked_box.ul.y,
                    tracked_box.ur.x,
                    tracked_box.ur.y,
                    tracked_box.lr.x,
                    tracked_box.lr.y,
                    tracked_box.ll.x,
                    tracked_box.ll.y,
                    tracked_box.centroid.x,
                    tracked_box.centroid.y,
                    tracked_box.valid,
                    @tagName(name_matcher(@intCast(tracked_box.matcher))),
                    name_criteria(tracked_box.crit),
                    active_tracker.imm.mu.data
                }
            );

            _ = try writer.interface.write(line);
            try writer.interface.flush();

            frame_num += 1;
            rl.updateTexture(frame_texture, buf.ptr);
        }

    }
}


pub fn parse_filename_json(cwd: *std.Io.Dir, smp: std.mem.Allocator, io: std.Io) !json.Parsed(FileJSON){
    const path = "Filename.json";

    const contents = try cwd.readFileAlloc(io, path, smp, .unlimited);

    return try json.parseFromSlice(FileJSON, smp, contents, .{});
}



//var kf_buffer: [config.num_filters]KalmanFilter = undefined;
//
//
//    var initial_covariance: Matrix = try .empty(smp, config.state_transition_rows, config.state_transition_cols, .row_major);
//    defer initial_covariance.destroy();
//    @memcpy(initial_covariance.data, &config.init_cov);
//    var initial_estimate: Matrix = try .empty(smp, config.state_transition_rows, 1, .row_major);
//    defer initial_estimate.destroy();
//
//    initial_estimate.data[0] = @floatCast(bb.centroid.x);
//    initial_estimate.data[3] = @floatCast(bb.centroid.y);
//
//    try create_kalman_filters(smp, &kf_buffer, @floatCast(1 / fps_frac), &initial_estimate, &initial_covariance);
//
//    var imm = build_imm(smp, &kf_buffer) catch |err| {
//        for (&kf_buffer) |*filter| filter.deinit();
//        return err;
//    };
//    defer imm.deinit();



//pub const Control = struct{
//    smp: std.mem.Allocator,
//    io: std.Io,
//    frame_lock: std.Io.Mutex = .init,
//    recv_ready: std.atomic.Value(bool) = .init(false),
//    box_ready: std.atomic.Value(bool) = .init(false),
//    condition: std.Io.Condition = .init,
//
//    pub fn transfer_frame(self: *Control, akaze: *AKAZE, buffer: []u8) !void{
//        if(self.recv_ready.load(.acquire))
//            try self.frame_lock.lock(self.io);
//            defer self.frame_lock.unlock(self.io);
//
//            akaze.update(buffer);
//    }
//    pub fn get_bounding_box(self: *Control, akaze: *AKAZE, bb: *BoundingBox) !void{
//        if(self.box_ready.load(.acquire)){
//            try self.frame_lock.lock(self.io);
//            defer self.frame_lock.unlock(self.io);
//
//            bb.* = akaze.get_bounding_box();
//        }
//    }
//};

// Create Logging File for Box Points
    //var log_file = try cwd.createFile(io, "BoundingBox.log", .{});
    //defer log_file.close(io);
    //var frame_counter: usize = 0;


    //var file_buffer: [512]u8 = .{0} ** 512;
    //var writer = log_file.writerStreaming(io, &file_buffer);
    //var string: [512]u8 = undefined;


//const line = try std.fmt.bufPrint(
            //    &string,
            //    "Frame Number {d}: ul: ({d:.2}, {d:.2}), ur: ({d:.2}, {d:.2}), lr: ({d:.2}, {d:.2}), ll: ({d:.2}, {d:.2}), centroid: ({d:.2}, {d:.2}), status: {}, rejection criteria: {d}, rejection value: {d:.5}\n\n",
            //    .{frame_counter, bb.ul.x, bb.ul.y, bb.ur.x, bb.ur.y, bb.lr.x, bb.lr.y, bb.ll.x, bb.ll.y, bb.centroid.x, bb.centroid.y, bb.valid, bb.crit, bb.rejection_value}
            //);
            //_ = try writer.interface.write(line);
            //try writer.interface.flush();

            //frame_counter += 1;


//pub fn write_box_file(io: std.Io, f: *std.Io.File, bb: *const BoundingBox, frame_num: usize) !void{
//    var file_buffer: [512]u8 = .{0} ** 512;
//    var writer = f.writerStreaming(io, &file_buffer);
//
//    var string: [512]u8 = undefined;
//
//    const line = try std.fmt.bufPrint(
//        &string,
//        "Frame Number {d}: ul: ({d:.2}, {d:.2}), ur: ({d:.2}, {d:.2}), lr: ({d:.2}, {d:.2}), ll: ({d:.2}, {d:.2})\n\n",
//        .{frame_num, bb.ul.x, bb.ul.y, bb.ur.x, bb.ur.y, bb.lr.x, bb.lr.y, bb.ll.x, bb.ll.y}
//    );
//
//
//    _ = try writer.interface.write(line);
//    try writer.interface.flush();
//}

//var cam = Camshift.init(
    //    smp,
    //    buf,
    //    .{.window_radius = 150, .xc = 1735, .yc = 356}, //.{.window_radius = 5, .xc = 115, .yc = 87}
    //    @max(video_memory.metadata.width, 1),
    //    @max(video_memory.metadata.height, 1),
    //    256,
    //    256
    //) catch {
    //    return;
    //};
    //defer cam.deinit();

    //var w: Camshift.SearchWindow = .{.window_radius = 150, .xc = 1735, .yc = 356};
//const t = std.Io.Clock.now(.awake, io).toMilliseconds();
            //w = try cam.track(buf);
            //const t_calc = std.Io.Clock.now(.awake, io).toMilliseconds();
            //print("Calculation Time: {d} ms\n", .{t_calc - t});

            //print("Window Radius: {d}\nx-coordinate: {d}\ny-coordinate: {d}\n", .{w.window_radius, w.xc, w.yc});
