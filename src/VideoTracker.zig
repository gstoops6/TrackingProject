const std = @import("std");
const root = @import("root");
const config = @import("Config.zig");
const IMM = @import("KalmanFilter/IMM.zig");
const AKAZE = @import("Tracking/AKAZE.zig");
const KalmanFilter = @import("KalmanFilter/KalmanFilter.zig");
const mat = @import("KalmanFilter/Matrix.zig");
const Camshift = @import("Tracking/Camshift.zig");
const frame_op = @import("Tracking/FrameOperations.zig");
const Frame = frame_op.Frame;
pub const opencv = @cImport({
    @cInclude("OpenCVTracking.h");
});

// Constants
const rgb_pixel_len = frame_op.rgb_pixel_len;
const expansion_step: f32 = 0.1;
const expansion_default: f32 = 1.5;
const expansion_min: f32 = 1.2;
const expansion_max: f32 = 2.5;
const kcf_confidence_min: f32 = 0.7;
const akaze_recompute_interval: usize = 10;
const kcf_confidence_threshold: f32 = 0.97;
//const akaze_update_interval: usize = 10;

// Types
pub const BoundingBox = opencv.BoundingBox;
pub const FrameDescriptor = opencv.FrameDescriptor;
pub const Matrix = mat.Matrix(config.Float);
pub const ECT = opencv.ECT;
pub const bbPoint = opencv.bbPoint;

//Akaze
pub const akaze_destroy = opencv.akaze_destroy;
pub const akaze_compute = opencv.akaze_compute;
pub const akaze_init = opencv.akaze_init;
pub const AkazeTracker = opencv.AkazeTracker;

//Camshift
pub const camshift_init = opencv.camshift_init;
pub const camshift_destroy = opencv.camshift_destroy;
pub const camshift_compute = opencv.camshift_compute;
pub const CamshiftTracker = opencv.CamshiftTracker;


//KCF
pub const kcf_init = opencv.kcf_init;
pub const kcf_destroy = opencv.kcf_destroy;
pub const kcf_compute = opencv.kcf_compute;
pub const KCFTracker = opencv.KCFTracker;


//Helpers
const copy_to_frame = frame_op.copy_to_frame;
const copy_region = frame_op.copy_region;
const translate = frame_op.translate_box_to_frame;


pub const VideoTracker = struct{
    color: Frame(u8),
    region: Frame(u8),
    smp: std.mem.Allocator,
    arena: std.heap.ArenaAllocator,
    bounding_box: BoundingBox,
    bounding_box_matrix: Matrix,
    ect: ECT = .{.ak = null, .cs = null, .kcf = null},
    imm: IMM,
    kf_buffer: []KalmanFilter,
    expansion_region: f32 = expansion_default,
    consecutive_primary_failures: usize = 0,
    future: ?std.Io.Future(anyerror!void) = null,
    kcf_score: f32 = 0,
    frames_since_last_update: usize = 0,
    kcf_lost: bool = false,
    akaze_update: AkazeAsyncUpdate,
    initial_akaze_matches: f32 = 0,
    initial_estimate: Matrix,
    initial_covariance: Matrix,

    pub fn init(smp: std.mem.Allocator, io: std.Io, first_frame: []u8, bb: BoundingBox, frame_rate: f64, width: usize, height: usize) !VideoTracker{

        const color_stride = width * rgb_pixel_len;
        const color = try smp.alloc(u8, height * color_stride);
        errdefer smp.free(color);
        const region = try smp.alloc(u8, height * color_stride);
        errdefer smp.free(region);


        //akaze_recompute_interval = @as(usize, @intFromFloat(frame_rate / 3));

        const kf_buffer = try smp.alloc(KalmanFilter, config.num_filters);
        errdefer smp.free(kf_buffer);
        var initial_covariance: Matrix = try .empty(smp, config.state_transition_rows, config.state_transition_cols, .row_major);
        errdefer initial_covariance.destroy();
        @memcpy(initial_covariance.data, &config.init_cov);
        var initial_estimate: Matrix = try .empty(smp, config.state_transition_rows, 1, .row_major);
        errdefer initial_estimate.destroy();

        initial_estimate.data[0] = @floatCast(bb.centroid.x);
        initial_estimate.data[3] = @floatCast(bb.centroid.y);

        var arena: std.heap.ArenaAllocator = .{.child_allocator = smp, .state = .init};

        try create_kalman_filters(
            arena.allocator(),
            kf_buffer,
            @floatCast(1 / frame_rate),
            &initial_estimate,
            &initial_covariance
        );

        var imm = build_imm(arena.allocator(), kf_buffer) catch |err| {
            for (kf_buffer) |*filter| filter.deinit();
            return err;
        };
        errdefer imm.deinit();

        var bounding_box_matrix: Matrix = try .empty(smp, 2, 1, .row_major);
        errdefer bounding_box_matrix.destroy();


        var t: VideoTracker = .{
            .color = .init(height, width, color_stride, color),
            .region = .init(height, width, color_stride, region),
            .smp = smp,
            .arena = arena,
            .bounding_box = bb,
            .bounding_box_matrix = bounding_box_matrix,
            .imm = imm,
            .kf_buffer = kf_buffer,
            .initial_estimate = initial_estimate,
            .initial_covariance = initial_covariance,
            .akaze_update = .{.io = io, .bounding_box = bb}
        };
        errdefer {
            if(t.ect.cs)|cs| camshift_destroy(cs);
            if(t.ect.ak)|ak| akaze_destroy(ak);
            if(t.ect.kcf)|kcf| kcf_destroy(kcf);
        }

        t.update(first_frame);

        var desc = to_descriptor(&t.color);

        if(root.tracking_type == .KCF){
            try t.create_kcf_tracker(&desc, &t.bounding_box);
        }else if(root.tracking_type == .Camshift){
            try t.create_camshift_tracker(&desc);
        } else {
            return error.TrackingTypeNotImplemented;
        }

        try t.create_akaze_tracker(&desc);
        return t;
    }

    pub fn deinit(self: *VideoTracker) void{
        self.cancel_async() catch {};
        if(self.ect.cs)|cs| camshift_destroy(cs);
        if(self.ect.ak)|ak| akaze_destroy(ak);
        if(self.ect.kcf) |kcf| kcf_destroy(kcf);
        self.imm.deinit();
        self.initial_covariance.destroy();
        self.initial_estimate.destroy();
        self.smp.free(self.kf_buffer);
        self.bounding_box_matrix.destroy();
        self.color.deinit(self.smp);
        self.region.deinit(self.smp);
        self.arena.deinit();
        self.* = undefined;
    }

    pub fn update(self: *VideoTracker, data: []u8) void{
        copy_to_frame(u8, &self.color, data);
    }


    pub fn create_akaze_tracker(self: *VideoTracker, desc: *FrameDescriptor) !void{
        self.ect.ak = akaze_init(
            desc,
            &self.bounding_box,
            &self.initial_akaze_matches
        ) orelse return error.AkazeTrackerInitializationFailed;
    }

    pub fn create_camshift_tracker(self: *VideoTracker, desc: *FrameDescriptor) !void{
        self.ect.cs = camshift_init(
            desc,
            &self.bounding_box,
            self.initial_akaze_matches
        ) orelse return error.CamshiftTrackerInitializationFailed;
    }

    pub fn create_kcf_tracker(self: *VideoTracker, desc: *FrameDescriptor, bb: *BoundingBox) !void{
        self.ect.kcf = kcf_init(
            desc,
            bb
        ) orelse return error.KCFTrackerInitializationFailure;
    }

    pub fn reinit_kcf_tracker(self: *VideoTracker, bb: *BoundingBox) !void{
        if(self.ect.kcf)|kcf| kcf_destroy(kcf);

        var color = to_descriptor(&self.color);
        try self.create_kcf_tracker(&color, bb);
    }

    pub fn compute(self: *VideoTracker) !void{
        if(root.tracking_type == .Camshift){
            try self.compute_camshift();
        } else if(root.tracking_type == .KCF){
            try self.compute_kcf();
        } else {
            return error.TrackingTypeNotImplemented;
        }
    }

    inline fn compute_camshift(self: *VideoTracker) !void{
        try self.imm.predict();

        var search_box = self.bounding_box;
        search_box.valid = true;
        const crop_origin: ?frame_op.CropOrigin = copy_region(
            &self.color,
            &self.region,
            &search_box,
            self.expansion_region
        ) catch null;


        var desc = to_descriptor(if(crop_origin != null) &self.region else &self.color);
        if(crop_origin) |origin| {
            const offset_x: f32 = @floatFromInt(origin.x);
            const offset_y: f32 = @floatFromInt(origin.y);
            for([_]*bbPoint{&search_box.ul, &search_box.ur, &search_box.lr, &search_box.ll, &search_box.centroid}) |point| {
                point.x -= offset_x;
                point.y -= offset_y;
            }
        }

        var tmp_bb: BoundingBox = camshift_compute(
            self.ect.cs orelse return error.NullTracker,
            &desc,
            &search_box
        );


        if(!tmp_bb.valid){
            self.expansion_region = @min(expansion_max, self.expansion_region + expansion_step);

            const akaze_bb = akaze_compute(
                self.ect.ak orelse return error.NullTracker,
                &desc
            );

            const primary_crit = tmp_bb.crit;
            tmp_bb = akaze_bb;

            if(akaze_bb.valid){
                tmp_bb.crit = primary_crit;
            }
        } else {
            self.expansion_region = expansion_default;
        }


        if(crop_origin) |origin| translate(&tmp_bb, origin);
        if(!tmp_bb.valid){
            var imm_update = try self.imm.get_prior_estimate();
            defer imm_update.covariance.destroy();
            defer imm_update.state.destroy();
            imm_update_bounding_box(&tmp_bb, &self.bounding_box,&imm_update);
            tmp_bb.matcher = opencv.IMM;
        } else {
            matrix_from_bounding_box(&tmp_bb, &self.bounding_box_matrix);
            var imm_update = try self.imm.update(&self.bounding_box_matrix);
            defer imm_update.covariance.destroy();
            defer imm_update.state.destroy();
        }

        self.bounding_box = tmp_bb;
    }


    inline fn compute_kcf(self: *VideoTracker) !void{
        try self.imm.predict();

        var color_desc = to_descriptor(&self.color);
        var tmp_bb =  if(!self.kcf_lost) kcf_compute(
            self.ect.kcf orelse return error.NullTracker,
            &color_desc,
            &self.kcf_score
        ) else self.bounding_box;


        if(self.future != null){
            self.akaze_update.join_after_complete(self.smp, &self.future.?);
        }else{
            self.akaze_update.join_after_complete(self.smp, null);
        }

        tmp_bb.valid = if(self.kcf_score < kcf_confidence_min) false else tmp_bb.valid;
        self.frames_since_last_update += 1;

        if(!tmp_bb.valid and self.consecutive_primary_failures >= 10 and @mod(self.consecutive_primary_failures, akaze_recompute_interval) == 0){
            self.consecutive_primary_failures += 1;
            //self.expansion_region += expansion_max;
            if(!self.kcf_lost) std.debug.print("KCF Tracker Lost\n", .{});
            self.kcf_lost = true;
            tmp_bb = self.bounding_box;
            try self.cancel_async();
            try self.compute_akaze(&tmp_bb, true);

            if(tmp_bb.valid){
                self.frames_since_last_update = 0;
                self.consecutive_primary_failures = 0;
                self.kcf_lost = false;
                std.debug.print("Akaze Recovered\n", .{});
                try self.reinit_kcf_tracker(&tmp_bb);
            }
        } else if(
                tmp_bb.valid and
                self.kcf_score > kcf_confidence_threshold and
                self.kcf_score > kcf_confidence_min and
                self.frames_since_last_update > akaze_recompute_interval and
                self.akaze_update.check_ready()
        ){
            if(self.ect.ak)|ak|{
                self.akaze_update.bounding_box = tmp_bb;
                try self.akaze_update.absorb_clone(self.smp, &self.color);
                self.akaze_update.ready_flag.store(false, .release);
                self.future = try self.akaze_update.io.concurrent(run_update, .{&self.akaze_update, ak, &self.region});
            } else return error.NullTracker;
            self.frames_since_last_update = 0;
            self.consecutive_primary_failures = 0;
        }

        if(!tmp_bb.valid) {
            self.consecutive_primary_failures += 1;
            var imm_update = try self.imm.get_prior_estimate();
            defer imm_update.covariance.destroy();
            defer imm_update.state.destroy();
            imm_update_bounding_box(&tmp_bb, &self.bounding_box,&imm_update);
            tmp_bb.matcher = opencv.IMM;
        }else {
            try self.valid_imm_update(&tmp_bb);
        }
        self.bounding_box = tmp_bb;
    }

    fn compute_akaze(self: *VideoTracker, bb: *BoundingBox, use_crop: bool) !void{
        const crop_origin: ?frame_op.CropOrigin = if(use_crop) copy_region(
            &self.color,
            &self.region,
            bb,
            self.expansion_region
        ) catch null
        else null;


        var desc = VideoTracker.to_descriptor(if(crop_origin != null) &self.region else &self.color);
        if(crop_origin) |origin| {
            const offset_x: f32 = @floatFromInt(origin.x);
            const offset_y: f32 = @floatFromInt(origin.y);
            for([_]*bbPoint{
                &bb.ul,
                &bb.ur,
                &bb.lr,
                &bb.ll,
                &bb.centroid
            }) |point| {
                point.x -= offset_x;
                point.y -= offset_y;
            }
        }

        var tmp_bb = akaze_compute(
            self.ect.ak orelse return error.NullTracker,
            &desc
        );

        if(!tmp_bb.valid){
            bb.crit = tmp_bb.crit;
            bb.rejection_value = tmp_bb.rejection_value;
            bb.valid = false;
        }else{
            if(crop_origin) |origin| translate(&tmp_bb, origin);
            bb.* = tmp_bb;
        }
    }

    fn valid_imm_update(self: *VideoTracker, bb: *BoundingBox) !void{
        matrix_from_bounding_box(bb, &self.bounding_box_matrix);
        var imm_update = try self.imm.update(&self.bounding_box_matrix);
        defer imm_update.covariance.destroy();
        defer imm_update.state.destroy();
    }

    fn cancel_async(self: *VideoTracker) !void{
        if(!self.akaze_update.check_completion()){
            if(self.future != null){
                self.future.?.cancel(self.akaze_update.io) catch |err| {
                    if(err != std.Io.Cancelable.Canceled){
                        return err;
                    }
                };
                //std.debug.print("Akaze Update Cancelled\n", .{});
                self.future = null;
            }
            self.akaze_update.join_after_complete(self.smp, null);
        }
    }


    pub fn get_bounding_box(self: *VideoTracker) BoundingBox{
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
    
    fn imm_update_bounding_box(next_bb: *BoundingBox, prior_bb: *const BoundingBox, imm_update: *IMM.Estimate) void{
        const new_x = imm_update.state.data[0];
        const new_y = imm_update.state.data[3];
        const offset_x = new_x - prior_bb.centroid.x;
        const offset_y = new_y - prior_bb.centroid.y;

        for (
            [_]*bbPoint{&next_bb.ul, &next_bb.ur, &next_bb.lr, &next_bb.ll},
            [_]bbPoint{prior_bb.ul, prior_bb.ur, prior_bb.lr, prior_bb.ll}
        ) |next_point, prior_point| {
            next_point.* = .{.x = prior_point.x + offset_x, .y = prior_point.y + offset_y};
        }
        next_bb.centroid = .{.x = new_x, .y = new_y};
    }

    inline fn matrix_from_bounding_box(bb: *const BoundingBox, m: *Matrix) void{
        m.data[0] = @floatCast(bb.centroid.x);
        m.data[1] = @floatCast(bb.centroid.y);
    }

};


pub const AkazeAsyncUpdate = struct{
    completion_flag: std.atomic.Value(bool) = .init(false),
    ready_flag: std.atomic.Value(bool) = .init(true),
    cancelled: bool = false,
    frame_copy: ?Frame(u8) = null,

    io: std.Io,
    bounding_box: BoundingBox,

    pub inline fn check_completion(self: *AkazeAsyncUpdate) bool{
        return self.completion_flag.load(.acquire);
    }

    pub inline fn check_ready(self: *AkazeAsyncUpdate) bool{
        return self.ready_flag.load(.acquire);
    }

    pub fn join_after_complete(self: *AkazeAsyncUpdate, smp: std.mem.Allocator, future: ?*std.Io.Future(anyerror!void)) void{
        if(self.check_completion() and !self.check_ready()){
            if(self.frame_copy != null){
                self.frame_copy.?.deinit(smp);
                self.frame_copy = null;
            }

            if(future)|f|{
                f.await(self.io) catch {};
            }

            self.completion_flag.store(false, .release);
            self.ready_flag.store(true, .release);
            //std.debug.print("Update Completed\n", .{});
        }
    }

    /// Pass frame to this sync primitive to manage the lifetime of the clone during the update.
    /// Simplifies lifetime as it will be destroyed after completing update.
    pub fn absorb_clone(self: *AkazeAsyncUpdate, smp: std.mem.Allocator, frame: *Frame(u8)) !void{
        self.frame_copy = try frame.clone(smp);
    }
};

pub fn run_update(update_primitive: *AkazeAsyncUpdate, akaze_tracker: *AkazeTracker, region: *Frame(u8)) anyerror!void{
    update_primitive.completion_flag.store(false, .release);
    defer update_primitive.completion_flag.store(true, .release);

    errdefer{
        update_primitive.cancelled = true;
    }

    var frame = update_primitive.frame_copy orelse {
        return;
    };

    const crop_origin: ?frame_op.CropOrigin = copy_region(
        &frame,
        region,
        &update_primitive.bounding_box,
        expansion_default
    ) catch null;


    var desc = VideoTracker.to_descriptor(if(crop_origin != null) region else &frame);
    if(crop_origin) |origin| {
        const offset_x: f32 = @floatFromInt(origin.x);
        const offset_y: f32 = @floatFromInt(origin.y);
        for([_]*bbPoint{
            &update_primitive.bounding_box.ul,
            &update_primitive.bounding_box.ur,
            &update_primitive.bounding_box.lr,
            &update_primitive.bounding_box.ll,
            &update_primitive.bounding_box.centroid
        }) |point| {
            point.x -= offset_x;
            point.y -= offset_y;
        }
    }

    // Check for cancellation before starting Akaze update. If cancellation is not found then the update must
    // run through in its entirety.
    try update_primitive.io.checkCancel();
    update_primitive.bounding_box = akaze_compute(
        akaze_tracker,
        &desc
    );

    if(crop_origin) |origin| translate(&update_primitive.bounding_box, origin);
}


/// Initializes CA/CV filters in caller-provided storage for transfer to IMM.init.
/// The initial estimate and covariance are borrowed; model matrices belong to the filters.
fn create_kalman_filters(
    smp: std.mem.Allocator,
    filter_buffer: []KalmanFilter,
    dt: config.Float,
    initial_estimate: *Matrix,
    initial_covariance: *Matrix,
) !void{
    if (initial_estimate.rows != config.state_transition_rows or initial_estimate.cols != 1) {
        return error.InvalidInitialEstimateDimensions;
    }
    if (initial_covariance.rows != config.state_transition_rows or initial_covariance.cols != config.state_transition_cols) {
        return error.InvalidInitialCovarianceDimensions;
    }


    var initialized_count: usize = 0;
    errdefer {
        for (filter_buffer[0..initialized_count]) |*filter| filter.deinit();
    }

    const models = .{
        .{
            .state_transition = config.ca_state_transisition_matrix,
            .process_noise = config.ca_process_noise_matrix,
            .observation = config.ca_observation_matrix,
            .measurement_noise = config.ca_measurement_noise_matrix,
        },
        .{
            .state_transition = config.cv_state_transition_matrix,
            .process_noise = config.cv_process_noise_matrix,
            .observation = config.cv_observation_matrix,
            .measurement_noise = config.cv_measurement_noise_matrix,
        },
    };

    inline for (models) |model| {
        var state_transition = try Matrix.empty(smp, config.state_transition_rows, config.state_transition_cols, .row_major);
        errdefer state_transition.destroy();
        const state_transition_data = model.state_transition(dt);
        @memcpy(state_transition.data, &state_transition_data);

        var process_noise = try Matrix.empty(smp, config.process_noise_rows, config.process_noise_cols, .row_major);
        errdefer process_noise.destroy();
        const process_noise_data = model.process_noise(dt);
        @memcpy(process_noise.data, &process_noise_data);

        var observation = try Matrix.empty(smp, config.observation_matrix_rows, config.observation_matrix_cols, .row_major);
        errdefer observation.destroy();
        const observation_data = model.observation;
        @memcpy(observation.data, &observation_data);

        var measurement_noise = try Matrix.empty(smp, config.measurement_noise_rows, config.measurement_noise_cols, .row_major);
        errdefer measurement_noise.destroy();
        const measurement_noise_data = model.measurement_noise;
        @memcpy(measurement_noise.data, &measurement_noise_data);

        filter_buffer[initialized_count] = try KalmanFilter.init(
            smp, &state_transition, &process_noise, &measurement_noise,
            &observation, initial_estimate, initial_covariance,
        );
        initialized_count += 1;
    }
}


fn build_imm(smp: std.mem.Allocator, filter_buffer: []KalmanFilter) !IMM{
    var mu: Matrix = try .empty(smp, config.mu_rows, config.mu_cols, .row_major);
    errdefer mu.destroy();
    @memcpy(mu.data, &config.mu);

    var transition_matrix: Matrix = try .empty(smp, config.transition_matrix_rows, config.transition_matrix_cols, .row_major);
    errdefer transition_matrix.destroy();
    @memcpy(transition_matrix.data, &config.transition_matrix);


    return try .init(smp, filter_buffer, &mu, &transition_matrix, config.state_transition_cols);
}
