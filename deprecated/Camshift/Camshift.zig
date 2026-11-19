const Self = @This();
const std = @import("std");
const assert = std.debug.assert;
const Image = @import("Image.zig");
const local_binary_pattern = @import("LocalBinaryPattern.zig").local_binary_pattern;
const histograms = @import("Histograms.zig");
const FeatureHistogram = histograms.FeatureHistogram;


const rgb_pixel_len = Image.rgb_pixel_len;
const convergence_threshold_size = 0.01;
const convergence_threshold_position = 0.01;
const max_iters = 15;
const central_moment_lower_bound = 1e-6;



pub const SearchWindowFloat = struct{
    xc: f64,
    yc: f64,
    window_radius: f64,
};

pub const SearchWindow = struct{
    xc: usize,
    yc: usize,
    window_radius: usize,
};


pub const ImageROI = struct{
    x_start: usize,
    y_start: usize,
    x_len: usize,
    y_len: usize,
    total_pixels: usize
};


frame_width: usize,
frame_height: usize,
smp: std.mem.Allocator,
lbp: []u8,
hue_projection: []f32,
texture_projection: []f32,
window: SearchWindowFloat,
current_image: Image,
hue_reference: FeatureHistogram,
texture_reference: FeatureHistogram,
fused_projection: []f32,

pub fn init(
    smp: std.mem.Allocator,
    initial_image: []u8,
    search_window: SearchWindow,
    frame_width: usize,
    frame_height: usize,
    texture_bins: usize,
    hue_bins: usize
) !Self{
    if(frame_height == 0 or frame_width == 0) return error.InvalidFrameDimensions;

    var texture = try FeatureHistogram.init(smp, texture_bins);
    errdefer texture.deinit(smp);

    var hue = try FeatureHistogram.init(smp, hue_bins);
    errdefer hue.deinit(smp);


    const roi = calculate_frame_roi(
        .{
            .window_radius = @floatFromInt(search_window.window_radius),
            .xc = @floatFromInt(search_window.xc),
            .yc = @floatFromInt(search_window.yc),
        },
        frame_height,
        frame_width
    );
    const buffer_window_pixels = try copy_roi(
        smp,
        initial_image,
        roi,
        frame_width
    );
    defer smp.free(buffer_window_pixels);


    var im = try Image.init(
        smp,
        roi.x_len,
        roi.y_len
    );
    errdefer im.deinit();

    try im.update(
        buffer_window_pixels,
        roi.x_len,
        roi.y_len
    );

    const lbp = try smp.alloc(u8, im.data_gray.len);
    errdefer smp.free(lbp);

    try local_binary_pattern(&im, lbp);

    // Histograms should be stored for the original image.
    try histograms.hue_histogram(smp, im.data_hue, &hue);
    try histograms.texture_histogram(smp, lbp, &texture);

    const hue_proj = try smp.alloc(f32, im.data_hue.len);
    errdefer smp.free(hue_proj);

    const text_proj = try smp.alloc(f32, im.data_gray.len);
    errdefer smp.free(text_proj);

    const fused_proj = try smp.alloc(f32, im.data_gray.len);
    errdefer smp.free(fused_proj);



    return .{
        .smp = smp,
        .lbp = lbp,
        .hue_projection = hue_proj,
        .texture_projection = text_proj,
        .current_image = im,
        .hue_reference = hue,
        .texture_reference = texture,
        .window = .{
            .window_radius = @floatFromInt(search_window.window_radius),
            .xc = @floatFromInt(search_window.xc),
            .yc = @floatFromInt(search_window.yc),
        },
        .fused_projection = fused_proj,
        .frame_width = frame_width,
        .frame_height = frame_height
    };
}

pub fn deinit(self: *Self) void{
    self.hue_reference.deinit(self.smp);
    self.texture_reference.deinit(self.smp);
    self.current_image.deinit();
    self.smp.free(self.lbp);
    self.smp.free(self.hue_projection);
    self.smp.free(self.texture_projection);
    self.smp.free(self.fused_projection);
}


pub fn track(self: *Self, frame: []u8) !SearchWindow{
    if(frame.len != (self.frame_height * self.frame_width * rgb_pixel_len)) return error.InvalidFrameSize;

    const roi = calculate_frame_roi(
        self.window,
        self.frame_height,
        self.frame_width
    );

    const search_region_pixels = try copy_roi(
        self.smp,
        frame,
        roi,
        self.frame_width
    );
    defer self.smp.free(search_region_pixels);

    // Compute the frame's features and fused probability map once.
    try self.current_image.update(search_region_pixels, roi.x_len, roi.y_len);


    const pixel_count = self.current_image.data_gray.len;
    if(pixel_count != self.lbp.len){
        self.lbp = try self.smp.realloc(self.lbp, pixel_count);
    }
    try local_binary_pattern(&self.current_image, self.lbp);

    if(pixel_count != self.texture_projection.len){
        self.texture_projection = try self.smp.realloc(self.texture_projection, pixel_count);
    }
    try back_project(&self.texture_reference, self.lbp, self.texture_projection);

    if(pixel_count != self.hue_projection.len){
        self.hue_projection = try self.smp.realloc(self.hue_projection, pixel_count);
    }
    try back_project(&self.hue_reference, self.current_image.data_hue, self.hue_projection);

    if(pixel_count != self.fused_projection.len){
            self.fused_projection = try self.smp.realloc(self.fused_projection, pixel_count);
    }
    try self.fused_distribution_map();

    // Converge the window over the fixed frame map.
    for(0..max_iters)|_|{
        const previous_window = self.window;
        self.window = self.calculate_frame_moment(roi);

        if(window_threshold_converged(&self.window, &previous_window)){
            //std.debug.print("Convergence achieved in {d} iterations.\n", .{i});
            break;
        }
    }
    return .{
        .window_radius = @intFromFloat(@round(self.window.window_radius)),
        .xc = @intFromFloat(self.window.xc),
        .yc = @intFromFloat(self.window.yc)
    };
}


inline fn window_threshold_converged(new: *const SearchWindowFloat, old: *const SearchWindowFloat) bool{
    const x_diff_pow2: f64 = std.math.pow(f64, new.xc - old.xc, 2);
    const y_diff_pow2: f64 = std.math.pow(f64, new.yc - old.yc, 2);
    const rad_abs_diff: f64 = @abs(new.window_radius - old.window_radius);

    return (x_diff_pow2 + y_diff_pow2) <= convergence_threshold_position and rad_abs_diff <= convergence_threshold_size;
}


pub fn calculate_frame_moment(self: *Self, roi: ImageROI) SearchWindowFloat{
    var m00: f64 = 0;
    var m01: f64 = 0;
    var m10: f64 = 0;

    for(0..roi.y_len)|i|{
        for(0..roi.x_len)|j|{
            //const idx: usize = (roi.y_start + i) * self.frame_width + roi.x_start + j;
            const idx = i * roi.x_len + j;
            const ref: f64 = @floatCast(self.fused_projection[idx]);

            m00 += ref;
            m01 += @as(f64, @floatFromInt(i)) * ref;
            m10 += @as(f64, @floatFromInt(j)) * ref;
        }
    }
    //if(m00 == 0) return error.UndefinedZerothMoment;
    m00 = @max(central_moment_lower_bound, m00);

    return .{
        .xc = @min(@as(f64, @floatFromInt(roi.x_start)) + m10/m00, @as(f64, @floatFromInt(self.frame_width - 1))),
        .yc = @min(@as(f64, @floatFromInt(roi.y_start)) + m01 / m00, @as(f64, @floatFromInt(self.frame_height - 1))),
        .window_radius = @max(@sqrt(m00 / 256), 1.0)
    };
}


/// Fuse the hue and texture back-projection maps populated by track().
pub fn fused_distribution_map(self: *Self) !void{
    const hue = self.hue_projection;
    const texture = self.texture_projection;
    const fused = self.fused_projection;

    if(hue.len != texture.len or fused.len != hue.len)
        return error.InvalidBackProjectionLength;

    for(0..fused.len)|i|{
        const total = hue[i] + texture[i];
        if(total <= 0){
            fused[i] = 0;
            continue;
        }

        const k = texture[i] / total;
        fused[i] = (1.0 - k) * hue[i] + k * texture[i];
    }
}




/// Calculates the ROI of the image, clipping its extent to the frame bounds.
/// Width and height should be total frame dimensions.
fn calculate_frame_roi(window: SearchWindowFloat, frame_height: usize, frame_width: usize) ImageROI{
    const xc: isize = @intFromFloat(window.xc);
    const yc: isize = @intFromFloat(window.yc);
    const window_rad: isize = @intFromFloat(window.window_radius);
    const window_diam = 2 * window_rad;

    const x_start: isize = @max(0, xc - window_rad);
    const y_start: isize = @max(0, yc - window_rad);

    const x_end: isize = @min(x_start + window_diam, frame_width);
    const y_end: isize = @min(y_start + window_diam, frame_height);

    const x_start_pixel: usize = @intCast(x_start);
    const y_start_pixel: usize = @intCast(y_start);
    const x_end_pixel: usize = @intCast(x_end);
    const y_end_pixel: usize = @intCast(y_end);
    const x_len = x_end_pixel - x_start_pixel;
    const y_len = y_end_pixel - y_start_pixel;

    return .{
        .x_start = x_start_pixel,
        .y_start = y_start_pixel,
        .x_len = x_len,
        .y_len = y_len,
        .total_pixels = x_len * y_len
    };
}

// src contains f32 hues (scaler = 360) or u8 texture codes (scaler = 256).
fn back_project(hist: *const FeatureHistogram, src: anytype, back_projection: []f32) !void{
    if(hist.histogram.len == 0) return error.InvalidHistogramLength;
    if(src.len == 0) return error.InvalidSourceImageLength;
    if(src.len != back_projection.len) return error.InvalidBackProjectionLength;
    if(!std.math.isFinite(hist.max) or hist.max <= 0) return error.InvalidHistogramMaximum;

    const scaler: f32 = if(@TypeOf(src[0]) == u8) 256.0 else 360.0;

    for(0..src.len)|i|{
        const pixel: f32 = if(@TypeOf(src[i]) == u8) @floatFromInt(src[i]) else src[i];
        const pixel_bin = try histograms.histogram_bin(pixel, hist.histogram.len, scaler);
        const hist_bin_value = hist.histogram[pixel_bin];

        back_projection[i] = @floor(255.0 * (hist_bin_value / hist.max));
    }
}

pub fn copy_roi(smp: std.mem.Allocator, buffer: []u8, roi: ImageROI, image_buffer_width: usize) ![]u8{
    const roi_image = try smp.alloc(u8, roi.total_pixels * rgb_pixel_len);

    var roi_idx: usize = 0;
    const row_bytes = roi.x_len * rgb_pixel_len;
    for(roi.y_start.. roi.y_start + roi.y_len) |y|{
        const idx = (y * image_buffer_width + roi.x_start) * rgb_pixel_len;
        @memcpy(roi_image[roi_idx..][0..row_bytes], buffer[idx..][0..row_bytes]);
        roi_idx += row_bytes;
    }
    return roi_image;
}

//test
fn test_print_lbp(lbp: []u8, width: usize, height: usize) !void{
    std.debug.print("\n", .{});
    std.debug.print("\n", .{});
    std.debug.print("\n", .{});
    std.debug.print("Width: {d}. Height: {d}\n\n", .{width, height});


    var r: usize = 0;
    for(0..lbp.len)|i|{
        if(i != 0 and i % (width) == 0) std.debug.print("\n", .{});
        const row = @divFloor(i, height);
        if(row == r){
            r += 1;
            std.debug.print("Row {d}: ", .{row});
        }

        std.debug.print("{d} ", .{lbp[i]});



    }
    std.debug.print("\n", .{});
    std.debug.print("\n", .{});
    std.debug.print("\n", .{});
    return error.NoError;
}


