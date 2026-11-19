const std = @import("std");
const assert = std.debug.assert;
const print = std.debug.print;


pub const FeatureHistogram = struct {
    histogram: []f32,
    max: f32,

    pub fn init(smp: std.mem.Allocator, n_bin: usize) !FeatureHistogram{
        if(n_bin == 0) return error.InvalidHistogramLength;
        const histogram = try smp.alloc(f32, n_bin);
        @memset(histogram, 0);
        return .{
            .histogram = histogram,
            .max = 0.0
        };
    }
    pub fn deinit(self: *FeatureHistogram, smp: std.mem.Allocator) void {
        smp.free(self.histogram);
    }
};


pub inline fn histogram_bin(value: f32, n_bins: usize, scaler: f32) !usize{
    if(value < 0 or value >= scaler) {
        std.debug.print("Value: {d}\n", .{value});
        return error.InvalidFeatureValue;
    }

    const bin: usize = @intFromFloat(@floor(
        (@as(f64, value) / @as(f64, scaler)) * @as(f64, @floatFromInt(n_bins))
    ));
    return @min(bin, n_bins - 1);
}

pub fn texture_histogram(smp: std.mem.Allocator, src: []const u8, hist: *FeatureHistogram) !void{
    if(hist.histogram.len == 0) return error.InvalidHistogramLength;
    if(src.len == 0) return error.InvalidSourceImageLength;

    const n_bins = hist.histogram.len;

    const counts = try smp.alloc(usize, n_bins);
    defer smp.free(counts);
    @memset(counts, 0);

    for(0..src.len)|i|{
        const texture_bin = try histogram_bin(@floatFromInt(src[i]), n_bins, 256.0);
        counts[texture_bin] += 1;
    }

    hist.max = 0.0;
    for(0..n_bins)|i|{
        hist.histogram[i] = @as(f32, @floatFromInt(counts[i])) / @as(f32, @floatFromInt(src.len));
        if(hist.histogram[i] > hist.max) hist.max = hist.histogram[i];
    }
}

pub fn hue_histogram(smp: std.mem.Allocator, src: []const f32, hist: *FeatureHistogram) !void{
    if(hist.histogram.len == 0) return error.InvalidHistogramLength;
    if(src.len == 0) return error.InvalidSourceImageLength;

    const n_bins = hist.histogram.len;

    const counts = try smp.alloc(usize, n_bins);
    defer smp.free(counts);
    @memset(counts, 0);

    for(0..src.len)|i|{
        const hue_bin = try histogram_bin(src[i], n_bins, 360.0);
        counts[hue_bin] += 1;
    }

    hist.max = 0.0;
    for(0..n_bins)|i|{
        hist.histogram[i] = @as(f32, @floatFromInt(counts[i])) / @as(f32, @floatFromInt(src.len));
        if(hist.histogram[i] > hist.max) hist.max = hist.histogram[i];
    }
}
