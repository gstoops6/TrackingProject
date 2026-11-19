const std = @import("std");
const builtin = @import("builtin");
const recording = @import("../Recording.zig");
pub const fmt = @cImport({
    @cInclude("stdint.h");
    @cInclude("inttypes.h");

    if (builtin.os.tag == .windows) @cUndef("__has_builtin");

    @cUndef("INT64_MAX");
    @cUndef("INT64_MIN");
    @cDefine("INT64_MAX", "9223372036854775807LL");
    @cDefine("INT64_MIN", "(-9223372036854775807LL - 1LL)");
    @cInclude("libavformat/avformat.h");
});

pub const codec = @cImport({
    @cInclude("stdint.h");
    @cInclude("inttypes.h");

    if (builtin.os.tag == .windows) @cUndef("__has_builtin");

    @cUndef("INT64_MAX");
    @cUndef("INT64_MIN");
    @cDefine("INT64_MAX", "9223372036854775807LL");
    @cDefine("INT64_MIN", "(-9223372036854775807LL - 1LL)");
    @cInclude("libavformat/avformat.h");
    @cInclude("libavcodec/avcodec.h");
    @cInclude("libavutil/frame.h");
    @cInclude("libavutil/imgutils.h");
    @cInclude("libswscale/swscale.h");
});

pub const VideoMemory = recording.VideoMemory;
pub const VideoMetadata = recording.VideoMetadata;

pub const GrayVideo = struct {
    allocator: std.mem.Allocator,
    width: usize = 0,
    height: usize = 0,
    frame_count: usize = 0,
    pixels: []u8 = &.{},

    pub fn deinit(self: *GrayVideo) void {
        self.allocator.free(self.pixels);
        self.pixels = &.{};
        self.frame_count = 0;
    }

    pub fn frame(self: *const GrayVideo, frame_index: usize) []const u8 {
        const frame_size = self.width * self.height;
        const start = frame_index * frame_size;

        return self.pixels[start .. start + frame_size];
    }
};

pub const FFmpegErrors = error{ UnableToOpenFile, InvalidStreamInfo, NoVideoStream, UnableToAllocatePacket, UnableToCopyCodecParameters, CodecFailedToOpen, NullVideoConfig, NullGPUConfig };

pub const DecodeErrors = error{
    UnableToFindDecoder,
    UnableToAllocateCodecContext,
    UnableToCopyCodecParameters,
    UnableToOpenCodec,
    UnableToAllocateFrame,
    UnableToAllocatePacket,
    UnableToCreateScaler,
    UnableToDecodeFrame,
    UnableToScaleFrame,
    IncompatibleOutputType
};

pub const CameraMemory = struct { image_rows: usize = 0, image_cols: usize = 0, image_stack_depth: usize = 0, camera_channels: usize = 0, camera_bit_depth: usize = 0, image_stack: []u16, stack_size: usize };

pub fn get_video_config(allocator: std.mem.Allocator, file_path: [:0]const u8, format: recording.VideoFormats) !VideoMemory {
    var video = VideoMemory{
        .allocator = allocator,
        .output_format = format
    };
    errdefer video.deinit();

    if (fmt.avformat_open_input(&video.format_ctx, file_path.ptr, null, null) < 0) {
        return FFmpegErrors.UnableToOpenFile;
    }


    if (fmt.avformat_find_stream_info(video.format_ctx.?, null) < 0) {
        return FFmpegErrors.InvalidStreamInfo;
    }

    //fmt.av_dump_format(video.format_ctx.?, 0, file_path.ptr, 0);

    const video_stream_index: c_int = fmt.av_find_best_stream(video.format_ctx.?, fmt.AVMEDIA_TYPE_VIDEO, -1, -1, null, 0);
    const stream_index: usize = @intCast(video_stream_index);

    for (0..video.format_ctx.?.nb_streams) |j| {
        if (j == stream_index) {
            video.format_ctx.?.streams[j].*.discard = fmt.AVDISCARD_DEFAULT;
        } else {
            video.format_ctx.?.streams[j].*.discard = fmt.AVDISCARD_ALL;
        }
    }

    if (video_stream_index < 0) {
        return FFmpegErrors.NoVideoStream;
    }

    video.video_stream = video.format_ctx.?.streams[stream_index];

    fill_video_metadata(&video);
    print_video_metadata(&video);

    video.decoder = codec.avcodec_find_decoder(video.metadata.codec_id);
    video.decoder_ctx = codec.avcodec_alloc_context3(video.decoder);

    if (video.video_stream) |stream| {
        if (codec.avcodec_parameters_to_context(video.decoder_ctx, @ptrCast(stream.codecpar)) < 0) {
            return FFmpegErrors.UnableToCopyCodecParameters;
        }
    } else {
        return FFmpegErrors.NoVideoStream;
    }

    if (codec.avcodec_open2(video.decoder_ctx, video.decoder, null) < 0) {
        return FFmpegErrors.CodecFailedToOpen;
    }

    const params = video.video_stream.?.codecpar;
    const video_fmt = params.*.format;

    const desc = codec.av_pix_fmt_desc_get(video_fmt);

    // Ensure that we are not trying to stream color video from grayscale.
    if(desc.*.nb_components == 1 and format == .rgb) return DecodeErrors.IncompatibleOutputType;

    video.scaler = codec.sws_getContext(
        video.decoder_ctx.*.width,
        video.decoder_ctx.*.height,
        video.decoder_ctx.*.pix_fmt,
        video.decoder_ctx.*.width,
        video.decoder_ctx.*.height,
        @intFromEnum(format),
        codec.SWS_BILINEAR,
        null,
        null,
        null,
    ) orelse return DecodeErrors.UnableToCreateScaler;

    return video;
}

pub fn fill_video_metadata(video: *VideoMemory) void {
    const format_ctx = video.format_ctx.?;
    const stream = video.video_stream.?;
    const codecpar = stream.codecpar;

    video.metadata = .{
        .nb_streams = format_ctx.nb_streams,
        .duration = format_ctx.duration,
        .bit_rate = format_ctx.bit_rate,
        .video_stream_index = stream.index,
        .width = codecpar.*.width,
        .height = codecpar.*.height,
        .codec_id = codecpar.*.codec_id,
        .time_base_num = stream.time_base.num,
        .time_base_den = stream.time_base.den,
        .metadata_count = fmt.av_dict_count(format_ctx.metadata),
    };
}

pub fn print_video_metadata(video: *const VideoMemory) void {
    const format_ctx = video.format_ctx.?;
    const metadata = video.metadata;
    const format_name = std.mem.span(format_ctx.iformat.*.name);

    std.debug.print("Video metadata\n", .{});
    std.debug.print("  format: {s}\n", .{format_name});
    std.debug.print("  streams: {d}\n", .{metadata.nb_streams});
    std.debug.print("  duration: {d}\n", .{metadata.duration});
    std.debug.print("  bit_rate: {d}\n", .{metadata.bit_rate});
    std.debug.print("  video_stream_index: {d}\n", .{metadata.video_stream_index});
    std.debug.print("  width: {d}\n", .{metadata.width});
    std.debug.print("  height: {d}\n", .{metadata.height});
    std.debug.print("  codec_id: {d}\n", .{metadata.codec_id});
    std.debug.print("  time_base: {d}/{d}\n", .{ metadata.time_base_num, metadata.time_base_den });
    std.debug.print("  metadata_entries: {d}\n", .{metadata.metadata_count});

    var tag = fmt.av_dict_iterate(format_ctx.metadata, null);
    while (tag != null) : (tag = fmt.av_dict_iterate(format_ctx.metadata, tag)) {
        std.debug.print("    {s}: {s}\n", .{ std.mem.span(tag.*.key), std.mem.span(tag.*.value) });
    }
}
