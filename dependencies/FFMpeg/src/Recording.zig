const std = @import("std");
pub const ffmpeg = @import("Recording/FFmpeg.zig");
pub const fmt = ffmpeg.fmt;
pub const codec = ffmpeg.codec;
const root = @import("root");

const debug_mode = false;

pub const VideoFormats = enum(c_int) {
    gray8 = codec.AV_PIX_FMT_GRAY8,
    gray16le = codec.AV_PIX_FMT_GRAY16LE,
    rgb = codec.AV_PIX_FMT_RGB24
};

pub const VideoMetadata = struct {
    nb_streams: u32 = 0,
    duration: i64 = 0,
    bit_rate: i64 = 0,
    video_stream_index: i32 = -1,
    width: i32 = 0,
    height: i32 = 0,
    output_stride: i32 = 0,
    codec_id: fmt.AVCodecID = fmt.AV_CODEC_ID_NONE,
    time_base_num: i32 = 0,
    time_base_den: i32 = 0,
    metadata_count: c_int = 0,
    packet_count: usize = 0,
    video_packet_count: usize = 0,
    video_bytes: usize = 0,
};

pub const VideoMemory = struct {
    allocator: std.mem.Allocator,
    format_ctx: ?*fmt.AVFormatContext = null,
    video_stream: ?*fmt.AVStream = null,
    metadata: VideoMetadata = .{},
    output_format: VideoFormats = .gray8,
    scaler: [*c]codec.struct_SwsContext = null,
    decoder: [*c]const codec.struct_AVCodec = null,
    decoder_ctx: [*c]codec.struct_AVCodecContext = null,
    source_frame_buffer: [*c]codec.struct_AVFrame = null,
    decoded_frame_buffer: [*c]codec.struct_AVFrame = null,
    packet: [*c]codec.struct_AVPacket = null,

    pub fn init(smp: std.mem.Allocator, video_slice: [:0]const u8, format: VideoFormats) !VideoMemory {
        var video = try ffmpeg.get_video_config(smp, video_slice, format);

        video.source_frame_buffer = codec.av_frame_alloc() orelse return error.UnableToAllocateFrame;
        video.decoded_frame_buffer = codec.av_frame_alloc() orelse return error.UnableToAllocateFrame;
        video.packet = codec.av_packet_alloc() orelse return error.UnableToAllocatePacket;

        video.decoded_frame_buffer.*.format = @intFromEnum(video.output_format);
        video.decoded_frame_buffer.*.width = video.decoder_ctx.*.width;
        video.decoded_frame_buffer.*.height = video.decoder_ctx.*.height;

        if (codec.av_frame_get_buffer(video.decoded_frame_buffer, 0) < 0) {
            return error.UnableToAllocateFrame;
        }

        const stride = video.decoded_frame_buffer.*.linesize;

        video.metadata.output_stride = @intCast(stride[0]); // Only concerned with RGB and grayscle. All information is on linesize[0]

        try video.restart();

        //std.debug.print("Stride: {d}. Width: {d}\n", .{video.metadata.output_stride, video.metadata.width});

        return video;
    }

    pub fn deinit(self: *VideoMemory) void {
        if (self.format_ctx != null) {
            fmt.avformat_close_input(&self.format_ctx);
        }

        if (self.decoder != null) {
            codec.avcodec_free_context(&self.decoder_ctx);
        }

        if (self.scaler != null) {
            codec.sws_free_context(&self.scaler);
        }

        if (self.source_frame_buffer != null) {
            codec.av_frame_free(&self.source_frame_buffer);
        }

        if (self.decoded_frame_buffer != null) {
            codec.av_frame_free(&self.decoded_frame_buffer);
        }

        if (self.packet != null) {
            codec.av_packet_free(&self.packet);
        }
    }

    pub fn switch_output_format(self: *VideoMemory, format: VideoFormats) void {
        if (self.scaler != null) {
            codec.sws_free_context(&self.scaler);
        }

        self.output_format = format;

        self.scaler = codec.sws_getContext(
            self.decoder_ctx.width,
            self.decoder_ctx.height,
            self.decoder_ctx.pix_fmt,
            self.decoder_ctx.width,
            self.decoder_ctx.height,
            @intFromEnum(format),
            codec.SWS_BILINEAR,
            null,
            null,
            null,
        );
    }

    pub fn restart(self: *VideoMemory) !void {
        const format_ctx = self.format_ctx orelse return error.NullFormatContext;

        if (codec.av_seek_frame(@ptrCast(format_ctx), self.metadata.video_stream_index, 0, codec.AVSEEK_FLAG_BACKWARD) < 0) {
            return error.FrameUnavailable;
        }

        codec.avcodec_flush_buffers(self.decoder_ctx);
        codec.av_packet_unref(self.packet);
    }

    pub fn get_frame(self: *VideoMemory, buffer: []u8) !void {
        var replay_attempted = false;

        while (true) {
            const read_frame: c_int = codec.av_read_frame(@ptrCast(self.format_ctx.?), self.packet);

            if (read_frame < 0) {
                if (replay_attempted) return error.FrameUnavailable;

                try self.restart();
                replay_attempted = true;
                continue;
            }

            defer codec.av_packet_unref(self.packet);

            if (self.packet.?.*.stream_index != self.metadata.video_stream_index) {
                continue;
            }

            if (codec.avcodec_send_packet(self.decoder_ctx, self.packet) < 0) {
                if (debug_mode) std.debug.print("Frame Undecoded\n", .{});
                return error.UnableToDecodeFrame;
            }

            const receive_result = codec.avcodec_receive_frame(self.decoder_ctx, self.source_frame_buffer);
            if (receive_result < 0) {
                if (debug_mode) std.debug.print("Receieve Result Invalid\n", .{});
                continue;
            }
            defer codec.av_frame_unref(self.source_frame_buffer);

            const scaled_height = codec.sws_scale(
                self.scaler,
                @ptrCast(&self.source_frame_buffer.*.data),
                @ptrCast(&self.source_frame_buffer.*.linesize),
                0,
                self.decoder_ctx.*.height,
                @ptrCast(&self.decoded_frame_buffer.*.data),
                @ptrCast(&self.decoded_frame_buffer.*.linesize),
            );

            if (scaled_height < 0) {
                return error.UnableToScaleFrame;
            }

            const frame_data = self.decoded_frame_buffer.*.data;
            const frame = frame_data[0];

            const no_padding_stride: usize = if(self.output_format == .rgb) @intCast(self.metadata.width * 3) else @intCast(self.metadata.width);
            const padded_stride: usize = @intCast(self.metadata.output_stride);
            for(0..@intCast(self.metadata.height))|i|{
                const source_start = i * padded_stride;
                const buffer_start = i * no_padding_stride;

                const row = frame[source_start..source_start + no_padding_stride];

                @memcpy(buffer[buffer_start..buffer_start+no_padding_stride], row);
            }
            return;
        }
    }

    pub fn get_frame_buffer(self: *VideoMemory, smp: std.mem.Allocator, stack_depth: ?usize) ![]u8{
        const height = self.metadata.height;
        const stride = self.metadata.width * 3;

        const total_pixels: usize = @intCast(height * stride);

        if(stack_depth)|depth|{
            if(depth < 1){
                return error.ZeroDepthStackRequested;
            } else {
                return try smp.alloc(u8, depth * total_pixels);
            }
        } else{
            return try smp.alloc(u8, total_pixels);
        }

    }
};

test "pull video" {
    var video_path = "tests/ViewTester/Capture0000.mov".*;
    const video_slice: [:0]const u8 = &video_path;

    const smp = std.heap.smp_allocator;
    var video = VideoMemory.init(smp, video_slice);
    defer video.deinit();

    const test_buffer = try smp.alloc(u16, @intCast(video.metadata.width * video.metadata.height));
    defer smp.free(test_buffer);
    try video.get_frame(test_buffer);

    var max: u16 = 0;
    for (0..test_buffer.len) |j| {
        if (test_buffer[j] > max) {
            max = test_buffer[j];
        }
    }

    std.debug.print("Max Value: {d}\n", .{max});
}
