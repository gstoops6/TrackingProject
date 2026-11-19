const std = @import("std");
const ffmpeg = @import("build/ffmpeg.zig");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{
        .default_target = if (@import("builtin").os.tag == .windows)
            .{ .cpu_arch = .x86_64, .os_tag = .windows, .abi = .msvc }
        else
            .{},
    });
    const optimize = b.standardOptimizeOption(.{});
    const install_dir = try ffmpeg.prepare(b, target);
    b.addNamedLazyPath("ffmpeg", install_dir);

    const install_ffmpeg = b.addInstallDirectory(.{
        .source_dir = install_dir,
        .install_dir = .prefix,
        .install_subdir = "",
    });
    const ffmpeg_step = b.step("ffmpeg", "Build and install FFmpeg from source");
    ffmpeg_step.dependOn(&install_ffmpeg.step);
    b.getInstallStep().dependOn(ffmpeg_step);

    const lib = b.addLibrary(.{
        .name = "Recording",
        .linkage = .static,
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/Recording.zig"),
            .target = target,
            .optimize = optimize,
            .link_libc = true,
        }),
    });
    ffmpeg.link(b, lib.root_module, install_dir, target.result.os.tag);
    b.installArtifact(lib);
}
