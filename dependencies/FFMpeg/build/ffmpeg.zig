const std = @import("std");

const libraries = [_][]const u8{
    "avformat",
    "avcodec",
    "swscale",
    "swresample",
    "avutil",
};

pub fn prepare(b: *std.Build, target: std.Build.ResolvedTarget) !std.Build.LazyPath {
    const os_tag = target.result.os.tag;
    const host = b.graph.host.result;
    if (os_tag != host.os.tag or target.result.cpu.arch != host.cpu.arch) {
        return error.FFmpegRequiresNativeTarget;
    }

    const target_os = switch (os_tag) {
        .windows => "mingw32",
        .macos => "darwin",
        .linux => "linux",
        else => return error.UnsupportedFFmpegTarget,
    };
    const arch = @tagName(target.result.cpu.arch);
    const target_name = b.fmt("{s}-{s}-{s}", .{
        arch, @tagName(os_tag), @tagName(target.result.abi),
    });
    const build_dir = b.pathFromRoot(b.fmt("build/{s}", .{target_name}));

    // Build Windows DLLs with Zig's MinGW libc. The import libraries can also
    // be linked into the project's MSVC-targeted executable.
    const zig_target = if (os_tag == .windows)
        b.fmt("{s}-windows-gnu", .{arch})
    else
        try target.result.zigTriple(b.allocator);

    const source = b.dependency("ffmpeg", .{});
    const build_cmd = b.addSystemCommand(&.{"cmake"});
    build_cmd.has_side_effects = true;
    build_cmd.addPrefixedDirectoryArg("-Dsource_dir=", source.path(""));
    build_cmd.addArg(b.fmt("-Dbuild_dir={s}", .{build_dir}));
    build_cmd.addArg(b.fmt("-Dzig_exe={s}", .{b.graph.zig_exe}));
    build_cmd.addArg(b.fmt("-Dzig_version={s}", .{@import("builtin").zig_version_string}));
    build_cmd.addArg(b.fmt("-Dzig_target={s}", .{zig_target}));
    build_cmd.addArg(b.fmt("-Dtarget_os={s}", .{target_os}));
    build_cmd.addArg(b.fmt("-Darch={s}", .{arch}));
    if (os_tag == .windows) {
        const msys2_root = b.graph.environ_map.get("MSYS2_ROOT") orelse "C:/msys64";
        build_cmd.addArg(b.fmt("-Dmsys2_root={s}", .{msys2_root}));
    }
    build_cmd.addArg("-P");
    build_cmd.addFileArg(b.path("build/build_ffmpeg.cmake"));

    // Keep the build dependency on both the libraries and the install directory
    // while storing the files outside Zig's build cache.
    const install_dir = try b.allocator.create(std.Build.GeneratedFile);
    install_dir.* = .{
        .step = &build_cmd.step,
        .path = b.pathJoin(&.{ build_dir, "install" }),
    };
    return .{ .generated = .{ .file = install_dir } };
}

pub fn link(
    b: *std.Build,
    module: *std.Build.Module,
    install_dir: std.Build.LazyPath,
    os_tag: std.Target.Os.Tag,
) void {
    module.addIncludePath(install_dir.path(b, "include"));
    for (libraries) |name| {
        const archive = if (os_tag == .windows)
            b.fmt("lib/{s}.lib", .{name})
        else
            b.fmt("lib/lib{s}.a", .{name});
        module.addObjectFile(install_dir.path(b, archive));
    }
}
