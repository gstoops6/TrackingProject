const std = @import("std");

pub const Paths = struct {
    include_dir: []const u8,
    lib_dir: []const u8,
    bin_dir: []const u8,
    build_step: *std.Build.Step,
};

fn pathExists(b: *std.Build, relative_path: []const u8) bool {
    std.Io.Dir.cwd().access(b.graph.io, b.pathFromRoot(relative_path), .{}) catch return false;
    return true;
}

fn hasInstalledOpenCv(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    install_dir: []const u8,
) bool {
    const is_windows = target.result.os.tag == .windows;

    // This corresponds to the include path returned below:
    // dependencies/OpenCV/<target>/include/opencv5
    if (!pathExists(
        b,
        b.fmt("{s}/include/opencv5/opencv2/core.hpp", .{install_dir}),
    ) or !pathExists(
        b,
        b.fmt("{s}/include/opencv5/opencv2/tracking.hpp", .{install_dir}),
    )) {
        return false;
    }

    for ([_][]const u8{
        "core",
        "flann",
        "geometry",
        "imgproc",
        "imgcodecs",
        "videoio",
        "dnn",
        "features",
        "highgui",
        "xfeatures2d",
        "imgcodecs",
        "plot",
        "tracking"
    }) |module_name| {
        const library = switch (target.result.os.tag) {
            .windows => b.fmt("{s}/lib/opencv_{s}520.lib", .{
                install_dir,
                module_name,
            }),
            .macos => b.fmt("{s}/lib/libopencv_{s}.dylib", .{
                install_dir,
                module_name,
            }),
            .linux => b.fmt("{s}/lib/libopencv_{s}.so", .{
                install_dir,
                module_name,
            }),
            else => return false,
        };

        if (!pathExists(b, library)) return false;
    }

    // Avoid an unused-variable issue if target support is later narrowed.
    _ = is_windows;
    return true;
}

pub fn prepare(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    clean_dependency_builds: bool,
) !Paths {
    if (target.result.os.tag != b.graph.host.result.os.tag or
        target.result.cpu.arch != b.graph.host.result.cpu.arch or
        (target.result.os.tag == .linux and
            target.result.abi != b.graph.host.result.abi))
    {
        return error.OpenCVRequiresNativeTarget;
    }

    if (target.result.os.tag == .windows and
        (target.result.cpu.arch != .x86_64 or target.result.abi != .msvc))
    {
        return error.OpenCVRequiresX64WindowsMsvc;
    }

    const target_name = b.fmt("{s}-{s}-{s}", .{
        @tagName(target.result.cpu.arch),
        @tagName(target.result.os.tag),
        @tagName(target.result.abi),
    });

    const install_dir = b.fmt("dependencies/OpenCV/{s}", .{target_name});
    const build_dir = b.fmt("build/opencv/{s}", .{target_name});

    // Keep `zig build opencv` available. It is a no-op when artifacts exist.
    const opencv_step = b.step(
        "opencv",
        "Build and install OpenCV only when required artifacts are missing",
    );

    // Important: keep b.dependency() below this check. Otherwise the build
    // still resolves the OpenCV source/contrib dependency before deciding
    // whether the installed artifacts are usable.
    if (!hasInstalledOpenCv(b, target, install_dir)) {
        const source = b.dependency("opencv", .{});
        const contrib = b.dependency("opencv_contrib", .{});

        const build_cmd = b.addSystemCommand(&.{ "cmake" });
        build_cmd.addArg(b.fmt("-Dtarget_name={s}", .{target_name}));
        build_cmd.addArg(b.fmt(
            "-Dclean_dependency_builds={s}",
            .{if (clean_dependency_builds) "ON" else "OFF"},
        ));
        build_cmd.addPrefixedDirectoryArg("-Dsource_dir=", source.path(""));
        build_cmd.addPrefixedDirectoryArg("-Dcontrib_dir=", contrib.path(""));
        build_cmd.addArg(b.fmt(
            "-Dbuild_dir={s}",
            .{b.pathFromRoot(build_dir)},
        ));
        build_cmd.addArg(b.fmt(
            "-Dinstall_dir={s}",
            .{b.pathFromRoot(install_dir)},
        ));
        build_cmd.addArg(b.fmt("-Dzig_exe={s}", .{b.graph.zig_exe}));
        build_cmd.addArg("-P");
        build_cmd.addFileArg(b.path("build/build_opencv.cmake"));
        build_cmd.has_side_effects = true;

        opencv_step.dependOn(&build_cmd.step);
    }

    return .{
        .include_dir = b.fmt("{s}/include/opencv5", .{install_dir}),
        .lib_dir = b.fmt("{s}/lib", .{install_dir}),
        .bin_dir = b.fmt("{s}/bin", .{install_dir}),
        .build_step = opencv_step,
    };
}