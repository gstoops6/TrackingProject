const std = @import("std");
const openblas = @import("build/openblas.zig");
const raylib_build = @import("build/raylib.zig");
const opencv_build = @import("build/opencv.zig");
const builtin = @import("builtin");

const PathsOpenCV = opencv_build.Paths;



/// Builds the GUI executable with Recording and the packaged raylib/raygui dependencies.
pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{
        .default_target = if (@import("builtin").os.tag == .windows)
            .{ .cpu_arch = .x86_64, .os_tag = .windows, .abi = .msvc }
        else
            .{},
    });

    const target_fmt = b.fmt("{s}-{s}-{s}", .{
        @tagName(target.result.cpu.arch),
        @tagName(target.result.os.tag),
        @tagName(target.result.abi),
    });

    //const installed_openblas_path = b.fmt("dependencies/OpenBLAS/{}", .{target_fmt});
    //const installed_opencv_path = b.fmt("dependencies/OpenCV/{}", .{target_fmt});
    //const installed_raylib_path = b.fmt("dependencies/raylib_zig/{}", .{target_fmt});

    // Default to Debug; other modes can be selected with -Doptimize.
    const optimize = b.standardOptimizeOption(.{});
    const clean_dependency_builds = b.option(bool, "clean-dependency-builds", "Remove OpenCV and OpenBLAS build directories after successful installation") orelse false;



    switch (target.result.os.tag) {
        .macos, .windows, .linux => {},
        else => return error.UnsupportedTarget,
    }

    const test_step = b.step("test", "Run the KalmanFilter tests using installed OpenBLAS");
    for ([_][]const u8{
        "src/KalmanFilter/KalmanFilter.zig",
        "src/KalmanFilter/Matrix.zig",
        "src/KalmanFilter/Stats.zig",
        "src/KalmanFilter/IMM.zig",
    }) |test_file| {
        const kalman_tests = b.addTest(.{
            .root_module = b.createModule(.{
                .root_source_file = b.path(test_file),
                .target = target,
                .optimize = optimize,
                .link_libc = true,
            }),
        });
        // OpenBLAS uses the DLL CRT; let Zig select the UCRT/VCRuntime import libraries.
        kalman_tests.linkage = if (target.result.os.tag == .windows and target.result.abi == .msvc) .dynamic else null;
        kalman_tests.root_module.addIncludePath(b.path(b.fmt("dependencies/OpenBLAS/{s}/include/openblas", .{target_fmt})));
        kalman_tests.root_module.addObjectFile(b.path(b.fmt("dependencies/OpenBLAS/{s}/lib/{s}", .{
            target_fmt, if (target.result.os.tag == .windows) "openblas.lib" else "libopenblas.a",
        })));
        if (target.result.os.tag == .linux) {
            kalman_tests.root_module.linkSystemLibrary("m", .{ .use_pkg_config = .no });
            kalman_tests.root_module.linkSystemLibrary("pthread", .{ .use_pkg_config = .no });
        }
        const run_tests = b.addRunArtifact(kalman_tests);
        test_step.dependOn(&run_tests.step);
    }

    const cleanup_cmd = b.addSystemCommand(&.{ "cmake", "-Dcleanup_package=all" });
    cleanup_cmd.addArg(b.fmt("-Dtarget_name={s}-{s}-{s}", .{
        @tagName(target.result.cpu.arch), @tagName(target.result.os.tag), @tagName(target.result.abi),
    }));
    cleanup_cmd.addArg("-P");
    cleanup_cmd.addFileArg(b.path("build/clean_dependency_builds.cmake"));
    cleanup_cmd.has_side_effects = true;
    const cleanup_step = b.step("clean-dependency-builds", "Remove installed dependencies' intermediate build directories");
    cleanup_step.dependOn(&cleanup_cmd.step);

    const ffmpeg = b.dependency("ffmpeg", .{
        .target = target,
        .optimize = optimize,
    });
    const ffmpeg_paths = ffmpeg.namedLazyPath("ffmpeg");
    const install_ffmpeg = b.addInstallDirectory(.{
        .source_dir = ffmpeg_paths,
        .install_dir = .prefix,
        .install_subdir = "",
    });
    const ffmpeg_step = b.step("ffmpeg", "Build and install FFmpeg from source");
    ffmpeg_step.dependOn(&install_ffmpeg.step);
    const raylib = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });

    const recording = ffmpeg.artifact("Recording");
    try b.modules.put(b.allocator, "recording", recording.root_module);
    try b.modules.put(b.allocator, "raylib", raylib.module("raylib"));
    try b.modules.put(b.allocator, "raygui", raylib.module("raygui"));

    const exe = b.addExecutable(.{
        .name = "Tracker",
        .linkage = if (target.result.os.tag == .windows and target.result.abi == .msvc) .dynamic else null,
        .root_module = b.createModule(
            .{ .root_source_file = b.path("src/main.zig"),
                .target = target,
                .optimize = optimize,
                .imports = &.{
                    .{ .name = "recording", .module = recording.root_module },
                    .{ .name = "raylib", .module = raylib.module("raylib") },
                    .{ .name = "raygui", .module = raylib.module("raygui") },
                },
                .link_libc = true, .link_libcpp = target.result.abi != .msvc
            }),
    });

    const options = b.addOptions();
    options.addOption(std.builtin.OptimizeMode, "optimize", optimize);

    exe.root_module.addOptions("Optimization", options);


    const opencv_paths = try opencv_build.prepare(b, target, clean_dependency_builds);
    exe.step.dependOn(opencv_paths.build_step);

    const is_windows = target.result.os.tag == .windows;
    // The Windows source build uses the x64 MSVC C++ ABI.
    if (is_windows and (target.result.cpu.arch != .x86_64 or target.result.abi != .msvc)) {
        return error.OpenCVRequiresX64WindowsMsvc;
    }

    exe.root_module.addIncludePath(b.path("src/Tracking"));
    exe.root_module.addCSourceFile(.{
        .file = b.path("src/Tracking/OpenCVAKAZE.cpp"),
        .flags = if (is_windows)
            &.{ "-std=c++17", "-fms-runtime-lib=dll", "-D_ITERATOR_DEBUG_LEVEL=0" }
        else
            &.{"-std=c++17"},
    });
    exe.root_module.addCSourceFile(.{
        .file = b.path("src/Tracking/OpenCVCamshift.cpp"),
        .flags = if (is_windows)
            &.{ "-std=c++17", "-fms-runtime-lib=dll", "-D_ITERATOR_DEBUG_LEVEL=0" }
        else
            &.{"-std=c++17"},
    });
    exe.root_module.addCSourceFile(.{
        .file = b.path("src/Tracking/OpenCVKCF.cpp"),
        .flags = if (is_windows)
            &.{ "-std=c++17", "-fms-runtime-lib=dll", "-D_ITERATOR_DEBUG_LEVEL=0" }
        else
            &.{"-std=c++17"},
    });
    exe.root_module.addIncludePath(b.path(opencv_paths.include_dir));

    if (is_windows) {
        raylib.module("raylib").linkSystemLibrary("user32", .{ .use_pkg_config = .no });
        raylib.module("raylib").linkSystemLibrary("shell32", .{ .use_pkg_config = .no });
        exe.root_module.linkSystemLibrary("msvcprt", .{ .use_pkg_config = .no });
    } else {
        exe.root_module.addLibraryPath(b.path(opencv_paths.lib_dir));
        exe.root_module.addRPath(b.path(opencv_paths.lib_dir));
    }

    // Include runtime dependencies of features, highgui, and xfeatures2d.
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
        "video",
        "tracking"
    }) |module_name| {
        if (is_windows) {
            exe.root_module.addObjectFile(b.path(b.fmt(
                "{s}/opencv_{s}520.lib",
                .{ opencv_paths.lib_dir, module_name },
            )));
        } else {
            exe.root_module.linkSystemLibrary(b.fmt("opencv_{s}", .{module_name}), .{
                .use_pkg_config = .no,
                .preferred_link_mode = .dynamic,
            });
        }
    }
    if (is_windows) {
        // Include transitive OpenCV modules and any installed video backend DLLs.
        const install_dlls = b.addInstallDirectory(.{
            .source_dir = b.path(opencv_paths.bin_dir),
            .install_dir = .bin,
            .install_subdir = "",
            .include_extensions = &.{".dll"},
        });
        install_dlls.step.dependOn(opencv_paths.build_step);
        b.getInstallStep().dependOn(&install_dlls.step);
    }

    try openblas.link(b, exe, target, clean_dependency_builds);

    exe.root_module.addIncludePath(b.path(b.fmt("dependencies/OpenBLAS/{s}/include/openblas", .{target_fmt})));
    exe.root_module.addObjectFile(b.path(b.fmt("dependencies/OpenBLAS/{s}/lib/{s}", .{ target_fmt, if (builtin.os.tag == .windows) "openblas.lib" else "libopenblas.a" })));

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);

    const run_step = b.step("run", "Run the GUI");
    run_step.dependOn(&run_cmd.step);

    b.installArtifact(recording);
    raylib_build.install(b, raylib, target, optimize);

    if (target.result.os.tag == .windows) {
        const install_dlls = b.addInstallDirectory(.{
            .source_dir = ffmpeg_paths.path(b, "bin"),
            .install_dir = .bin,
            .install_subdir = "",
            .include_extensions = &.{".dll"},
        });
        b.getInstallStep().dependOn(&install_dlls.step);
    }
}
