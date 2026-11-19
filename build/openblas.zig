const std = @import("std");

fn pathExists(b: *std.Build, relative_path: []const u8) bool {
    std.Io.Dir.cwd().access(b.graph.io, b.pathFromRoot(relative_path), .{}) catch return false;
    return true;
}

fn hasInstalledOpenBlas(
    b: *std.Build,
    install_dir: []const u8,
    is_windows: bool,
) bool {
    const header = b.fmt("{s}/include/openblas/cblas.h", .{install_dir});
    const library = b.fmt("{s}/lib/{s}", .{
        install_dir,
        if (is_windows) "openblas.lib" else "libopenblas.a",
    });

    return pathExists(b, header) and pathExists(b, library);
}

pub fn link(
    b: *std.Build,
    exe: *std.Build.Step.Compile,
    target: std.Build.ResolvedTarget,
    clean_dependency_builds: bool,
) !void {
    // CMake uses the native toolchain; do not silently link a host library into a cross build.
    if (target.result.os.tag != b.graph.host.result.os.tag or
        target.result.cpu.arch != b.graph.host.result.cpu.arch)
    {
        return error.OpenBLASRequiresNativeTarget;
    }

    const is_windows = target.result.os.tag == .windows;
    if (is_windows and target.result.abi != .msvc) {
        return error.OpenBLASRequiresWindowsMsvc;
    }

    if (target.result.os.tag == .linux and
        target.result.abi != b.graph.host.result.abi)
    {
        return error.OpenBLASRequiresNativeTarget;
    }

    const target_name = b.fmt("{s}-{s}-{s}", .{
        @tagName(target.result.cpu.arch),
        @tagName(target.result.os.tag),
        @tagName(target.result.abi),
    });

    const install_dir = b.fmt("dependencies/OpenBLAS/{s}", .{target_name});
    const build_dir = b.fmt("build/openblas/{s}", .{target_name});

    // Keep `zig build openblas` available. It is a no-op when artifacts exist.
    const openblas_step = b.step(
        "openblas",
        "Build and install OpenBLAS only when required artifacts are missing",
    );

    // Do not resolve the source dependency or invoke CMake when the installed
    // header and library for this target already exist.
    if (!hasInstalledOpenBlas(b, install_dir, is_windows)) {
        const source = b.dependency("openblas", .{});

        const build_cmd = b.addSystemCommand(&.{ "cmake" });
        build_cmd.addArg(b.fmt("-Dtarget_name={s}", .{target_name}));
        build_cmd.addArg(b.fmt(
            "-Dclean_dependency_builds={s}",
            .{if (clean_dependency_builds) "ON" else "OFF"},
        ));
        build_cmd.addPrefixedDirectoryArg("-Dsource_dir=", source.path(""));
        build_cmd.addArg(b.fmt(
            "-Dbuild_dir={s}",
            .{b.pathFromRoot(build_dir)},
        ));
        build_cmd.addArg(b.fmt(
            "-Dinstall_dir={s}",
            .{b.pathFromRoot(install_dir)},
        ));
        build_cmd.addArg(b.fmt(
            "-Dlibrary_name={s}",
            .{if (is_windows) "openblas.lib" else "libopenblas.a"},
        ));
        build_cmd.addArg("-P");
        build_cmd.addFileArg(b.path("build/build_openblas.cmake"));
        build_cmd.has_side_effects = true;

        openblas_step.dependOn(&build_cmd.step);
        exe.step.dependOn(&build_cmd.step);
    }

    exe.root_module.addIncludePath(
        b.path(b.fmt("{s}/include/openblas", .{install_dir})),
    );
    exe.root_module.addObjectFile(
        b.path(b.fmt("{s}/lib/{s}", .{
            install_dir,
            if (is_windows) "openblas.lib" else "libopenblas.a",
        })),
    );

    if (target.result.os.tag == .linux) {
        exe.root_module.linkSystemLibrary("m", .{ .use_pkg_config = .no });
        exe.root_module.linkSystemLibrary("pthread", .{ .use_pkg_config = .no });
    }
}