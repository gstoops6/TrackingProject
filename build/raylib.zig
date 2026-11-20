const std = @import("std");

pub fn install(
    b: *std.Build,
    raylib_dep: *std.Build.Dependency,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
) void {
    const install_dir = b.fmt("dependencies/raylib_zig/{s}-{s}-{s}/{s}", .{
        @tagName(target.result.cpu.arch), @tagName(target.result.os.tag),
        @tagName(target.result.abi), @tagName(optimize),
    });
    // Use the package's own build and installation steps, including its C headers.
    const raylib_builder = raylib_dep.builder;
    raylib_builder.resolveInstallPrefix(b.pathFromRoot(install_dir), .{});
    const install_bindings = raylib_builder.addInstallDirectory(.{
        .source_dir = raylib_dep.path("lib"),
        .install_dir = .prefix,
        .install_subdir = "bindings",
    });
    const install_step = raylib_builder.getInstallStep();
    install_step.dependOn(&install_bindings.step);
    b.getInstallStep().dependOn(install_step);

    const raylib_step = b.step("raylib", "Build and install packaged raylib and Zig bindings");
    raylib_step.dependOn(install_step);
}
