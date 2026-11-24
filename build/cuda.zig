const std = @import("std");
const os_tag = @import("builtin").os.tag;



fn find_cuda(b: *std.Build, allocator: std.mem.Allocator) ![]u8 {
    if (b.graph.environ_map.get("CUDA_PATH")) |cuda_path| {
        if (cuda_path.len > 0) {
            return try allocator.dupe(u8, cuda_path);
        }
    } else if (b.graph.environ_map.get("CUDA_PATH_V13_2")) |cuda_path| {
        if (cuda_path.len > 0) {
            return try allocator.dupe(u8, cuda_path);
        }
    }

    const path_lookup = switch (os_tag) {
        .windows => [_][]const u8{ "C:/Program Files", "C:/ProgramData" },
        .linux => [_][]const u8{"/usr"},
        else => return error.NoPath,
    };

    for (path_lookup) |path| {
        var cuda_dir = try std.Io.Dir.openDirAbsolute(b.graph.io, path, .{ .iterate = true });
        var walker = try cuda_dir.walk(allocator);

        defer walker.deinit();

        while (true) {
            const file = walker.next(b.graph.io) catch {
                break;
            };

            if (file) |f| {
                switch (f.kind) {
                    .directory => {
                        if (os_tag == .windows) {
                            if (std.mem.startsWith(u8, f.path, "NVIDIA GPU Computing Toolkit\\CUDA\\v")) {
                                const output = try allocator.alloc(u8, f.path.len);
                                defer allocator.free(output);

                                _ = std.mem.replace(u8, f.path, "\\", "/", output);

                                return try std.fmt.allocPrint(allocator, "{s}/{s}", .{ path, output });
                            }
                        } else if (os_tag == .linux) {
                            if (std.mem.indexOf(u8, f.path, "local/cuda-")) |_| {
                                return try std.fmt.allocPrint(allocator, "{s}/{s}", .{ path, f.path });
                            }
                        }
                    },
                    else => continue,
                }
            } else {
                break;
            }
        }
    }

    return error.NoPath;
}

const CudaCorrelationLibrary = struct {
    dynamic_library: std.Build.LazyPath,
    import_library: std.Build.LazyPath,
};

const CudaVersion = struct {
    major: u16,
    minor: u16,
};

fn get_cuda_version(b: *std.Build, nvcc_path: []const u8) !CudaVersion {
    const result = try std.process.run(
        b.allocator,
        b.graph.io,
        .{ .argv = &.{ nvcc_path, "--version" } },
    );
    defer b.allocator.free(result.stderr);
    defer b.allocator.free(result.stdout);

    if (result.term != .exited or result.term.exited != 0) {
        return error.CudaVersionDetectionFailed;
    }

    const release_marker = "release ";
    const release_start = std.mem.indexOf(u8, result.stdout, release_marker) orelse
        return error.CudaVersionDetectionFailed;
    const version_start = release_start + release_marker.len;
    const version_end = std.mem.indexOfScalarPos(u8, result.stdout, version_start, ',') orelse
        return error.CudaVersionDetectionFailed;
    const version_text = result.stdout[version_start..version_end];
    const separator = std.mem.indexOfScalar(u8, version_text, '.') orelse
        return error.CudaVersionDetectionFailed;

    return .{
        .major = try std.fmt.parseInt(u16, version_text[0..separator], 10),
        .minor = try std.fmt.parseInt(u16, version_text[separator + 1 ..], 10),
    };
}

fn cuda_requires_conforming_preprocessor(cuda_version: CudaVersion) bool {
    return cuda_version.major > 13 or
        (cuda_version.major == 13 and cuda_version.minor >= 2);
}

fn find_nvidia_smi(b: *std.Build) ![]u8 {
    const allocator = b.allocator;

    const path_lookup = switch (os_tag) {
        .windows => [_][]const u8{"C:/Windows/System32"}, //, "C:/Program Files/NVIDIA Corporation"
        else => return error.NoPath,
    };

    for (path_lookup) |path| {
        var cuda_dir = try std.Io.Dir.openDirAbsolute(b.graph.io, path, .{ .iterate = true });
        var walker = try cuda_dir.walk(allocator);

        defer walker.deinit();

        while (true) {
            const file = walker.next(b.graph.io) catch {
                continue;
            };

            if (file) |f| {
                switch (f.kind) {
                    .file => {
                        //std.debug.print("Path: {s}. File: {s}\n", .{path, f.path});
                        if (std.mem.eql(u8, f.path, "nvidia-smi.exe")) {
                            const output = try allocator.alloc(u8, f.path.len);
                            defer allocator.free(output);

                            _ = std.mem.replace(u8, f.path, "\\", "/", output);

                            return try std.fmt.allocPrint(allocator, "{s}/{s}", .{ path, output });
                        }
                    },
                    else => continue,
                }
            } else {
                break;
            }
        }
    }

    return error.NoPath;
}

fn determine_cuda_arch(b: *std.Build) ![]u8 {
    const smi = try find_nvidia_smi(b);
    defer b.allocator.free(smi);

    var query = "--query-gpu=compute_cap,memory.total".*;

    const result = try std.process.run(b.allocator, b.graph.io, .{ .argv = &[_][]u8{ smi, &query } });
    defer b.allocator.free(result.stderr);
    defer b.allocator.free(result.stdout);

    const stdout = result.stdout;

    var split = std.mem.splitScalar(u8, stdout, '\n');
    var max_mem: usize = 0;
    var max_mem_idx: usize = 0;

    var archs: std.ArrayList(u8) = .empty;
    defer archs.deinit(b.allocator);
    var idx: usize = 0;

    _ = split.next(); //cast out first line as it is a header

    while (split.next()) |line| {
        var line_split = std.mem.splitScalar(u8, line, ',');
        var line_idx: usize = 0;

        while (line_split.next()) |desc| {
            const trim_desc = std.mem.trim(u8, desc, " ");

            if (line_idx == 1) {
                var mem_buffer: [10]u8 = undefined;
                var mem_buffer_idx: usize = 0;

                for (trim_desc) |chr| {
                    if (chr >= '0' and chr <= '9') {
                        mem_buffer[mem_buffer_idx] = chr;
                        mem_buffer_idx += 1;
                    }
                }

                const total_mem_int = try std.fmt.parseInt(usize, mem_buffer[0..mem_buffer_idx], 10);

                if (total_mem_int > max_mem) {
                    max_mem = total_mem_int;
                    max_mem_idx = idx;
                }
            } else {
                var arch_buffer: [10]u8 = undefined;
                var arch_buffer_idx: usize = 0;

                for (trim_desc) |chr| {
                    if (chr >= '0' and chr <= '9') {
                        arch_buffer[arch_buffer_idx] = chr;
                        arch_buffer_idx += 1;
                    }
                }

                if (arch_buffer_idx > 0) try archs.append(b.allocator, try std.fmt.parseInt(u8, arch_buffer[0..arch_buffer_idx], 10));
            }
            line_idx += 1;
        }
        idx += 1;
    }
    return try std.fmt.allocPrint(b.allocator, "-arch=sm_{d}", .{archs.items[max_mem_idx]});
}

pub fn link(
    b: *std.Build,
    exe: *std.Build.Step.Compile,
    target: std.Build.ResolvedTarget,
) !void {
    const target_os = target.result.os.tag;
    const library_subdir = switch (target_os) {
        .windows => "lib/x64",
        .linux => "lib64",
        else => return,
    };
    if (target_os != b.graph.host.result.os.tag or
        target.result.cpu.arch != b.graph.host.result.cpu.arch)
    {
        return error.CudaRequiresNativeTarget;
    }

    const cuda_path_override = b.option(
        []const u8,
        "cuda-path",
        "CUDA Toolkit root directory; defaults to CUDA_PATH or automatic discovery",
    );
    const cuda_dir = if (cuda_path_override) |cuda_path|
        try b.allocator.dupe(u8, b.pathFromRoot(cuda_path))
    else
        try find_cuda(b, b.allocator);
    defer b.allocator.free(cuda_dir);

    exe.root_module.addCMacro("__NV_NO_VECTOR_DEPRECATION_DIAG", "1");
    exe.root_module.addSystemIncludePath(.{
        .cwd_relative = b.fmt("{s}/include", .{cuda_dir}),
    });
    const cuda_lib: std.Build.LazyPath = .{
        .cwd_relative = b.fmt("{s}/{s}", .{ cuda_dir, library_subdir }),
    };
    exe.root_module.addLibraryPath(cuda_lib);
    if (target_os == .linux) exe.root_module.addRPath(cuda_lib);

    for ([_][]const u8{ "cudart", "cublas", "curand" }) |lib_name| {
        exe.root_module.linkSystemLibrary(lib_name, .{
            .use_pkg_config = .no,
            .preferred_link_mode = .dynamic,
            .search_strategy = .mode_first,
        });
    }
}
