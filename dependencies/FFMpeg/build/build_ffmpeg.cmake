cmake_minimum_required(VERSION 3.21)

foreach(required_var source_dir install_dir zig_exe zig_target target_os arch)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "Missing -D${required_var}=...")
    endif()
endforeach()
if(NOT EXISTS "${source_dir}/configure")
    message(FATAL_ERROR "FFmpeg sources missing: ${source_dir}")
endif()

if(CMAKE_HOST_WIN32)
    # Install make and binutils in MSYS2. Use the MSYS shell and tools even
    # when Zig is invoked from PowerShell or the Visual Studio command prompt.
    file(TO_CMAKE_PATH "${msys2_root}" msys2_root)
    set(msys2_bin "${msys2_root}/usr/bin")
    foreach(tool bash make nm)
        if(NOT EXISTS "${msys2_bin}/${tool}.exe")
            message(FATAL_ERROR
                "FFmpeg requires MSYS2 with make and binutils (missing ${msys2_bin}/${tool}.exe). Set MSYS2_ROOT for a custom installation.")
        endif()
    endforeach()
    set(shell_command "${msys2_bin}/bash.exe" --noprofile --norc -c
        [=[export PATH=/usr/bin:/bin && exec "$@"]=] ffmpeg)
    set(make_command make)
else()
    find_program(make_command NAMES gmake make REQUIRED)
endif()

# FFmpeg's configure/Makefiles assume a source path without spaces. Build
# inside a private copy and use relative source and compiler wrapper paths.
set(work_dir "${install_dir}/work")
file(REMOVE_RECURSE "${work_dir}")
file(MAKE_DIRECTORY "${work_dir}")
file(COPY "${source_dir}/" DESTINATION "${work_dir}")
file(READ "${work_dir}/configure" configure_script)
if(target_os STREQUAL "darwin")
    # Zig's Mach-O linker rejects FFmpeg's unconditional Apple ld flags.
    string(REPLACE "add_ldflags -Wl,-dynamic,-search_paths_first"
        "check_ldflags -Wl,-dynamic,-search_paths_first" configure_script "${configure_script}")
elseif(target_os STREQUAL "linux")
    # Older glibc symbol tables still expose sysctl without its removed header.
    string(REPLACE "check_func  sysctl" "check_func_headers sys/sysctl.h sysctl"
        configure_script "${configure_script}")
endif()
# Let the compiler supply atomics when possible before adding libatomic.
string(REPLACE [=[for LATOMIC in "-latomic" ""; do]=] [=[for LATOMIC in "" "-latomic"; do]=]
    configure_script "${configure_script}")
file(WRITE "${work_dir}/configure" "${configure_script}")
file(GLOB build_scripts "${work_dir}/ffbuild/*.sh")
file(CHMOD ${build_scripts} "${work_dir}/compat/windows/makedef"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

file(TO_CMAKE_PATH "${zig_exe}" zig_exe)
string(REPLACE "'" "'\"'\"'" zig_exe "${zig_exe}")
function(write_zig_wrapper name command)
    file(WRITE "${work_dir}/${name}" "#!/bin/sh\nexec '${zig_exe}' ${command} \"\$@\"\n")
    file(CHMOD "${work_dir}/${name}"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()
write_zig_wrapper(zig_cc "cc -target ${zig_target}")
write_zig_wrapper(zig_ar ar)
write_zig_wrapper(zig_ranlib ranlib)

function(run_ffmpeg)
    execute_process(COMMAND ${shell_command} ${ARGN}
        WORKING_DIRECTORY "${work_dir}"
        RESULT_VARIABLE result)
    if(NOT result STREQUAL "0")
        message(FATAL_ERROR "FFmpeg command failed (${result}): ${ARGN}. See ${work_dir}/ffbuild/config.log for configure probes.")
    endif()
endfunction()

set(configure_args
    --cc=./zig_cc --host-cc=./zig_cc --ar=./zig_ar --ranlib=./zig_ranlib
    "--prefix=${install_dir}" "--target-os=${target_os}" "--arch=${arch}"
    --disable-autodetect --disable-iconv --disable-stripping
    --disable-programs --disable-doc --disable-avdevice --disable-avfilter)
if(arch STREQUAL "x86" OR arch STREQUAL "x86_64")
    list(APPEND configure_args --enable-x86asm)
else()
    list(APPEND configure_args --disable-x86asm)
endif()
if(target_os STREQUAL "mingw32")
    write_zig_wrapper(zig_dlltool dlltool)
    # GNU windres would invoke a separate C preprocessor for DLL metadata.
    list(APPEND configure_args --enable-shared --disable-static --windres=false)
    # FFmpeg selects dlltool internally for MinGW. Override it at make time.
    set(make_args DLLTOOL=./zig_dlltool)
else()
    list(APPEND configure_args --enable-static --disable-shared --enable-pic)
endif()

run_ffmpeg(sh ./configure ${configure_args})
run_ffmpeg("${make_command}" -r -j6 ${make_args})
run_ffmpeg("${make_command}" -r ${make_args} install)

foreach(name avformat avcodec swscale swresample avutil)
    if(target_os STREQUAL "mingw32")
        file(GLOB dlls "${install_dir}/bin/${name}-*.dll")
        list(LENGTH dlls dll_count)
        if(NOT dll_count EQUAL 1 OR NOT EXISTS "${install_dir}/bin/${name}.lib")
            message(FATAL_ERROR "FFmpeg installation is missing the ${name} DLL or import library")
        endif()
        set(archive "${install_dir}/lib/${name}.lib")
        file(RENAME "${install_dir}/bin/${name}.lib" "${archive}")
    else()
        set(archive "${install_dir}/lib/lib${name}.a")
    endif()
    if(NOT EXISTS "${archive}" OR NOT EXISTS "${install_dir}/include/lib${name}/${name}.h")
        message(FATAL_ERROR "FFmpeg installation is missing ${name} headers or library")
    endif()

    # Keep pkg-config usable after Zig copies the installation to its prefix.
    set(pc_file "${install_dir}/lib/pkgconfig/lib${name}.pc")
    file(READ "${pc_file}" pc_contents)
    string(REGEX REPLACE "^prefix=[^\n]*" [=[prefix=${pcfiledir}/../..]=] pc_contents "${pc_contents}")
    string(REGEX REPLACE "\nlibdir=[^\n]*" "\nlibdir=\${prefix}/lib" pc_contents "${pc_contents}")
    string(REGEX REPLACE "\nincludedir=[^\n]*" "\nincludedir=\${prefix}/include" pc_contents "${pc_contents}")
    file(WRITE "${pc_file}" "${pc_contents}")
endforeach()
file(REMOVE_RECURSE "${work_dir}")
