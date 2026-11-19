cmake_minimum_required(VERSION 3.21)
include("${CMAKE_CURRENT_LIST_DIR}/clean_dependency_builds.cmake")

# Zig supplies the downloaded source directory. Build outside its immutable cache.
# Requires a native C compiler and CMake; Windows also needs VS C++/LLVM tools.
foreach(required_var source_dir build_dir install_dir library_name)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "Missing -D${required_var}=...")
    endif()
endforeach()
if(NOT EXISTS "${source_dir}/CMakeLists.txt")
    message(FATAL_ERROR "OpenBLAS sources missing: ${source_dir}")
endif()

# The dependency path includes Zig's content hash. Script changes also invalidate the stamp.
file(SHA256 "${CMAKE_CURRENT_LIST_FILE}" script_hash)
set(build_key "${source_dir}\n${script_hash}\n${library_name}\n")
set(stamp_file "${install_dir}/.build_stamp")
if(EXISTS "${stamp_file}" AND EXISTS "${install_dir}/lib/${library_name}"
        AND EXISTS "${install_dir}/include/openblas/cblas.h"
        AND EXISTS "${install_dir}/include/openblas/openblas_config.h")
    file(READ "${stamp_file}" installed_key)
    if(installed_key STREQUAL build_key)
        if(clean_dependency_builds)
            cleanup_installed_build(openblas "${target_name}")
        endif()
        return()
    endif()
endif()

# A different dependency revision must not reuse a CMake cache for the old source tree.
if(EXISTS "${build_dir}/CMakeCache.txt")
    file(STRINGS "${build_dir}/CMakeCache.txt" cached_source REGEX "^CMAKE_HOME_DIRECTORY:INTERNAL=")
    file(TO_CMAKE_PATH "${source_dir}" normalized_source)
    if(NOT cached_source STREQUAL "CMAKE_HOME_DIRECTORY:INTERNAL=${normalized_source}")
        string(SHA256 source_hash "${source_dir}")
        set(build_dir "${build_dir}/${source_hash}")
    endif()
endif()

set(configure_args
    -S "${source_dir}" -B "${build_dir}"
    "-DCMAKE_INSTALL_PREFIX=${install_dir}"
    -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_INCLUDEDIR=include
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=OFF
    -DBUILD_WITHOUT_LAPACK=OFF -DNOFORTRAN=ON
    -DBUILD_WITHOUT_LAPACKE=OFF -DC_LAPACKE=ON
    -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=OFF
    -DUSE_THREAD=ON -DUSE_OPENMP=OFF -DFIXED_LIBNAME=ON)

if(CMAKE_HOST_WIN32)
    # Find the LLVM and Ninja tools bundled with Visual Studio when absent from PATH.
    set(vswhere_path "$ENV{ProgramFiles\(x86\)}/Microsoft Visual Studio/Installer/vswhere.exe")
    if(EXISTS "${vswhere_path}")
        execute_process(COMMAND "${vswhere_path}" -latest -products *
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64
            -property installationPath OUTPUT_VARIABLE vs_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    if(EXISTS "${vs_dir}/VC/Auxiliary/Build/vcvars64.bat")
        file(MAKE_DIRECTORY "${build_dir}")
        file(WRITE "${build_dir}/compiler_environment.cmd"
            "@echo off\ncall \"${vs_dir}/VC/Auxiliary/Build/vcvars64.bat\" >nul\nif errorlevel 1 exit /b 1\nset\n")
        execute_process(COMMAND cmd /d /c compiler_environment.cmd WORKING_DIRECTORY "${build_dir}"
            OUTPUT_VARIABLE compiler_environment RESULT_VARIABLE environment_result)
        if(NOT environment_result STREQUAL "0")
            message(FATAL_ERROR "Could not initialize the Visual Studio x64 toolchain")
        endif()
        string(REPLACE ";" "\\;" compiler_environment "${compiler_environment}")
        string(REPLACE "\n" ";" compiler_environment "${compiler_environment}")
        foreach(environment_line IN LISTS compiler_environment)
            string(STRIP "${environment_line}" environment_line)
            if(environment_line MATCHES "^([^=]+)=(.*)$")
                set("ENV{${CMAKE_MATCH_1}}" "${CMAKE_MATCH_2}")
            endif()
        endforeach()
    endif()
    find_program(clang_cl_path clang-cl HINTS "${vs_dir}/VC/Tools/Llvm/x64/bin")
    find_program(ninja_path ninja HINTS
        "${vs_dir}/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja")
    if(NOT clang_cl_path OR NOT ninja_path)
        message(FATAL_ERROR "OpenBLAS requires clang-cl and Ninja. Install the VS C++ and LLVM tools.")
    endif()
    list(APPEND configure_args -G Ninja "-DCMAKE_MAKE_PROGRAM=${ninja_path}"
        "-DCMAKE_C_COMPILER=${clang_cl_path}" -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL)
endif()

execute_process(COMMAND "${CMAKE_COMMAND}" ${configure_args} RESULT_VARIABLE configure_result)
if(NOT configure_result STREQUAL "0")
    message(FATAL_ERROR "OpenBLAS configuration failed (${configure_result})")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${build_dir}" --config Release --target openblas_static --parallel 6
    RESULT_VARIABLE build_result)
if(NOT build_result STREQUAL "0")
    message(FATAL_ERROR "OpenBLAS build failed (${build_result})")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${build_dir}" --config Release
    RESULT_VARIABLE install_result)
if(NOT install_result STREQUAL "0")
    message(FATAL_ERROR "OpenBLAS installation failed (${install_result})")
endif()
if(NOT EXISTS "${install_dir}/lib/${library_name}" OR NOT EXISTS "${install_dir}/include/openblas/cblas.h"
        OR NOT EXISTS "${install_dir}/include/openblas/openblas_config.h")
    message(FATAL_ERROR "OpenBLAS installation is missing its static library or headers")
endif()
file(WRITE "${stamp_file}" "${build_key}")
if(clean_dependency_builds)
    cleanup_installed_build(openblas "${target_name}")
endif()
