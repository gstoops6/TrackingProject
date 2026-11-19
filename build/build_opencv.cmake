cmake_minimum_required(VERSION 3.21)
include("${CMAKE_CURRENT_LIST_DIR}/clean_dependency_builds.cmake")

# Configure OpenCV's own CMake build using source trees fetched by Zig.
foreach(required_var source_dir contrib_dir build_dir install_dir zig_exe)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "Missing -D${required_var}=...")
    endif()
endforeach()
if(NOT EXISTS "${source_dir}/CMakeLists.txt" OR
        NOT EXISTS "${contrib_dir}/modules/xfeatures2d/CMakeLists.txt")
    message(FATAL_ERROR "OpenCV and matching contrib sources are required")
endif()

# Keep CMake caches separate when either content-addressed source tree changes.
string(SHA256 source_hash "${source_dir}\n${contrib_dir}")
string(SUBSTRING "${source_hash}" 0 12 source_hash)
set(build_dir "${build_dir}/${source_hash}")
file(SHA256 "${CMAKE_CURRENT_LIST_FILE}" script_hash)
set(build_key "${source_dir}\n${contrib_dir}\n${script_hash}\n${zig_exe}\n")
set(stamp_file "${install_dir}/.build_stamp")
set(manifest_file "${install_dir}/.install_manifest.txt")
set(required_modules core flann geometry imgproc imgcodecs videoio dnn features highgui xfeatures2d tracking plot)
set(required_files
    "${install_dir}/include/opencv5/opencv2/core/version.hpp"
    "${install_dir}/include/opencv5/opencv2/xfeatures2d.hpp"
    "${install_dir}/include/opencv5/opencv2/tracking.hpp")
foreach(module_name IN LISTS required_modules)
    if(CMAKE_HOST_WIN32)
        list(APPEND required_files "${install_dir}/lib/opencv_${module_name}520.lib"
            "${install_dir}/bin/opencv_${module_name}520.dll")
    elseif(CMAKE_HOST_APPLE)
        list(APPEND required_files "${install_dir}/lib/libopencv_${module_name}.dylib")
    else()
        list(APPEND required_files "${install_dir}/lib/libopencv_${module_name}.so")
    endif()
endforeach()
if(EXISTS "${stamp_file}" AND EXISTS "${manifest_file}")
    file(READ "${stamp_file}" installed_key)
    if(installed_key STREQUAL build_key)
        file(STRINGS "${manifest_file}" installed_files)
        set(installation_complete TRUE)
        foreach(installed_file IN LISTS required_files installed_files)
            if(NOT EXISTS "${installed_file}")
                set(installation_complete FALSE)
                break()
            endif()
        endforeach()
        if(installation_complete)
            if(clean_dependency_builds)
                cleanup_installed_build(opencv "${target_name}")
            endif()
            return()
        endif()
    endif()
endif()

#-DCMAKE_ASM_COMPILER:FILEPATH=OFF
# Preserve the module set and nonfree setting from the existing rebuild scripts.
set(configure_args -S "${source_dir}" -B "${build_dir}"
    "-DCMAKE_INSTALL_PREFIX=${install_dir}"
    "-DOPENCV_EXTRA_MODULES_PATH=${contrib_dir}/modules"
    "-DOPENCV_DOWNLOAD_PATH=${build_dir}/downloads"
     -DOPENCV_PYTHON_SKIP_DETECTION=ON
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
    -DOPENCV_BIN_INSTALL_PATH=bin -DOPENCV_LIB_INSTALL_PATH=lib
    -DOPENCV_LIB_ARCHIVE_INSTALL_PATH=lib -DOPENCV_INCLUDE_INSTALL_PATH=include/opencv5
    -DOPENCV_CONFIG_INSTALL_PATH=lib/cmake/opencv5
    -DBUILD_LIST=core,imgproc,imgcodecs,highgui,videoio,features,xfeatures2d,flann,geometry,calib,stereo,ptcloud,dnn,objdetect,video,photo,stitching,tracking
    -DBUILD_SHARED_LIBS=ON -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_opencv_world=OFF
    -DBUILD_opencv_xfeatures2d=ON -DOPENCV_ENABLE_NONFREE=ON -DWITH_CUDA=OFF
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
    -DBUILD_opencv_tracking=ON -DBUILD_opencv_plot=ON
    -DBUILD_opencv_apps=OFF -DBUILD_opencv_python3=OFF -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_java=OFF -DBUILD_opencv_objc=OFF)
if(CMAKE_HOST_WIN32)
    list(APPEND configure_args -G "Visual Studio 17 2022" -A x64 #"Visual Studio 17 2022"
        -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL)
elseif(CMAKE_HOST_APPLE)
    list(APPEND configure_args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
        "-DCMAKE_INSTALL_RPATH=@loader_path")
else()
    # Zig's C++ compiler uses the same libc++ ABI as the project's C++ wrapper.
    list(APPEND configure_args "-DCMAKE_C_COMPILER=${zig_exe}" -DCMAKE_C_COMPILER_ARG1=cc
        "-DCMAKE_CXX_COMPILER=${zig_exe}" -DCMAKE_CXX_COMPILER_ARG1=c++
        "-DCMAKE_ASM_COMPILER=${zig_exe}" -DCMAKE_ASM_COMPILER_ARG1=cc
        -DOPENCV_SKIP_IPP_EXCLUDE_LIBS=ON -DOPENCV_SKIP_IPP_EXCLUDE_LIBS_CORE=ON)
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" ${configure_args} RESULT_VARIABLE configure_result)
if(NOT configure_result STREQUAL "0")
    message(FATAL_ERROR "OpenCV configuration failed (${configure_result})")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${build_dir}" --config Release --parallel 6
    RESULT_VARIABLE build_result)
if(NOT build_result STREQUAL "0")
    message(FATAL_ERROR "OpenCV build failed (${build_result})")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${build_dir}" --config Release
    RESULT_VARIABLE install_result)
if(NOT install_result STREQUAL "0")
    message(FATAL_ERROR "OpenCV installation failed (${install_result})")
endif()
foreach(required_file IN LISTS required_files)
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR "OpenCV installation is missing ${required_file}")
    endif()
endforeach()
configure_file("${build_dir}/install_manifest.txt" "${manifest_file}" COPYONLY)
file(WRITE "${stamp_file}" "${build_key}")
if(clean_dependency_builds)
    cleanup_installed_build(opencv "${target_name}")
endif()
