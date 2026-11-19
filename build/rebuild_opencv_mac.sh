#!/bin/bash
set -euo pipefail

# Run from any directory. Requires Git, CMake, and Xcode Command Line Tools.
# Downloads contrib on the first run; installs over deps/OpenCVMac.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$project_dir/deps/OpenCVSource"
contrib_dir="$project_dir/deps/OpenCVContrib"
build_dir="$source_dir/build_mac_akaze"
install_dir="$project_dir/deps/OpenCVMac"
build_jobs="${build_jobs:-$(sysctl -n hw.ncpu)}"

if [[ "$(uname -s)" != "Darwin" ]]; then
    printf 'This script builds OpenCV for macOS.\n' >&2
    exit 1
fi

for required_command in git cmake clang clang++; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        printf 'Required command not found: %s\n' "$required_command" >&2
        exit 1
    fi
done

if [[ ! -f "$source_dir/CMakeLists.txt" ]]; then
    printf 'OpenCV source not found: %s\n' "$source_dir" >&2
    exit 1
fi

# This project's OpenCVSource checkout is on the 5.x development branch.
# Keep an existing contrib checkout unchanged when rerunning the script.
if [[ ! -e "$contrib_dir" ]]; then
    git clone --depth 1 --branch 5.x --single-branch \
        https://github.com/opencv/opencv_contrib.git "$contrib_dir"
fi

if [[ ! -f "$contrib_dir/modules/xfeatures2d/CMakeLists.txt" ]]; then
    printf 'Missing contrib xfeatures2d module: %s\n' "$contrib_dir" >&2
    exit 1
fi

# Preserve the installed module set and add xfeatures2d (AKAZE).
# BUILD_LIST prevents building unrelated contrib modules.
cmake -S "$source_dir" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_INSTALL_PREFIX="$install_dir" \
    '-DCMAKE_INSTALL_RPATH=@loader_path' \
    -DBUILD_SHARED_LIBS=ON \
    -DOPENCV_EXTRA_MODULES_PATH="$contrib_dir/modules" \
    -DBUILD_LIST=core,imgproc,imgcodecs,highgui,videoio,features,xfeatures2d,flann,geometry,calib,stereo,ptcloud,dnn,objdetect,video,photo,stitching \
    -DBUILD_opencv_xfeatures2d=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_objc=OFF

cmake --build "$build_dir" --config Release --parallel 6
cmake --install "$build_dir" --config Release

if [[ ! -f "$install_dir/include/opencv5/opencv2/xfeatures2d.hpp" || \
      ! -f "$install_dir/lib/libopencv_xfeatures2d.dylib" ]]; then
    printf 'Installation is missing the xfeatures2d header or library.\n' >&2
    exit 1
fi

printf 'OpenCV with AKAZE installed into: %s\n' "$install_dir"
