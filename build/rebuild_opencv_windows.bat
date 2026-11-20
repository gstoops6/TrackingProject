@echo off
setlocal EnableExtensions DisableDelayedExpansion

rem Requires CMake and Visual Studio 2022 with Desktop development with C++.
rem Uses the existing OpenCV 5.2 and matching contrib sources in deps.
rem Run from any directory. Existing installed OpenCV files are overwritten.
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?" goto :usage
if not "%~1"=="" goto :invalid_argument

for %%i in ("%~dp0..") do set "project_dir=%%~fi"
set "source_dir=%project_dir%\deps\OpenCVSource"
set "contrib_dir=%project_dir%\deps\OpenCVContrib"
set "build_dir=%project_dir%\build\opencv_windows"
set "install_dir=%project_dir%\deps\OpenCVWindows"
if not defined build_jobs set "build_jobs=6"

where cmake >nul 2>&1
if errorlevel 1 (
    echo ERROR: CMake must be installed and available on PATH. 1>&2
    exit /b 1
)
if not exist "%source_dir%\CMakeLists.txt" (
    echo ERROR: OpenCV source not found at "%source_dir%". 1>&2
    exit /b 1
)
if not exist "%contrib_dir%\modules\xfeatures2d\CMakeLists.txt" (
    echo ERROR: Matching contrib xfeatures2d source not found at "%contrib_dir%". 1>&2
    exit /b 1
)

rem Keep separate DLLs and the x64/vc17 layout expected by build.zig.
rem BUILD_LIST preserves the existing module set and adds xfeatures2d.
cmake -S "%source_dir%" -B "%build_dir%" ^
    -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_INSTALL_PREFIX:PATH="%install_dir%" ^
    -DCMAKE_CXX_STANDARD=17 ^
    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL ^
    -DOPENCV_EXTRA_MODULES_PATH:PATH="%contrib_dir%\modules" ^
    -DOPENCV_BIN_INSTALL_PATH=x64/vc17/bin ^
    -DOPENCV_LIB_INSTALL_PATH=x64/vc17/lib ^
    -DOPENCV_INCLUDE_INSTALL_PATH=include ^
    -DBUILD_LIST=core,imgproc,imgcodecs,highgui,videoio,features,xfeatures2d,flann,geometry,calib,stereo,ptcloud,dnn,objdetect,video,photo,stitching ^
    -DBUILD_SHARED_LIBS=ON ^
    -DBUILD_WITH_STATIC_CRT=OFF ^
    -DBUILD_opencv_world=OFF ^
    -DBUILD_opencv_xfeatures2d=ON ^
    -DOPENCV_ENABLE_NONFREE=ON ^
    -DWITH_CUDA=OFF ^
    -DBUILD_TESTS=OFF ^
    -DBUILD_PERF_TESTS=OFF ^
    -DBUILD_EXAMPLES=OFF ^
    -DBUILD_opencv_apps=OFF ^
    -DBUILD_opencv_python3=OFF ^
    -DBUILD_opencv_java=OFF ^
    -DBUILD_opencv_objc=OFF
if errorlevel 1 goto :failed

cmake --build "%build_dir%" --config Release --parallel "%build_jobs%"
if errorlevel 1 goto :failed

cmake --install "%build_dir%" --config Release
if errorlevel 1 goto :failed

if not exist "%install_dir%\include\opencv2\xfeatures2d.hpp" goto :missing_artifact
if not exist "%install_dir%\x64\vc17\lib\opencv_xfeatures2d520.lib" goto :missing_artifact
if not exist "%install_dir%\x64\vc17\bin\opencv_xfeatures2d520.dll" goto :missing_artifact

echo OpenCV with xfeatures2d installed into "%install_dir%".
exit /b 0

:missing_artifact
echo ERROR: Installation is missing the OpenCV 5.2 xfeatures2d header, library, or DLL. 1>&2
exit /b 1

:failed
echo ERROR: OpenCV build or installation failed. See the output above. 1>&2
exit /b 1

:invalid_argument
echo ERROR: Unexpected argument "%~1". 1>&2
exit /b 1

:usage
echo Usage: %~nx0
echo Requires CMake and Visual Studio 2022 with the C++ workload and Windows SDK.
echo Uses deps\OpenCVSource and deps\OpenCVContrib; installs into deps\OpenCVWindows.
echo Build files are placed in build\opencv_windows relative to the project root.
echo Set build_jobs before running to override the default of 6 parallel jobs.
echo Nonfree algorithms are enabled. Configuration may download third-party data.
exit /b 0
