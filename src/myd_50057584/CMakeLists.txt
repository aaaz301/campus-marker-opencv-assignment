cmake_minimum_required(VERSION 3.12)
project(campus_marker_recognition LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(OpenCV REQUIRED)

# --- Marker detection (required assignment) ---
add_executable(marker_detect marker_detect.cpp)
target_link_libraries(marker_detect PRIVATE
    opencv_core
    opencv_imgproc
    opencv_videoio
    opencv_highgui
    opencv_imgcodecs
)
target_include_directories(marker_detect PRIVATE ${OpenCV_INCLUDE_DIRS})

# --- Camera calibration + board pose estimation (challenge) ---
add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate PRIVATE
    opencv_core
    opencv_imgproc
    opencv_videoio
    opencv_highgui
    opencv_objdetect
    opencv_calib
    opencv_geometry
)
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})



