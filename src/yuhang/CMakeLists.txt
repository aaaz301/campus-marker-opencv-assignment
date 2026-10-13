cmake_minimum_required(VERSION 3.16)
project(campus_marker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core videoio highgui imgproc)

add_executable(marker_detector main.cpp)
target_include_directories(marker_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(marker_detector PRIVATE ${OpenCV_LIBS})
