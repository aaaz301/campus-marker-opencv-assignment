cmake_minimum_required(VERSION 3.10)
project(CampusMarkerDetection)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(detect_marker main.cpp)

target_link_libraries(detect_marker ${OpenCV_LIBS})