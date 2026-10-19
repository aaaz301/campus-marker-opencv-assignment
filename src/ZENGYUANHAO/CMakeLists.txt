cmake_minimum_required(VERSION 3.16)
project(Main LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_executable(Main
    src/main.cpp
    src/frame_proceed.cpp
)

target_include_directories(Main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Main PRIVATE
    ${OpenCV_LIBS}
)