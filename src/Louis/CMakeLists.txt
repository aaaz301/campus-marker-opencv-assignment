cmake_minimum_required(VERSION 3.10)
project(armor_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)

add_executable(armor src/main.cpp src/Detector.cpp)
target_include_directories(armor PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(armor PRIVATE ${OpenCV_LIBS})
