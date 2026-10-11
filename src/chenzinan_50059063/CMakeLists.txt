cmake_minimum_required(VERSION 3.16)

project(assignmnet LANGUAGES CXX)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio features2d calib3d)

set(OpenCV_DIR /usr/include/opencv4)

add_executable(PJ ./main.cpp)
target_include_directories(PJ PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(PJ PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(PJ PRIVATE ${OpenCV_LIBS})

target_compile_features(PJ PRIVATE cxx_std_17)
set_target_properties(PJ PROPERTIES CXX_EXTENSIONS OFF)
