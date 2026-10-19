cmake_minimum_required(VERSION 3.16)
project(HELLO LANGUAGES CXX)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)

add_executable(marker_demo main.cpp)
target_compile_features(marker_demo PRIVATE cxx_std_17)
set_target_properties(marker_demo PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(marker_demo ${OpenCV_LIBS})