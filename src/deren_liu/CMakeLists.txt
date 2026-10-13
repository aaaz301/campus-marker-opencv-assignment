cmake_minimum_required(VERSION 3.16)
project(light_panel_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc videoio)

add_executable(white_demo src/main.cpp)
target_include_directories(white_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(white_demo PRIVATE ${OpenCV_LIBS})