cmake_minimum_required(VERSION 3.26)
project(EE5110_CA2)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(EE5110_CA2 main.cpp
        EventCamera.h
        EventCamera.cpp)

target_link_libraries(EE5110_CA2 ${OpenCV_LIBS})