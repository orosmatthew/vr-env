cmake_minimum_required(VERSION 3.27)
project(vr_env)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV CONFIG REQUIRED)

add_subdirectory(external/raylib-5.0)
add_subdirectory(external/raylib-cpp-5.0.0)
add_subdirectory(external/json-3.11.3)
add_subdirectory(external/raygui-4.0)

add_executable(vr_env src/main.cpp)
target_link_libraries(vr_env raylib raylib_cpp)

add_executable(camera_calibration src/camera_calibration/main.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS} raylib raylib_cpp json raygui)
