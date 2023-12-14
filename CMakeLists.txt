cmake_minimum_required(VERSION 3.27)
project(vr_env)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(external/raylib-5.0)
add_subdirectory(external/raylib-cpp-5.0.0)

add_executable(vr_env src/main.cpp)

target_link_libraries(vr_env raylib raylib_cpp)