cmake_minimum_required(VERSION 3.27)
project(vr_env)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV CONFIG REQUIRED)

add_subdirectory(external/raylib-5.0)
add_subdirectory(external/raylib-cpp-5.0.0)
add_subdirectory(external/json-3.11.3)

add_executable(vr_env src/main.cpp)
target_link_libraries(vr_env raylib raylib_cpp)

add_executable(cv_testing src/main_cv.cpp)
target_link_libraries(cv_testing ${OpenCV_LIBS} json)
