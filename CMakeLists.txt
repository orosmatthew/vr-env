cmake_minimum_required(VERSION 3.27)
project(vr_env)

set(CMAKE_CXX_STANDARD 20)

if (MSVC)
    option(STATIC_LINKING "Build using static linking (ON) or dynamic linking (OFF)" OFF)
    if (STATIC_LINKING)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else ()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()


find_path(RAYGUI_INCLUDE_DIRS "raygui.h")
find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(asio CONFIG REQUIRED)

add_subdirectory(external/raylib-5.0)
add_subdirectory(external/raylib-cpp-5.0.0)

add_executable(vr_env src/env/main.cpp)
target_link_libraries(vr_env PRIVATE
        asio::asio
        Eigen3::Eigen
        raylib
        raylib_cpp)

add_executable(camera_calibration src/camera_calibration/main.cpp)
target_include_directories(camera_calibration PRIVATE ${RAYGUI_INCLUDE_DIRS})
target_link_libraries(camera_calibration PRIVATE
        ${OpenCV_LIBS}
        raylib
        raylib_cpp
        nlohmann_json::nlohmann_json)

add_executable(aruco src/aruco/main.cpp src/aruco/network.cpp)
target_include_directories(aruco PRIVATE ${RAYGUI_INCLUDE_DIRS})
target_link_libraries(aruco PRIVATE ${OpenCV_LIBS}
        raylib
        raylib_cpp
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
        asio::asio)
