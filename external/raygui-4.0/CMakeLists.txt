cmake_minimum_required(VERSION 3.27)
project(raygui)

set(CMAKE_CXX_STANDARD 20)

add_library(raygui INTERFACE)
target_include_directories(raygui INTERFACE include)