cmake_minimum_required(VERSION 3.27)
project(json)

set(CMAKE_CXX_STANDARD 20)

add_library(json INTERFACE)
target_include_directories(json INTERFACE include)