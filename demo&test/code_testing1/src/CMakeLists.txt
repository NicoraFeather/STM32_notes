cmake_minimum_required(VERSION 3.10)

project(hello)

add_library(a a.cpp)

add_executable(hello main.cpp)

target_link_libraries(main a)