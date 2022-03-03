cmake_minimum_required(VERSION 3.10)
project(EigenTest)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories(/usr/local/include)

add_executable(eigenMatrix eigenMatrix.cpp)
