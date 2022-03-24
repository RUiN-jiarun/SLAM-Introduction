cmake_minimum_required(VERSION 3.10)
project(SophusTest)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/local/include")
add_executable(SophusTest useSophus.cpp)
target_link_libraries(SophusTest Sophus::Sophus)


