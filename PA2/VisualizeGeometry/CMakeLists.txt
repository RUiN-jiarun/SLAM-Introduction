cmake_minimum_required(VERSION 3.10)
project(VisualizeGeometry)

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
include_directories(/usr/local/include)

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
