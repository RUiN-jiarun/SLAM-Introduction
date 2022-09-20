cmake_minimum_required(VERSION 3.10)
project(DrawTrajectory)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/include")

add_executable(DrawTrajectory draw_trajectory.cpp)
target_link_libraries(DrawTrajectory Sophus::Sophus)
target_link_libraries(DrawTrajectory ${Sophus_LIBARARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${EIGEN3_LIBS})