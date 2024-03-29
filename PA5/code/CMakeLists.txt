cmake_minimum_required(VERSION 3.10)
project(PA4)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/local/include")

add_executable(ORB computeORB.cpp)
target_link_libraries(ORB ${OpenCV_LIBS})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${EIGEN3_LIBS} Sophus::Sophus)

add_executable(BA GN-BA.cpp)
target_link_libraries(BA ${EIGEN3_LIBS} Sophus::Sophus)

add_executable(ICP ICP.cpp)
target_link_libraries(ICP ${EIGEN3_LIBS} Sophus::Sophus ${Pangolin_LIBRARIES})