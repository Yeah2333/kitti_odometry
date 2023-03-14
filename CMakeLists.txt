cmake_minimum_required(VERSION 3.20)
project(kitti_odometry)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${BOOST_INCLUDE_DIRS})

add_executable(kitti_odometry main.cpp)
target_link_libraries(kitti_odometry ${PCL_LIBRARIES} ${BOOST_LIBRARIES})
