cmake_minimum_required(VERSION 3.10)

project(LidarProject)

# directory 'cmake' contains macros to find thirdparty libraries
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(Eigen3 3.3.4 REQUIRED)

add_subdirectory(Lidar)
add_subdirectory(main)