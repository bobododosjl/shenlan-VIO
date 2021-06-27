cmake_minimum_required(VERSION 2.8)
project(VIO1)

set( CMAKE_BUILD_TYPE "Release" )
#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
#Eigen
include_directories("/usr/include/eigen3")

add_executable(VIO1 eigensophus.cpp)


target_link_libraries( VIO1 ${Sophus_LIBRARIES} )
