cmake_minimum_required(VERSION 2.8.3)
project(InverseKinematics)
add_definitions(-std=c++11 -Wall -O2)

find_package( Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
        ./include/InverseKinematics
        ./include/Utility
)

add_library( InverseKinematics ./include/InverseKinematics/InverseKinematics.cpp)

add_executable(InvKin_test test/InvKin_test.cpp)
target_link_libraries(InvKin_test InverseKinematics)
