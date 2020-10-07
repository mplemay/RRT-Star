cmake_minimum_required(VERSION 3.17)
project(zebra_assignent_part_2)

set(CMAKE_CXX_STANDARD 20)

add_executable(zebra_assignment_part_2 main.cpp RTTStar.hpp Costmap2D.hpp ${PCL_LIBRARIES})