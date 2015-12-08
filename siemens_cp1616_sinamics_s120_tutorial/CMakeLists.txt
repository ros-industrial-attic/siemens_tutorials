cmake_minimum_required(VERSION 2.8.3)
project(siemens_cp1616_sinamics_s120_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(siemens_cp1616_sinamics_s120_tutorial_node src/siemens_cp1616_sinamics_s120_tutorial_node.cpp)
target_link_libraries(siemens_cp1616_sinamics_s120_tutorial_node ${catkin_LIBRARIES})