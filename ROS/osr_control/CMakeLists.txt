cmake_minimum_required(VERSION 3.0.2)
project(osr_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  roboclaw-hardware-interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
               src/control_loop.cpp
               src/hardware_interface_node.cpp
)

add_dependencies(${PROJECT_NAME}_node 
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
                     ${catkin_LIBRARIES}
)
