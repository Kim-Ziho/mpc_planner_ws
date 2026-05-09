cmake_minimum_required(VERSION 3.8)
project(mpc_planner_rosnavigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP (OPENMP not found)")
endif()

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  nav2_costmap_2d
  nav2_util
  ros_tools
  mpc_planner
  mpc_planner_msgs
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
)

add_executable(jackal_planner src/ros2_rosnavigation.cpp)
target_include_directories(jackal_planner PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(jackal_planner ${DEPENDENCIES})

add_executable(costmap_pair_node src/costmap_pair_node.cpp)
ament_target_dependencies(costmap_pair_node
  rclcpp
  nav2_costmap_2d
  nav2_util
  ament_index_cpp
)

install(TARGETS jackal_planner costmap_pair_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/goal_publisher.py
  scripts/pedsim_starter.py
  scripts/navfn_goal_caller.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPENDENCIES})
ament_package()
