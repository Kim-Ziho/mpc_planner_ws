cmake_minimum_required(VERSION 3.8)
project(mpc_planner_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(DEPENDENCIES
  mpc_planner_util
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

# Pull in solver-specific sources/libraries (acados or forces)
include(solver.cmake)

include_directories(
  include
  ${solver_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/mpc_planner_parameters.cpp
  src/state.cpp
  ${solver_SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
set(_solver_system_include_dirs)
set(_solver_local_include_dirs)
foreach(dir ${solver_INCLUDE_DIRS})
  if(IS_ABSOLUTE "${dir}")
    list(APPEND _solver_system_include_dirs "${dir}")
  else()
    list(APPEND _solver_local_include_dirs "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
  endif()
endforeach()
foreach(dir ${_solver_local_include_dirs})
  target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${dir}>")
endforeach()
if(_solver_system_include_dirs)
  target_include_directories(${PROJECT_NAME} PUBLIC ${_solver_system_include_dirs})
endif()

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${solver_LIBRARIES})

# Propagate solver selection (-DACADOS_SOLVER or -DFORCES_SOLVER from solver.cmake) to downstream
get_directory_property(_dir_defs COMPILE_DEFINITIONS)
foreach(_def IN LISTS _dir_defs)
  if(_def MATCHES "^(ACADOS_SOLVER|FORCES_SOLVER)$")
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${_def})
  endif()
endforeach()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install generated solver headers (Solver/include/*) so downstream can find them
install(DIRECTORY Solver/include/
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

# Install acados-generated solver headers so downstream `#include "Solver/..."` resolves
install(DIRECTORY acados/Solver
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
# Export absolute system include dirs (acados/blasfeo/hpipm) so downstream consumers find them
if(_solver_system_include_dirs)
  ament_export_include_directories(${_solver_system_include_dirs})
endif()
ament_package()
