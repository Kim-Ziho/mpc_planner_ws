if(USE_ROS2)
	find_package(guidance_planner REQUIRED)
	find_package(decomp_util REQUIRED)
endif()
set(MODULE_DEPENDENCIES
	guidance_planner
	decomp_util
)

set(MODULE_SOURCES
	src/mpc_base.cpp
	src/contouring.cpp
	src/guidance_constraints.cpp
	src/linearized_constraints.cpp
	src/ellipsoid_constraints.cpp
	src/decomp_constraints.cpp
)
