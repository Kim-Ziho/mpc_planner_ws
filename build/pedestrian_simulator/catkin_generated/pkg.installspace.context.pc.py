# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;roscpp;std_msgs;ros_tools;pedsim_original;mpc_planner_msgs;asr_rapidxml;roslib;std_srvs;message_filters".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpedestrian_simulator".split(';') if "-lpedestrian_simulator" != "" else []
PROJECT_NAME = "pedestrian_simulator"
PROJECT_SPACE_DIR = "/workspace/install"
PROJECT_VERSION = "0.0.0"
