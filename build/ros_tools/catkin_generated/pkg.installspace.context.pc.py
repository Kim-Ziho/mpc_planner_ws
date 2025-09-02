# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;roslib;tf;std_msgs;geometry_msgs;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lros_tools".split(';') if "-lros_tools" != "" else []
PROJECT_NAME = "ros_tools"
PROJECT_SPACE_DIR = "/workspace/install"
PROJECT_VERSION = "1.0.0"
