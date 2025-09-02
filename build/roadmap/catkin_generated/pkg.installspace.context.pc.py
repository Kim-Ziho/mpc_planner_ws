# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;roslib;roscpp;std_msgs;asr_rapidxml;roadmap_msgs;ros_tools".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lroadmap".split(';') if "-lroadmap" != "" else []
PROJECT_NAME = "roadmap"
PROJECT_SPACE_DIR = "/workspace/install"
PROJECT_VERSION = "0.0.0"
