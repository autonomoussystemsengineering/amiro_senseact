cmake_minimum_required(VERSION 2.4.6)
cmake_policy(SET CMP0005 NEW)
cmake_policy(SET CMP0012 NEW)
cmake_policy(SET CMP0003 NEW)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

project(kfc)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

SET(NAME RobocupKinectFloorCalibration)
PROJECT(${NAME})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Pcl
FIND_PACKAGE(PCL 1.6 REQUIRED COMPONENTS common segmentation visualization filters)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# Boost
FIND_PACKAGE(Boost 1.38.0 COMPONENTS  filesystem thread program_options)
IF(Boost_FOUND)
	SET(Boost_USE_MULTITHREADED ON)
ELSE(Boost_FOUND)
	MESSAGE(FATAL_ERROR "boost is required with components: ${BOOST_COMPONENTS}")
ENDIF()

# OpenNI
FIND_PACKAGE(OpenNI REQUIRED)
LINK_DIRECTORIES(${OPENNI_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIRS})

# Qt 4
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

# VTK
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
SET(VTK_VERSION "${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}")
MESSAGE(STATUS "Found VTK version ${VTK_VERSION}")
IF(${VTK_VERSION} VERSION_LESS "5.6")
    MESSAGE(FATAL_ERROR "VTK version 5.6 required")
ENDIF(${VTK_VERSION} VERSION_LESS "5.6")

ADD_SUBDIRECTORY(src)
