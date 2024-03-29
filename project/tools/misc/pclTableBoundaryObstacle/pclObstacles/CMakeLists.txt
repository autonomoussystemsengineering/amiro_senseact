cmake_minimum_required(VERSION 2.6)
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

SET(NAME pclObstacles)
PROJECT(${NAME})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Pcl
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS common segmentation visualization filters features surface)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# Log4cxx
FIND_PACKAGE(Log4cxx REQUIRED)
LINK_DIRECTORIES(${LOG4CXX_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${LOG4CXX_INCLUDE_DIRS})

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

FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/logging.properties DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(src)
