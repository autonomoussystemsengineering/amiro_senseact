cmake_minimum_required(VERSION 2.6)

PROJECT("HarrisCornerDetection")

FIND_PACKAGE(OpenCV REQUIRED)
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(cornerdetection main.cxx)
TARGET_LINK_LIBRARIES(cornerdetection ${OpenCV_LIBS})
