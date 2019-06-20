cmake_minimum_required(VERSION 2.6)

# Set the project name to the name of the folder
include("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Check for build types and set the tty-outputs accordingly
include("$ENV{MUROX_CMAKE_MODULES}/chkBuildTypes.cmake")

# Read the pkg-config modules
find_package(PkgConfig)

if("$ENV{ARCH}" MATCHES arm)
    pkg_check_modules(PROTOBUF protobuf)
    pkg_check_modules(RSB rsb0.13)
    pkg_check_modules(RSC rsc0.13)
    pkg_check_modules(RST rstsandbox0.13)
    pkg_check_modules(EIGEN eigen3)
    pkg_check_modules(OpenCV opencv)
else("$ENV{ARCH}" MATCHES arm)
    find_package(RSB REQUIRED)
    find_package(RSC REQUIRED)
    find_package(RST COMPONENTS sandbox)
    find_package(Protobuf REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV REQUIRED)
endif("$ENV{ARCH}" MATCHES arm)

# Get modules
set(BOOST_COMPONENTS regex date_time program_options system thread)
include("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
include("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include path
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Define the source files
set(SOURCE
    main.cxx
    particlefilter.cpp
    sensormodel.h
#    raycastingmodel.cpp
    likelihoodfieldmodel.h
    map.h
    sampleset.h
    laserscan.h
    $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc
    $ENV{MUROX_INCLUDE_DIRS}/types/PoseEuler.pb.cc
    $ENV{MUROX_INCLUDE_DIRS}/types/TargetPoseEuler.pb.cc
    $ENV{MUROX_INCLUDE_DIRS}/types/RotationEuler.pb.cc
    $ENV{MUROX_INCLUDE_DIRS}/converter/vecFloatConverter/main.hpp
    importancetype/importancetype.h
    importancetype/addgaussians.h
    importancetype/inversedistance.h
    importancetype/multiplygaussians.h
    importancetype/multiplygaussianslog.h
)

include("$ENV{MUROX_CMAKE_MODULES}/vecFloatConverterSource.cmake")

# Add the executable
add_executable("${PROJECT_NAME}" ${SOURCE})

target_link_libraries(
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# Additional compiler flags
add_definitions(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
# Common flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations -Wno-write-strings")
# Debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O2 -g -pg -fprofile-arcs -ftest-coverage")
# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math -funsafe-loop-optimizations -Wunsafe-loop-optimizations")
