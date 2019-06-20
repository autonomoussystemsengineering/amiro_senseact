cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

if ("$ENV{ARCH}" MATCHES arm)
  pkg_check_modules(RSB rsb0.11)
  pkg_check_modules(RSC rsc0.11)
  pkg_check_modules(PROTOBUF protobuf)
  pkg_check_modules(RST rstsandbox0.11)
  pkg_check_modules(OpenCV opencv)
  pkg_check_modules(EIGEN eigen3)
else("$ENV{ARCH}" MATCHES arm)
  find_package(OpenCV REQUIRED)
  find_package(RSB REQUIRED)
  find_package(RSC REQUIRED)
  find_package(RST COMPONENTS sandbox)
  find_package(Protobuf REQUIRED)
  find_package(Eigen3 REQUIRED)
endif("$ENV{ARCH}" MATCHES arm)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${EIGEN_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    "$ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM"
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE} $ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM/CoreSLAM.c $ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM/CoreSLAM_loop_closing.c $ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM/CoreSLAM_state.c $ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM/CoreSLAM_random.c $ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM/CoreSLAM_ext.c $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc)

MESSAGE(INFO ${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})