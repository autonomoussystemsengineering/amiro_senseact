cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

if ("$ENV{ARCH}" MATCHES arm)

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
# pkg_check_modules(RSB rsb0.13)
# pkg_check_modules(RSC rsc0.13)
pkg_check_modules(PROTOBUF protobuf)
# pkg_check_modules(RST rstsandbox0.13)
pkg_check_modules(OpenCV opencv)
#pkg_check_modules(EIGEN eigen3)
#FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(RSB REQUIRED)
#FIND_PACKAGE(RSC REQUIRED)
#FIND_PACKAGE(RST REQUIRED COMPONENTS sandbox)
#    pkg_check_modules(OPENCV OpenCV)
pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(RST rst0.13)
pkg_check_modules(RSTSANDBOX rstsandbox0.13)

else("$ENV{ARCH}" MATCHES arm)

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
# pkg_check_modules(RSB rsb0.13)
# pkg_check_modules(RSC rsc0.13)
pkg_check_modules(PROTOBUF protobuf)
# pkg_check_modules(RST rstsandbox0.13)
pkg_check_modules(OpenCV opencv)
#pkg_check_modules(EIGEN eigen2)
#FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(RSB REQUIRED)
FIND_PACKAGE(RSC REQUIRED)
FIND_PACKAGE(RST REQUIRED COMPONENTS sandbox)
endif("$ENV{ARCH}" MATCHES arm)
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})



include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GXX_INCLUDES}
    # ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE}  $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc  $ENV{MUROX_INCLUDE_DIRS}/types/twbTracking.pb.cc laser_processor.cpp calc_leg_features.cpp)

MESSAGE(INFO ${Boost_LIBRARIES})

if ("$ENV{ARCH}" MATCHES arm)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    ${PROTO_LIBRARIES}
    # Boost
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    boost_chrono
    ${OpenCV_LIBRARIES}
)

else("$ENV{ARCH}" MATCHES arm)
TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
)

endif("$ENV{ARCH}" MATCHES arm)
# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O3 -g -Wno-write-strings -std=c++11)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
