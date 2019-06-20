cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
pkg_check_modules(PROTOBUF protobuf)
pkg_check_modules(OpenCV opencv)
if ($ENV{ARCH} MATCHES "arm")
  pkg_check_modules(RSB rsb$ENV{MUROX_RSB_VERSION_ARM})
  pkg_check_modules(RSC rsc$ENV{MUROX_RSB_VERSION_ARM})
  pkg_check_modules(RST rstsandbox$ENV{MUROX_RSB_VERSION_ARM})
  if($ENV{MUROX_RSB_VERSION_ARM} LESS 0.14)
    ADD_DEFINITIONS(-DRST_013_USED)
  endif()
else()
  pkg_check_modules(RSB rsb$ENV{MUROX_RSB_VERSION})
  pkg_check_modules(RSC rsc$ENV{MUROX_RSB_VERSION})
  pkg_check_modules(RST rstsandbox$ENV{MUROX_RSB_VERSION})
endif()
# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})



include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    # ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})
# ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE}  $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc  $ENV{MUROX_INCLUDE_DIRS}/types/twbTracking.pb.cc  )

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
    opencv_core
    opencv_highgui
    opencv_imgproc
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
