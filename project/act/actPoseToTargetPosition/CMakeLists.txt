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
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(PROTOBUF protobuf)

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
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})
# ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE} $ENV{MUROX_INCLUDE_DIRS}/types/PoseEuler.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/RotationEuler.pb.cc)

MESSAGE(INFO ${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Additional compiler flags
# ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11 -funsafe-math-optimizations" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -funsafe-math-optimizations" )
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
