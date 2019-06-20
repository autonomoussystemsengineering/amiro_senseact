cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Check for build types and set the tty-outputs accordingly
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/chkBuildTypes.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)

pkg_check_modules(PROTOBUF protobuf)
find_package(PkgConfig)
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
    ${Boost_INCLUDE_DIR}
    "$ENV{MUROX_INCLUDE_DIRS}/libs/CoreSLAM"
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})
# ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE} $ENV{MUROX_INCLUDE_DIRS}/types/LocatedLaserScan.pb.cc)

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

# Additional compiler flags
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11 -funsafe-math-optimizations -Wno-unused-variable" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -funsafe-math-optimizations -Wno-unused-variable" )
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
