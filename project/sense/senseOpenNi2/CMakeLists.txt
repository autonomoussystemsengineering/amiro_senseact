cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
find_package(PkgConfig)
if ($ENV{ARCH} MATCHES "arm")
  pkg_check_modules(RSB rsb$ENV{MUROX_RSB_VERSION_ARM})
  pkg_check_modules(RSC rsc$ENV{MUROX_RSB_VERSION_ARM})
  pkg_check_modules(RST rstsandbox$ENV{MUROX_RSB_VERSION_ARM})
  pkg_check_modules(OPENNI2 OpenNI2)
  if($ENV{MUROX_RSB_VERSION_ARM} LESS 0.14)
    ADD_DEFINITIONS(-DRST_013_USED)
  endif()
else()
  pkg_check_modules(RSB rsb$ENV{MUROX_RSB_VERSION})
  pkg_check_modules(RSC rsc$ENV{MUROX_RSB_VERSION})
  pkg_check_modules(RST rstsandbox$ENV{MUROX_RSB_VERSION})
endif()
pkg_check_modules(OPENCV opencv)
pkg_check_modules(EIGEN eigen3)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
)
if ($ENV{ARCH} MATCHES "arm")
  include_directories (${OPENNI2_INCLUDE_DIRS})
else()
  include_directories ("$ENV{OPENNI2_INCLUDE}")
endif()

# Define the source
SET (SOURCE main.cxx)

# Add sources to build
# INCLUDE("$ENV{MUROX_CMAKE_MODULES}/iplImageConverterSource.cmake")

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
)
if ($ENV{ARCH} MATCHES "arm")
  LINK_DIRECTORIES (${OPENNI2_LIBRARY_DIRS})
else()
  LINK_DIRECTORIES ("$ENV{OPENNI2_REDIST}")
endif()


message(status OPENCV_LIBRARY_DIRS: ${OPENCV_LIBRARY_DIRS})

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENCV_LIBRARIES}
)
if ($ENV{ARCH} MATCHES "arm")
  target_link_libraries ("${PROJECT_NAME}" ${OPENNI2_LIBRARIES})
else()
  target_link_libraries("${PROJECT_NAME}" "$ENV{OPENNI2_REDIST}" -lOpenNI2)
endif()

# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set compiler flags as you like
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11" )
