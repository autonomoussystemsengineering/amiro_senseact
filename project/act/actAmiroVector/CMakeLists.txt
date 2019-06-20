cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)
pkg_check_modules(PROTOBUF protobuf)
if ($ENV{ARCH} MATCHES "arm")
    pkg_check_modules(RSB rsb0.13)
    pkg_check_modules(RSC rsc0.13)
    pkg_check_modules(RST rstsandbox0.13)
else()
    pkg_check_modules(RSB rsb0.16)
    pkg_check_modules(RSC rsc0.16)
    pkg_check_modules(RST rstsandbox0.16)
endif()

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    4{RST_INCLUDE_DIRS}                                             # RST
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)


# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)
ADD_DEFINITIONS(${RST_CFLAGS})
