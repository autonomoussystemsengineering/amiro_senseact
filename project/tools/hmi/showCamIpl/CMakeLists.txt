cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(RSB rsb0.11)
pkg_check_modules(RSC rsc0.11)
pkg_check_modules(RST rst0.11)
pkg_check_modules(OPENCV opencv)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
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
)

# Define the source
SET (SOURCE main.cxx)

# Add sources to build
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/iplImageConverterSource.cmake")

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${RST_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENCV_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wno-deprecated-declarations -O2 ${RST_CFLAGS})
