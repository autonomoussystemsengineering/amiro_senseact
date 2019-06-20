cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
find_package(PkgConfig)
find_package(OpenCV REQUIRED)

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

LINK_DIRECTORIES (
    ${OPENCV_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wno-deprecated-declarations -O2)
