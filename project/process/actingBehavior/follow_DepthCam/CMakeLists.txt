cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(RST rst0.13)
pkg_check_modules(OPENCV opencv)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(OPENNI2 OpenNI2 )

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system chrono filesystem thread)
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
    ${OPENNI2_INCLUDE_DIRS}
)

# Define the source
SET (SOURCE main.cxx)

if (NOT TARGET vecIntConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
endif()

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${RST_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
    ${OPENNI2_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE} $ENV{MUROX_INCLUDE_DIRS}/Color.cpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.hpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.cxx)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OPENNI2_LIBRARIES}
    jpeg
    # -- Own libraries
    vecIntConverter
)

# Additional compiler flags
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O3 -ffast-math -funsafe-math-optimizations -std=c++0x")

SET ( CMAKE_BUILD_TYPE Release )
#ADD_DEFINITIONS (-DCMAKE_BUILD_TYPE=Release -Wno-deprecated-declarations -O3)
