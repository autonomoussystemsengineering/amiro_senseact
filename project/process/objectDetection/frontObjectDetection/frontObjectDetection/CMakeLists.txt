cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")


# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(RST rst0.13)
pkg_check_modules(RSTSANDBOX rstsandbox0.13)
pkg_check_modules(LIBV4l2 libv4l2)
pkg_check_modules(OPENCV opencv)

# Need to add, because rst manual said so:
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${RSTSANDBOX_INCLUDE_DIRS}                                      # RST syndbox types
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Add folders to build and run CMakeLists.txt in it
if (NOT TARGET vecIntConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
endif()

# Define the source
SET (SOURCE main.cxx)

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${RST_LIBRARY_DIRS}
    ${RSTSANDBOX_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/Color.cpp $ENV{MUROX_INCLUDE_DIRS}/types/twbTracking.pb.cc)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENCV_LIBRARIES}
    # -- Own libraries
    vecIntConverter
    jpeg
    ${LIBV4l2_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wno-deprecated-declarations -O3 -funsafe-math-optimizations -std=c++11)
