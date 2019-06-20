cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)

# Link the executable to the library.

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    # -- boost
    boost_system
    boost_thread
    boost_regex
    boost_filesystem
    boost_program_options
    curses
    pthread
)

# Get rid of depricated warnings

SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2"
)
