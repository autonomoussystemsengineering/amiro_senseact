cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(OPENCV opencv)
pkg_check_modules(LIBV4l2 libv4l2)

# Not needed. The libraries will be set manualy
# link_directories(
#     ${RSB_LIBRARY_DIRS}
#     ${OPENCV_LIBRARY_DIRS}
#     ${LIBV4l2_LIBRARY_DIRS}
# )

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${RSTSANDBOX_INCLUDE_DIRS}                                      # RST syndbox types
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
)

# Add folders to build and run CMakeLists.txt in it
add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/matConverter lib/matConverter)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx)

# Set the compiler
# SET (CMAKE_C_COMPILER $ENV{CPP})
# SET (CMAKE_CXX_COMPILER $ENV{CXX})

# Link the executable to the library. 
# SET (BLAH "$ENV{SDKTARGETSYSROOT}/usr/lib")
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)

# Link the executable to the library.
# link_directories (
#     "/usr/lib"                             # For standard libraries
#     "/usr/local/lib/"                      # For OpenCV libraries
#     "/usr/lib/x86_64-linux-gnu/"           # For boost libraries
# )

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    # boost_regex-mt
    # boost_date_time-mt
    # boost_program_options-mt
    boost_system
    opencv_core
    opencv_highgui
    # -- Own libraries
    matConverter
)

# Get rid of depricated warnings
SET (
    CMAKE_C_FLAGS
    -Wno-deprecated-declarations
)

SET (
    CMAKE_CXX_FLAGS
    -Wno-deprecated-declarations
)
