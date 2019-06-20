cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)


pkg_check_modules(LIBV4l2 libv4l2)



# Set the include pathe
include_directories(
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    "../../includes"                                                # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
)

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
    rt
    m
    dl
    v4l2
)

# Get rid of depricated warnings
SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2"
)
