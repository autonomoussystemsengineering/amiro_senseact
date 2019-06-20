cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)

# Set the include pathe
include_directories(
    ${LIBV4l2_INCLUDE_DIRS}                                         # Video for Linux
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
    ${GXX_INCLUDES}
)


message(STATUS " RST ${RST_INCLUDE_DIRS}")

file(GLOB SOURCES *.cxx)
file(GLOB HEADERS *.hpp)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCES} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/Color.cpp)

# Link the executable to the library. 
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)
target_link_libraries (
    "${PROJECT_NAME}"
    # Boost
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    ${PROTO_LIBRARIES}

)

# Get rid of depricated warnings

SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2 -std=c++0x"
)
