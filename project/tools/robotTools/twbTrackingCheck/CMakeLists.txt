cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")


# Read the pkg-config modules
find_package(PkgConfig)


if ("$ENV{ARCH}" MATCHES arm)

pkg_check_modules(OPENCV OpenCV)
pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
pkg_check_modules(RST rst0.13)
pkg_check_modules(RSTSANDBOX rstsandbox0.13)

else("$ENV{ARCH}" MATCHES arm)

find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED)
find_package(Boost 1.55.0 COMPONENTS system chrono thread regex filesystem) 

endif("$ENV{ARCH}" MATCHES arm)

# Need to add, because rst manual said so:
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${RSTSANDBOX_INCLUDE_DIRS}                                      # RST syndbox types
    ${PROTO_INCLUDE_DIRS}
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
    "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams
    ${GXX_INCLUDES}
)

message(STATUS " RST ${RST_INCLUDE_DIRS}")

file(GLOB SOURCES *.cxx)
file(GLOB HEADERS *.h)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCES} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/types/enum.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/loc.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/pose.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/rotation.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/shapes.pb.cc $ENV{MUROX_INCLUDE_DIRS}/types/vertex.pb.cc)
#$ENV{MUROX_INCLUDE_DIRS}/types/twbTracking.pb.cc

# Link the executable to the library.
link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)

if ("$ENV{ARCH}" MATCHES arm)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    ${PROTO_LIBRARIES}
    # Boost
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    boost_chrono
)

else("$ENV{ARCH}" MATCHES arm)
target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    ${PROTO_LIBRARIES}
    # Boost
    ${Boost_LIBRARIES}
)
endif("$ENV{ARCH}" MATCHES arm)

# Get rid of depricated warnings

SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2 -std=c++0x"
)
