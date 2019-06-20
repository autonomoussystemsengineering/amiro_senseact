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

find_package(OpenCV REQUIRED)
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED COMPONENTS sandbox)

endif("$ENV{ARCH}" MATCHES arm)

# Need to add, because rst manual said so:
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like
    ${CMAKE_CURRENT_BINARY_DIR}                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

message(STATUS " RST ${RST_INCLUDE_DIRS}")

# Add folders to build and run CMakeLists.txt in it
if (NOT TARGET vecIntConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
endif()

# Add folders to build and run CMakeLists.txt in it
if (NOT TARGET matConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/matConverter lib/matConverter)
endif()

if (NOT TARGET mapUpdateConverter)
    add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/mapUpdateConverter lib/mapUpdateConverter)
endif()

file(GLOB SOURCES *.cxx)
file(GLOB HEADERS *.hpp)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCES} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/types/twbTracking.pb.cc)

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
    #${OpenCV_LIBS}
    #   OPENCV
    opencv_imgproc
    opencv_calib3d
    opencv_contrib
    opencv_core
    opencv_features2d
    opencv_flann
    opencv_gpu
    opencv_highgui
    opencv_legacy
    opencv_ml
    opencv_nonfree
    opencv_objdetect
    opencv_ocl
    opencv_photo
    opencv_stitching
    opencv_superres
    # Boost
    boost_regex
    boost_date_time
    boost_program_options
    boost_system
    boost_thread
    # -- Own libraries
    vecIntConverter
    matConverter
    mapUpdateConverter
    ${PROTO_LIBRARIES}

)

else("$ENV{ARCH}" MATCHES arm)
target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    ${PROTO_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # Boost
    ${Boost_LIBRARIES}
    # -- Own libraries
    vecIntConverter
    matConverter
    mapUpdateConverter

)
endif("$ENV{ARCH}" MATCHES arm)

# Get rid of depricated warnings

SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2 -std=c++0x"
)
