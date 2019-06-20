cmake_minimum_required(VERSION 2.6)

# Set the project name to the name of the folder
include("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Check for build types and set the tty-outputs accordingly
include("$ENV{MUROX_CMAKE_MODULES}/chkBuildTypes.cmake")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)

if("$ENV{ARCH}" MATCHES arm)
    pkg_check_modules(PROTOBUF protobuf)
    pkg_check_modules(RSB rsb0.11)
    pkg_check_modules(RSC rsc0.11)
    pkg_check_modules(RST rstsandbox0.11)
else("$ENV{ARCH}" MATCHES arm)
    find_package(RSB REQUIRED)
    find_package(RSC REQUIRED)
    find_package(RST COMPONENTS sandbox)
    find_package(Protobuf REQUIRED)
endif("$ENV{ARCH}" MATCHES arm)

# Get modules
set(BOOST_COMPONENTS program_options thread)
include("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
include("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include path
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# List all custom RST type, they need to be compiled, too
set(CUSTOM_RST_TYPES
    "$ENV{MUROX_INCLUDE_DIRS}/types/TargetPoseEuler.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/PoseEuler.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/RotationEuler.pb.cc"

    "$ENV{MUROX_INCLUDE_DIRS}/types/loc.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/shapes.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/enum.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/pose.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/vertex.pb.cc"
    "$ENV{MUROX_INCLUDE_DIRS}/types/rotation.pb.cc"
)

# Define the source
set(SOURCE main.cxx ${CUSTOM_RST_TYPES} "$ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.cxx")

# Add the vector<int> RSB converter
include("$ENV{MUROX_CMAKE_MODULES}/vecIntConverterSource.cmake")

# add the executable
add_executable("${PROJECT_NAME}" ${SOURCE})

target_link_libraries(
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Add RST flags
add_definitions(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Additional compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations -Wno-write-strings")
# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_definitions(-DDEBUG_MSG_)
