cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

# Read the pkg-config modules
find_package(PkgConfig)

pkg_check_modules(RSB rsb0.13)
pkg_check_modules(RSC rsc0.13)
# pkg_check_modules(PROTO protobuf)

# Get modules
# find_package(Boost REQUIRED regex date_time program_options system thread)

# Set the include path
include_directories(
    ${RSB_INCLUDE_DIRS}
    ${RSC_INCLUDE_DIRS}
    "${CMAKE_CURRENT_BINARY_DIR}"
    "./mongoose"
)

# add the executable
add_executable ("${PROJECT_NAME}" main.cxx ./mongoose/mongoose.c)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
#     ${RSC_LIBRARIES}
#     ${PROTO_LIBRARIES}
)

# Set compiler flags as you like
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11 -funsafe-math-optimizations" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -funsafe-math-optimizations" )
