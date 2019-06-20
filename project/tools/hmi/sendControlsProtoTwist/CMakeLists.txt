cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED COMPONENTS sandbox)
find_package(Protobuf REQUIRED)

# Set the include pathe
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
include_directories(include
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RSC
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

# add the executable
ADD_EXECUTABLE ("${PROJECT_NAME}" ${SOURCE})

# Link the executable to the library.
link_directories (
    "/usr/lib"                             # For standard libraries
    "/usr/local/lib/"                      # For OpenCV libraries
    "/usr/lib/x86_64-linux-gnu/"           # For boost libraries
)

TARGET_LINK_LIBRARIES (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
)

# Additional compiler flags
ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})