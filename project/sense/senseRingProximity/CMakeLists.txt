cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Read the pkg-config modules
find_package(PkgConfig)
pkg_check_modules(PROTOBUF protobuf)
pkg_check_modules(OPENCV opencv)
if ($ENV{ARCH} MATCHES "arm")
    pkg_check_modules(RSB rsb0.13)
    pkg_check_modules(RSC rsc0.13)
    pkg_check_modules(RST rstsandbox0.13)
else()
    pkg_check_modules(RSB rsb0.16)
    pkg_check_modules(RSC rsc0.16)
    pkg_check_modules(RST rstsandbox0.16)
endif()

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}
     # RSC                                            
    ${RST_INCLUDE_DIRS}
     # RST
    ${OPENCV_INCLUDE_DIRS}                                          # OpenCV
    ${PROTOBUF_INCLUDE_DIRS}                                        # Protobuffer
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    ${GXX_INCLUDES}
    ${Boost_INCLUDE_DIR}
)

# Add folders to build and run CMakeLists.txt in it
#if (NOT TARGET vecIntConverter)
#   add_subdirectory ($ENV{MUROX_INCLUDE_DIRS}/converter/vecIntConverter lib/vecIntConverter)
#endif()

# Define the source
SET (SOURCE main.cxx)

LINK_DIRECTORIES (
    ${RSB_LIBRARY_DIRS}
    ${RSC_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCE})

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${RST_LIBRARIES}
    ${RSTSANDBOX_LIBRARIES}
    jpeg
    # -- Own libraries
    #vecIntConverter
)

# Additional compiler flags
#ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11 -Wcpp)
ADD_DEFINITIONS (-Wno-deprecated-declarations -O3 -std=c++11)
# Special flags for RST
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

# Set compiler flags as you like
#SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++11" )
#SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++11" )