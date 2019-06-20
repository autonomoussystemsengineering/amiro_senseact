cmake_minimum_required (VERSION 2.6)

# Set the project name to the name of the folder
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

# Read the modules
find_package(Eigen3 REQUIRED)
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED COMPONENTS sandbox)
find_package(Boost REQUIRED COMPONENTS regex date_time program_options system)
find_package(OpenCV REQUIRED)

# Set the include path
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
  ${RSC_INCLUDE_DIRS}
  ${RST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "$ENV{MUROX_INCLUDE_DIRS}/types"
  "$ENV{MUROX_INCLUDE_DIRS}"
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${EIGEN3_INCLUDE_DIR}
)

# Add converter to build
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/matConverterSource.cmake")

# add the executable
add_executable ("${PROJECT_NAME}" main.cpp ${SOURCE})

LINK_DIRECTORIES (
  ${OpenCV_LIBRARY_DIRS}
)

target_link_libraries (
  "${PROJECT_NAME}"
  ${RSB_LIBRARIES}
  ${RSC_LIBRARIES}
  ${RST_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Set compiler flags as you like
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g -std=c++11" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O2 -std=c++11" ) 
