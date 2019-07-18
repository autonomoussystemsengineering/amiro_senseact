cmake_minimum_required (VERSION 2.6)

INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")

FIND_PACKAGE(PkgConfig)
pkg_check_modules(PROTOBUF protobuf)
if ($ENV{ARCH} MATCHES "arm")
  pkg_check_modules(RSB rsb0.13)
  pkg_check_modules(RSC rsc0.13)
  pkg_check_modules(RST rst0.13)
  pkg_check_modules(RSTSANDBOX rstsandbox0.13)
else()
  pkg_check_modules(RSB rsb0.16)
  pkg_check_modules(RSC rsc0.16)
  pkg_check_modules(RST rst0.16)
  pkg_check_modules(RSTSANDBOX rstsandbox0.16)
endif()

# Get modules
SET (BOOST_COMPONENTS regex date_time program_options system thread chrono)
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")

include_directories(
    ${RSB_INCLUDE_DIRS}                                             # RSB
    ${RSC_INCLUDE_DIRS}                                             # RSC
    ${RST_INCLUDE_DIRS}                                             # RST types
    ${PROTOBUF_INCLUDE_DIRS}
    ${RSTSANDBOX_INCLUDE_DIRS}                                      # RST syndbox types
    "$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    ${Boost_INCLUDE_DIR}
    "$ENV{MUROX_INCLUDE_DIRS}/extspread"		            # Programatik spread
    "${CMAKE_CURRENT_BINARY_DIR}"                                   # add the binary tree to the search path for include files
    "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
    ${GXX_INCLUDES}
)

# Define the source
SET (SOURCE main.cxx)



file(GLOB SOURCES *.cxx)
file(GLOB HEADERS *.hpp)

# add the executable
add_executable ("${PROJECT_NAME}" ${SOURCES} ${HEADERS} $ENV{MUROX_INCLUDE_DIRS}/Color.cpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.hpp $ENV{MUROX_INCLUDE_DIRS}/extspread/extspread.cxx)

target_link_libraries (
    "${PROJECT_NAME}"
    ${RSB_LIBRARIES}
    ${RSC_LIBRARIES}
    ${RST_LIBRARIES}
    ${PROTOBUF_INCLUDE_DIRS}
    ${RSTSANDBOX_LIBRARIES}
    ${BOOST_LIRBRARIES}
    boost_thread
    boost_chrono
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS} -Wall -Wno-deprecated-declarations -Wno-write-strings -Wno-unused-function -std=c++11)
