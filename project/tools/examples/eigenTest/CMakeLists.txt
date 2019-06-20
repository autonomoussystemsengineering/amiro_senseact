
# Set the project name to the name of the folder
string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message (STATUS "Set PROJECT_NAME to ${PROJECT_NAME}")

project ("${PROJECT_NAME}")

find_package( PkgConfig )
pkg_check_modules(OPENCV OpenCV)
pkg_check_modules(EIGEN eigen3)

link_directories (
    "$ENV{SDKTARGETSYSROOT}/usr/lib"
)

include_directories(
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN_INCLUDE_DIRS}
  "$ENV{SDKTARGETSYSROOT}/usr/include"                            # System includes
  "$ENV{SDKTARGETSYSROOT}/usr/include/c++"                        # System c++ includes
  "$ENV{SDKTARGETSYSROOT}/usr/include/c++/arm-poky-linux-gnueabi" # IO streams)
)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} opencv_core)


SET (
    CMAKE_CXX_FLAGS
    "-Wno-deprecated-declarations -O2 -std=c++0x" 
)
