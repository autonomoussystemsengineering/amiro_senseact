string (REGEX MATCH "[^/]+$" CONVERTER_NAME "${CMAKE_CURRENT_BINARY_DIR}")
add_library("${CONVERTER_NAME}" mapUpdateConverter.cpp mapUpdate.hpp)

SET (
 CMAKE_CXX_FLAGS
     "-Wno-deprecated-declarations -O2 -std=c++0x"
)

