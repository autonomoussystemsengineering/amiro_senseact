string (REGEX MATCH "[^/]+$" CONVERTER_NAME "${CMAKE_CURRENT_BINARY_DIR}")
add_library("${CONVERTER_NAME}" matConverter.cpp)
