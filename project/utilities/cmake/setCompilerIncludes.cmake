# Get the compiler specific includes and store them in GXX_INCLUDES in the parent scope

set (test "${CMAKE_CXX_COMPILER}${CMAKE_CXX_COMPILER_ARG1}")

execute_process(COMMAND printCompilerIncludes OUTPUT_VARIABLE GXX_INCLUDES)

# MESSAGE(STATUS "${GXX_INCLUDES}")
message (STATUS "Set GXX_INCLUDES to ${GXX_INCLUDES}")