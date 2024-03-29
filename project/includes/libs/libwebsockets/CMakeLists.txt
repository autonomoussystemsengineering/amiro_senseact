cmake_minimum_required(VERSION 2.6)

project(libwebsockets)

set(PACKAGE "libwebsockets")
set(CPACK_PACKAGE_NAME "${PACKAGE}")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "3")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
set(CPACK_PACKAGE_VENDOR "andy@warmcat.com")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PACKAGE} ${PACKAGE_VERSION}")
set(SOVERSION "4.0.0")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(VERSION "${CPACK_PACKAGE_VERSION}")

set(LWS_LIBRARY_VERSION ${CPACK_PACKAGE_VERSION})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

message(STATUS "CMAKE_TOOLCHAIN_FILE='${CMAKE_TOOLCHAIN_FILE}'")

# Try to find the current Git hash.
find_package(Git)
if(GIT_EXECUTABLE)
	execute_process(
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMAND "${GIT_EXECUTABLE}" log -n 1 --pretty=%h
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(LWS_BUILD_HASH ${GIT_HASH})
    message("Git commit hash: ${LWS_BUILD_HASH}")
endif()

option(LWS_WITH_SSL "Include SSL support (default OpenSSL, CyaSSL if LWS_USE_CYASSL is set)" OFF)
option(LWS_SSL_CLIENT_USE_OS_CA_CERTS "SSL support should make use of OS installed CA root certs" ON)
option(LWS_USE_EXTERNAL_ZLIB "Search the system for ZLib instead of using the included one (on Windows)" OFF)
option(LWS_USE_CYASSL "Use CyaSSL replacement for OpenSSL. When settings this, you also need to specify LWS_CYASSL_LIB and LWS_CYASSL_INCLUDE_DIRS" OFF)
option(LWS_WITHOUT_BUILTIN_GETIFADDRS "Don't use BSD getifaddrs implementation from libwebsockets if it is missing (this will result in a compilation error) ... Default is your libc provides it. On some systems such as uclibc it doesn't exist." OFF)
option(LWS_WITHOUT_CLIENT "Don't build the client part of the library" ON)
option(LWS_WITHOUT_SERVER "Don't build the server part of the library" OFF)
option(LWS_LINK_TESTAPPS_DYNAMIC "Link the test apps to the shared version of the library. Default is to link statically" ON)
option(LWS_WITHOUT_TESTAPPS "Don't build the libwebsocket-test-apps" ON)
option(LWS_WITHOUT_TEST_SERVER "Don't build the test server" ON)
option(LWS_WITHOUT_TEST_SERVER_EXTPOLL "Don't build the test server version that uses external poll" ON)
option(LWS_WITHOUT_TEST_PING "Don't build the ping test application" ON)
option(LWS_WITHOUT_TEST_CLIENT "Don't build the client test application" ON)
option(LWS_WITHOUT_TEST_FRAGGLE "Don't build the ping test application" ON)
option(LWS_WITHOUT_DEBUG "Don't compile debug related code" OFF)
option(LWS_WITHOUT_EXTENSIONS "Don't compile with extensions" ON)
option(LWS_WITH_LATENCY "Build latency measuring code into the library" OFF)
option(LWS_WITHOUT_DAEMONIZE "Don't build the daemonization api" OFF)
option(LWS_WITH_LIBEV "Compile with support for libev" OFF)
option(LWS_IPV6 "Compile with support for ipv6" OFF)
option(LWS_WITH_HTTP2 "Compile with support for http2" OFF)

# Allow the user to override installation directories.
set(LWS_INSTALL_LIB_DIR       lib CACHE PATH "Installation directory for libraries")
set(LWS_INSTALL_BIN_DIR       bin CACHE PATH "Installation directory for executables")
set(LWS_INSTALL_INCLUDE_DIR   include CACHE PATH "Installation directory for header files")
set(LWS_INSTALL_EXAMPLES_DIR  bin CACHE PATH "Installation directory for example files")

if (LWS_WITHOUT_CLIENT AND LWS_WITHOUT_SERVER)
	message(FATAL_ERROR "Makes no sense to compile without both client or server.")
endif()

# The base dir where the test-apps look for the SSL certs.
set(LWS_OPENSSL_CLIENT_CERTS ../share CACHE PATH "Server SSL certificate directory")
if (WIN32)
	set(LWS_OPENSSL_CLIENT_CERTS . CACHE PATH "Client SSL certificate directory")

	if (LWS_IPV6)
		set(LWS_IPV6 OFF)
		message(WARNING "IPv6 does currently not work on Windows!")
	endif()
else()
	set(LWS_OPENSSL_CLIENT_CERTS /etc/pki/tls/certs/ CACHE PATH "Client SSL certificate directory")
endif()

set(LWS_CYASSL_LIB CACHE PATH "Path to the CyaSSL library")
set(LWS_CYASSL_INCLUDE_DIRS CACHE PATH "Path to the CyaSSL include directory")

if (LWS_USE_CYASSL)
	if ("${LWS_CYASSL_LIB}" STREQUAL "" OR "${LWS_CYASSL_INCLUDE_DIRS}" STREQUAL "")
		message(FATAL_ERROR "You must set LWS_CYASSL_LIB and LWS_CYASSL_INCLUDE_DIRS when LWS_USE_CYASSL is turned on")
	endif()
	set(USE_CYASSL 1)
endif()

if (LWS_WITHOUT_EXTENSIONS)
	set(LWS_NO_EXTENSIONS 1)
endif()

if (LWS_WITH_SSL)
	set(LWS_OPENSSL_SUPPORT 1)
endif()

if (LWS_SSL_CLIENT_USE_OS_CA_CERTS)
	set(LWS_SSL_CLIENT_USE_OS_CA_CERTS 1)
endif()

if (LWS_WITH_LATENCY)
	set(LWS_LATENCY 1)
endif()

if (LWS_WITHOUT_DAEMONIZE OR WIN32)
	set(LWS_NO_DAEMONIZE 1)
endif()

if (LWS_WITHOUT_SERVER)
	set(LWS_NO_SERVER 1)
endif()

if (LWS_WITHOUT_CLIENT)
	set(LWS_NO_CLIENT 1)
endif()

if (LWS_WITHOUT_DEBUG)
	set(_DEBUG 0)
else()
	set(_DEBUG 1)
endif()

if (LWS_WITH_LIBEV)
	set(LWS_USE_LIBEV 1)
	set(LWS_NO_EXTERNAL_POLL 1)
endif()

if (LWS_IPV6)
	set(LWS_USE_IPV6 1)
endif()

if (LWS_WITH_HTTP2)
	set(LWS_USE_HTTP2 1)
endif()

if (MINGW)
	set(LWS_MINGW_SUPPORT 1)
endif()

include_directories("${PROJECT_BINARY_DIR}")

include(CheckCSourceCompiles)

# Check for different inline keyword versions.
foreach(KEYWORD "inline" "__inline__" "__inline")
	set(CMAKE_REQUIRED_DEFINITIONS "-DKEYWORD=${KEYWORD}")
	CHECK_C_SOURCE_COMPILES(
		"
		#include <stdio.h>
		KEYWORD void a() {}
		int main(int argc, char **argv) { a(); return 0; }
		" HAVE_${KEYWORD})
endforeach()

if (NOT HAVE_inline)
	if (HAVE___inline__)
		set(inline __inline__)
	elseif(HAVE___inline)
		set(inline __inline)
	endif()
endif()

# Put the libaries and binaries that get built into directories at the
# top of the build tree rather than in hard-to-find leaf directories. 
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# Put absolute path of dynamic libraries into the object code. Some
# architectures, notably Mac OS X, need this.
SET(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${LWS_INSTALL_LIB_DIR}${LIB_SUFFIX}")

# So we can include the CMake generated config file only when
# building with CMAKE.
add_definitions(-DCMAKE_BUILD)

include(CheckFunctionExists)
include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckLibraryExists)
include(CheckTypeSize)

CHECK_FUNCTION_EXISTS(bzero HAVE_BZERO)
CHECK_FUNCTION_EXISTS(fork HAVE_FORK)
CHECK_FUNCTION_EXISTS(getenv HAVE_GETENV)
CHECK_FUNCTION_EXISTS(malloc HAVE_MALLOC)
CHECK_FUNCTION_EXISTS(memset HAVE_MEMSET)
CHECK_FUNCTION_EXISTS(realloc HAVE_REALLOC)
CHECK_FUNCTION_EXISTS(socket HAVE_SOCKET)
CHECK_FUNCTION_EXISTS(strerror HAVE_STRERROR)
CHECK_FUNCTION_EXISTS(vfork HAVE_VFORK)
CHECK_FUNCTION_EXISTS(getifaddrs HAVE_GETIFADDRS)

if (NOT HAVE_GETIFADDRS)
	if (LWS_WITHOUT_BUILTIN_GETIFADDRS)
		message(FATAL_ERROR "No getifaddrs was found on the system. Turn off the LWS_WITHOUT_BUILTIN_GETIFADDRS compile option to use the supplied BSD version.")
	endif()

	set(LWS_BUILTIN_GETIFADDRS 1)
endif()

CHECK_INCLUDE_FILE(dlfcn.h HAVE_DLFCN_H)
CHECK_INCLUDE_FILE(fcntl.h HAVE_FCNTL_H)
CHECK_INCLUDE_FILE(in6addr.h HAVE_IN6ADDR_H)
CHECK_INCLUDE_FILE(inttypes.h HAVE_INTTYPES_H)
CHECK_INCLUDE_FILE(memory.h HAVE_MEMORY_H)
CHECK_INCLUDE_FILE(netinet/in.h HAVE_NETINET_IN_H)
CHECK_INCLUDE_FILE(stdint.h HAVE_STDINT_H)
CHECK_INCLUDE_FILE(stdlib.h HAVE_STDLIB_H)
CHECK_INCLUDE_FILE(strings.h HAVE_STRINGS_H)
CHECK_INCLUDE_FILE(string.h HAVE_STRING_H)
CHECK_INCLUDE_FILE(sys/prctl.h HAVE_SYS_PRCTL_H)
CHECK_INCLUDE_FILE(sys/socket.h HAVE_SYS_SOCKET_H)
CHECK_INCLUDE_FILE(sys/stat.h HAVE_SYS_STAT_H)
CHECK_INCLUDE_FILE(sys/types.h HAVE_SYS_TYPES_H)
CHECK_INCLUDE_FILE(unistd.h HAVE_UNISTD_H)
CHECK_INCLUDE_FILE(vfork.h HAVE_VFORK_H)
CHECK_INCLUDE_FILE(zlib.h HAVE_ZLIB_H)

# TODO: These can be tested if they actually work also...
set(HAVE_WORKING_FORK HAVE_FORK)
set(HAVE_WORKING_VFORK HAVE_VFORK)

CHECK_INCLUDE_FILES("stdlib.h;stdarg.h;string.h;float.h" STDC_HEADERS)

CHECK_TYPE_SIZE(pid_t PID_T_SIZE)
CHECK_TYPE_SIZE(size_t SIZE_T_SIZE)

if (NOT PID_T_SIZE)
	set(pid_t int)
endif()

if (NOT SIZE_T_SIZE)
	set(size_t "unsigned int")
endif()

if (NOT HAVE_MALLOC)
	set(malloc rpl_malloc)
endif()

if (NOT HAVE_REALLOC)
	set(realloc rpl_realloc)
endif()

# Generate the config.h that includes all the compilation settings.
configure_file(
       "${PROJECT_SOURCE_DIR}/config.h.cmake"
       "${PROJECT_BINARY_DIR}/lws_config.h")

if (MSVC)
	# Turn off stupid microsoft security warnings.
	add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
endif(MSVC)

include_directories("${PROJECT_SOURCE_DIR}/lib")

# Group headers and sources.
# Some IDEs use this for nicer file structure.
set(HDR_PRIVATE
	lib/private-libwebsockets.h
	"${PROJECT_BINARY_DIR}/lws_config.h"
	)

set(HDR_PUBLIC	
	"${PROJECT_SOURCE_DIR}/lib/libwebsockets.h"
	)

set(SOURCES
	lib/base64-decode.c
	lib/handshake.c
	lib/libwebsockets.c
	lib/service.c
	lib/pollfd.c
	lib/output.c
	lib/parsers.c
	lib/context.c
	lib/sha-1.c
	)

if (NOT LWS_WITHOUT_CLIENT)
	list(APPEND SOURCES
		lib/client.c
		lib/client-handshake.c
		lib/client-parser.c
		)
endif()

if (LWS_WITH_SSL)
	list(APPEND SOURCES
		lib/ssl.c
		)
endif()

if (LWS_WITH_HTTP2)
	list(APPEND SOURCES
		lib/ssl-http2.c
		)
endif()
# select the active platform files

if (WIN32)
	list(APPEND SOURCES
		lib/lws-plat-win.c
		)
else()
	list(APPEND SOURCES
		lib/lws-plat-unix.c
		)
endif()

if (NOT LWS_WITHOUT_SERVER)
	list(APPEND SOURCES
		lib/server.c
		lib/server-handshake.c
		)
endif()

if (NOT LWS_WITHOUT_EXTENSIONS)
	list(APPEND HDR_PRIVATE
		lib/extension-deflate-frame.h
		lib/extension-deflate-stream.h
		)

	list(APPEND SOURCES
		lib/extension.c
		lib/extension-deflate-frame.c
		lib/extension-deflate-stream.c
		)
endif()

if (LWS_WITH_LIBEV)
	list(APPEND SOURCES
		lib/libev.c
		)
endif(LWS_WITH_LIBEV)

# Add helper files for Windows.
if (WIN32)
	set(WIN32_HELPERS_PATH win32port/win32helpers)
	include_directories(${WIN32_HELPERS_PATH})
else(WIN32)
	# Unix.
	if (NOT LWS_WITHOUT_DAEMONIZE)
		list(APPEND SOURCES
			lib/daemonize.c
			)
	endif()
endif(WIN32)

if (UNIX)
	if (NOT HAVE_GETIFADDRS)
		list(APPEND HDR_PRIVATE lib/getifaddrs.h)
		list(APPEND SOURCES lib/getifaddrs.c)
	endif()
endif(UNIX)


if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    if (UNIX)
	set( CMAKE_C_FLAGS "-Wall -Werror -O4 -fvisibility=hidden ${CMAKE_C_FLAGS}" )
    else(UNIX)
	set( CMAKE_C_FLAGS "-Wall -O4 -fvisibility=hidden ${CMAKE_C_FLAGS}" )
    endif(UNIX)
endif ()

source_group("Headers Private"  FILES ${HDR_PRIVATE})
source_group("Headers Public"   FILES ${HDR_PUBLIC})
source_group("Sources"          FILES ${SOURCES})

#
# Create the lib.
#
add_library(websockets STATIC
			${HDR_PRIVATE}
			${HDR_PUBLIC}
			${SOURCES})
add_library(websockets_shared SHARED
			${HDR_PRIVATE}
			${HDR_PUBLIC}
			${SOURCES})

if (WIN32)
	# On Windows libs have the same file ending (.lib)
	# for both static and shared libraries, so we
	# need a unique name for the static one.
	set_target_properties(websockets 
		PROPERTIES
		OUTPUT_NAME websockets_static)

	# Compile as DLL (export function declarations)
	set_property(
		TARGET websockets_shared
		PROPERTY COMPILE_DEFINITIONS 
		LWS_DLL
		LWS_INTERNAL
		)
endif(WIN32)

# We want the shared lib to be named "libwebsockets"
# not "libwebsocket_shared".
set_target_properties(websockets_shared
		PROPERTIES 
		OUTPUT_NAME websockets)

# Set the so version of the lib.
# Equivalent to LDFLAGS=-version-info x:x:x
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	foreach(lib websockets websockets_shared)
		set_target_properties(${lib} 
			PROPERTIES
			SOVERSION ${SOVERSION})
	endforeach()
endif()

set(LIB_LIST)

#
# Find libraries.
#

#
# ZLIB (Only needed for deflate extensions).
#
if (NOT LWS_WITHOUT_EXTENSIONS)
	if (WIN32 AND NOT LWS_USE_EXTERNAL_ZLIB)
		message("Using included Zlib version")

		# Compile ZLib if needed.
		set(WIN32_ZLIB_PATH "win32port/zlib")
		set(ZLIB_SRCS
			${WIN32_ZLIB_PATH}/adler32.c
			${WIN32_ZLIB_PATH}/compress.c
			${WIN32_ZLIB_PATH}/crc32.c
			${WIN32_ZLIB_PATH}/deflate.c
			${WIN32_ZLIB_PATH}/gzclose.c
			${WIN32_ZLIB_PATH}/gzio.c
			${WIN32_ZLIB_PATH}/gzlib.c
			${WIN32_ZLIB_PATH}/gzread.c
			${WIN32_ZLIB_PATH}/gzwrite.c
			${WIN32_ZLIB_PATH}/infback.c
			${WIN32_ZLIB_PATH}/inffast.c
			${WIN32_ZLIB_PATH}/inflate.c
			${WIN32_ZLIB_PATH}/inftrees.c
			${WIN32_ZLIB_PATH}/trees.c
			${WIN32_ZLIB_PATH}/uncompr.c
			${WIN32_ZLIB_PATH}/zutil.c
		)

		# Create the library.
		add_library(ZLIB STATIC ${ZLIB_SRCS})

		# Set the same variables as find_package would.
		set(ZLIB_INCLUDE_DIRS ${WIN32_ZLIB_PATH})
		get_property(ZLIB_LIBRARIES TARGET ZLIB PROPERTY LOCATION)
		set(ZLIB_FOUND 1)
	else()
		find_package(ZLIB REQUIRED)
	endif()

	# Make sure ZLib is compiled before the libs.
	foreach (lib websockets websockets_shared)
		add_dependencies(${lib} ZLIB)
	endforeach()

	message("ZLib include dirs: ${ZLIB_INCLUDE_DIRS}")
	message("ZLib libraries: ${ZLIB_LIBRARIES}")
	include_directories(${ZLIB_INCLUDE_DIRS})
	list(APPEND LIB_LIST ${ZLIB_LIBRARIES})
endif(NOT LWS_WITHOUT_EXTENSIONS)

#
# OpenSSL
#
if (LWS_WITH_SSL)
	message("Compiling with SSL support")

	if (LWS_USE_CYASSL)
		# Use CyaSSL as OpenSSL replacement.
		# TODO: Add a find_package command for this also.
		message("CyaSSL include dir: ${LWS_CYASSL_INCLUDE_DIRS}")
		message("CyaSSL libraries: ${LWS_CYASSL_LIB}")

		# Additional to the root directory we need to include
		# the cyassl/ subdirectory which contains the OpenSSL
		# compatability layer headers.
		foreach(inc ${LWS_CYASSL_INCLUDE_DIRS})
			include_directories("${inc}" "${inc}/cyassl")
		endforeach()

		list(APPEND LIB_LIST "${LWS_CYASSL_LIB}")
	else()
		# TODO: Add support for STATIC also.
		find_package(OpenSSL REQUIRED)

		message("OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")
		message("OpenSSL libraries: ${OPENSSL_LIBRARIES}")

		include_directories("${OPENSSL_INCLUDE_DIR}")
		list(APPEND LIB_LIST ${OPENSSL_LIBRARIES})

		# Link against dynamic linking functions.
		# (Don't link directly to libdl since it is not needed on all platforms, it's now a part of libc).
		list(APPEND LIB_LIST ${CMAKE_DL_LIBS})
	endif()
endif(LWS_WITH_SSL)

if (LWS_WITH_LIBEV)
	list(APPEND LIB_LIST "ev")
endif(LWS_WITH_LIBEV)

#
# Platform specific libs.
#
if (WINCE)
	list(APPEND LIB_LIST ws2.lib)
elseif (WIN32)
	list(APPEND LIB_LIST ws2_32.lib)
endif()

if (UNIX)
	list(APPEND LIB_LIST m)
endif()

# Setup the linking for all libs.
foreach (lib websockets websockets_shared)
	target_link_libraries(${lib} ${LIB_LIST})
endforeach()

#
# Test applications
#
set(TEST_APP_LIST)
if (NOT LWS_WITHOUT_TESTAPPS)
	#
	# Helper function for adding a test app.
	#
	macro(create_test_app TEST_NAME MAIN_SRC)
		
		set(TEST_SRCS ${MAIN_SRC})
		set(TEST_HDR)

		if (WIN32)
			list(APPEND TEST_SRCS 
				${WIN32_HELPERS_PATH}/getopt.c
				${WIN32_HELPERS_PATH}/getopt_long.c
				${WIN32_HELPERS_PATH}/gettimeofday.c
			)

			list(APPEND TEST_HDR 
				${WIN32_HELPERS_PATH}/getopt.h
				${WIN32_HELPERS_PATH}/gettimeofday.h
			)
		endif(WIN32)

		source_group("Headers Private"   FILES ${TEST_HDR})
		source_group("Sources"   FILES ${TEST_SRCS})
		add_executable(${TEST_NAME} ${TEST_SRCS} ${TEST_HDR})
		
		if (LWS_LINK_TESTAPPS_DYNAMIC)
			target_link_libraries(${TEST_NAME} websockets_shared)
			add_dependencies(${TEST_NAME} websockets_shared)
		else(LWS_LINK_TESTAPPS_DYNAMIC)
			target_link_libraries(${TEST_NAME} websockets)
			add_dependencies(${TEST_NAME} websockets)
		endif(LWS_LINK_TESTAPPS_DYNAMIC)

		# Set test app specific defines.
		set_property(TARGET ${TEST_NAME}
					PROPERTY COMPILE_DEFINITIONS 
						INSTALL_DATADIR="${CMAKE_INSTALL_PREFIX}/share"
					)

		# Prefix the binary names with libwebsockets.
		set_target_properties(${TEST_NAME} 
			PROPERTIES
			OUTPUT_NAME libwebsockets-${TEST_NAME})

		# Add to the list of tests.
		list(APPEND TEST_APP_LIST ${TEST_NAME})
	endmacro()

	if (LWS_WITH_SSL AND NOT LWS_USE_CYASSL)
		message("Searching for OpenSSL executable and dlls")
		find_package(OpenSSLbins)
		message("OpenSSL executable: ${OPENSSL_EXECUTABLE}")
	endif()

	if (NOT LWS_WITHOUT_SERVER)
		#
		# test-server
		#
		if (NOT LWS_WITHOUT_TEST_SERVER)
			create_test_app(test-server "test-server/test-server.c")
		endif()

		#
		# test-server-extpoll
		#
		if (NOT LWS_WITHOUT_TEST_SERVER_EXTPOLL)
			create_test_app(test-server-extpoll "test-server/test-server.c")
			# Set defines for this executable only.
			set_property(
				TARGET test-server-extpoll
				PROPERTY COMPILE_DEFINITIONS 
					EXTERNAL_POLL 
					INSTALL_DATADIR="${CMAKE_INSTALL_PREFIX}/share"
				)

			# We need to link against winsock code.
			if (WIN32)
				target_link_libraries(test-server-extpoll ws2_32.lib)
			endif(WIN32)
		endif()

		# Data files for running the test server.
		set(TEST_SERVER_DATA
			"${PROJECT_SOURCE_DIR}/test-server/favicon.ico"
			"${PROJECT_SOURCE_DIR}/test-server/leaf.jpg"
			"${PROJECT_SOURCE_DIR}/test-server/libwebsockets.org-logo.png"
			"${PROJECT_SOURCE_DIR}/test-server/test.html")

		# Generate self-signed SSL certs for the test-server.
		if (LWS_WITH_SSL AND OPENSSL_EXECUTABLE AND NOT LWS_WITHOUT_TEST_SERVER)
			message("Generating SSL Certificates for the test-server...")

			set(TEST_SERVER_SSL_KEY "${PROJECT_BINARY_DIR}/libwebsockets-test-server.key.pem")
			set(TEST_SERVER_SSL_CERT "${PROJECT_BINARY_DIR}/libwebsockets-test-server.pem")

			if (WIN32)
				file(WRITE "${PROJECT_BINARY_DIR}/openssl_input.txt"
					"GB\n"
					"Erewhon\n"
					"All around\n"
					"libwebsockets-test\n"
					"localhost\n"
					"none@invalid.org\n\n"
					)

				# The "type" command is a bit picky with paths.
				file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/openssl_input.txt" OPENSSL_INPUT_WIN_PATH)
				message("OPENSSL_INPUT_WIN_PATH = ${OPENSSL_INPUT_WIN_PATH}")
				message("cmd = \"${OPENSSL_EXECUTABLE}\" req -new -newkey rsa:1024 -days 10000 -nodes -x509 -keyout \"${TEST_SERVER_SSL_KEY}\" -out \"${TEST_SERVER_SSL_CERT}\"")

				execute_process(
					COMMAND cmd /c type "${OPENSSL_INPUT_WIN_PATH}"
					COMMAND "${OPENSSL_EXECUTABLE}" req -new -newkey rsa:1024 -days 10000 -nodes -x509 -keyout "${TEST_SERVER_SSL_KEY}" -out "${TEST_SERVER_SSL_CERT}"
					RESULT_VARIABLE OPENSSL_RETURN_CODE)

				message("\n")

				if (OPENSSL_RETURN_CODE)
					message(WARNING "!!! Failed to generate SSL certificate for Test Server using cmd.exe !!!:\nOpenSSL return code = ${OPENSSL_RETURN_CODE}")
				else()
					message("SUCCSESFULLY generated SSL certificate")
				endif()
			else()
				# Unix.
				execute_process(
					COMMAND printf "GB\\nErewhon\\nAll around\\nlibwebsockets-test\\n\\nlocalhost\\nnone@invalid.org\\n"
					COMMAND "${OPENSSL_EXECUTABLE}"
						req -new -newkey rsa:1024 -days 10000 -nodes -x509 -keyout "${TEST_SERVER_SSL_KEY}" -out "${TEST_SERVER_SSL_CERT}"
					RESULT_VARIABLE OPENSSL_RETURN_CODE)

				if (OPENSSL_RETURN_CODE)
					message(WARNING "!!! Failed to generate SSL certificate for Test Server!!!:\nOpenSSL return code = ${OPENSSL_RETURN_CODE}")
				else()
					message("SUCCSESFULLY generated SSL certificate")
				endif()
			endif()

			list(APPEND TEST_SERVER_DATA 
				"${TEST_SERVER_SSL_KEY}"
				"${TEST_SERVER_SSL_CERT}")
		endif()

		add_custom_command(TARGET test-server
						POST_BUILD 
						COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:test-server>/../share/libwebsockets-test-server")

		# Copy the file needed to run the server so that the test apps can
		# reach them from their default output location
		foreach (TEST_FILE ${TEST_SERVER_DATA})
			if (EXISTS ${TEST_FILE})
				add_custom_command(TARGET test-server
							POST_BUILD 
							COMMAND "${CMAKE_COMMAND}" -E copy "${TEST_FILE}" "$<TARGET_FILE_DIR:test-server>/../share/libwebsockets-test-server" VERBATIM)
			endif()
		endforeach()
	endif(NOT LWS_WITHOUT_SERVER)

	if (NOT LWS_WITHOUT_CLIENT)
		#
		# test-client
		#
		if (NOT LWS_WITHOUT_TEST_CLIENT)
			create_test_app(test-client "test-server/test-client.c")
		endif()

		#
		# test-fraggle
		#
		if (NOT LWS_WITHOUT_TEST_FRAGGLE)
			create_test_app(test-fraggle "test-server/test-fraggle.c")
		endif()

		#
		# test-ping
		#
		if (NOT LWS_WITHOUT_TEST_PING)
			create_test_app(test-ping "test-server/test-ping.c")
		endif()
		#
		# test-echo
		#
		if (NOT WITHOUT_TEST_ECHO)
			create_test_app(test-echo "test-server/test-echo.c")
		endif()

	endif(NOT LWS_WITHOUT_CLIENT)

	#
	# Copy OpenSSL dlls to the output directory on Windows.
	# (Otherwise we'll get an error when trying to run)
	#
	if (WIN32 AND LWS_WITH_SSL AND NOT LWS_USE_CYASSL)
		if(OPENSSL_BIN_FOUND)
			message("OpenSSL dlls found:")
			message("  Libeay: ${LIBEAY_BIN}")
			message("  SSLeay: ${SSLEAY_BIN}")

			foreach(TARGET_BIN ${TEST_APP_LIST})			
				add_custom_command(TARGET ${TARGET_BIN}
					POST_BUILD 
					COMMAND "${CMAKE_COMMAND}" -E copy "${LIBEAY_BIN}" "$<TARGET_FILE_DIR:${TARGET_BIN}>" VERBATIM)
					
				add_custom_command(TARGET ${TARGET_BIN}
					POST_BUILD 
					COMMAND "${CMAKE_COMMAND}" -E copy "${SSLEAY_BIN}" "$<TARGET_FILE_DIR:${TARGET_BIN}>" VERBATIM)
			endforeach()
		endif()
	endif()
endif(NOT LWS_WITHOUT_TESTAPPS)

if (UNIX)
	# Generate documentation.
	# TODO: Fix this on Windows.
	message("Generating API documentation")
	file(GLOB C_FILES "${PROJECT_SOURCE_DIR}/lib/*.c")
	execute_process(COMMAND "${CMAKE_COMMAND}" -E make_directory "${PROJECT_BINARY_DIR}/doc/")

	execute_process(
		COMMAND "${PROJECT_SOURCE_DIR}/scripts/kernel-doc" -html ${C_FILES} ${HDR_PUBLIC}
		OUTPUT_FILE "${PROJECT_BINARY_DIR}/doc/libwebsockets-api-doc.html"
		ERROR_QUIET)

	execute_process(
		COMMAND "${PROJECT_SOURCE_DIR}/scripts/kernel-doc" -text ${C_FILES} ${HDR_PUBLIC}
		OUTPUT_FILE "${PROJECT_BINARY_DIR}/doc/libwebsockets-api-doc.txt"
		ERROR_QUIET)

# Generate and install pkgconfig.
# (This is not indented, because the tabs will be part of the output)
file(WRITE "${PROJECT_BINARY_DIR}/libwebsockets.pc"
"prefix=\"${CMAKE_INSTALL_PREFIX}\"
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib${LIB_SUFFIX}
includedir=\${prefix}/include

Name: libwebsockets
Description: Websockets server and client library
Version: ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}

Libs: -L\${libdir} -lwebsockets
Cflags: -I\${includedir}"
)

	install(FILES "${PROJECT_BINARY_DIR}/libwebsockets.pc"
		DESTINATION lib${LIB_SUFFIX}/pkgconfig)
endif(UNIX)

#
# Installation preparations.
#

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/libwebsockets)
endif()

set(LWS_INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Export targets (This is used for other CMake projects to easily find the libraries and include files).
export(TARGETS websockets websockets_shared
        FILE "${PROJECT_BINARY_DIR}/LibwebsocketsTargets.cmake")
export(PACKAGE libwebsockets)

# Generate the config file for the build-tree.
set(LWS__INCLUDE_DIRS 
    "${PROJECT_SOURCE_DIR}/lib"
    "${PROJECT_BINARY_DIR}")
set(LIBWEBSOCKETS_INCLUDE_DIRS ${LWS__INCLUDE_DIRS} CACHE PATH "Libwebsockets include directories")
configure_file(${PROJECT_SOURCE_DIR}/cmake/LibwebsocketsConfig.cmake.in
                ${PROJECT_BINARY_DIR}/LibwebsocketsConfig.cmake 
                @ONLY)

# Generate the config file for the installation tree.
get_filename_component(LWS_ABSOLUTE_INSTALL_CMAKE_DIR ${LWS_INSTALL_CMAKE_DIR} ABSOLUTE)
get_filename_component(LWS_ABSOLUTE_INSTALL_INCLUDE_DIR ${LWS_INSTALL_INCLUDE_DIR} ABSOLUTE)
file(RELATIVE_PATH 
    REL_INCLUDE_DIR 
    "${LWS_ABSOLUTE_INSTALL_CMAKE_DIR}"
    "${LWS_ABSOLUTE_INSTALL_INCLUDE_DIR}") # Calculate the relative directory from the cmake dir.

# Note the EVENT_CMAKE_DIR is defined in JanssonConfig.cmake.in, 
# we escape it here so it's evaluated when it is included instead
# so that the include dirs are given relative to where the 
# config file is located.
set(LWS__INCLUDE_DIRS 
    "\${LWS_CMAKE_DIR}/${REL_INCLUDE_DIR}") 
configure_file(${PROJECT_SOURCE_DIR}/cmake/LibwebsocketsConfig.cmake.in
                ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/LibwebsocketsConfig.cmake 
                @ONLY)

# Generate version info for both build-tree and install-tree.
configure_file(${PROJECT_SOURCE_DIR}/cmake/LibwebsocketsConfigVersion.cmake.in
                ${PROJECT_BINARY_DIR}/LibwebsocketsConfigVersion.cmake 
                @ONLY)

set_target_properties(websockets websockets_shared 
					PROPERTIES PUBLIC_HEADER "${HDR_PUBLIC}")

#
# Installation.
#

# Install libs and headers.
install(TARGETS websockets websockets_shared
		EXPORT LibwebsocketsTargets
		LIBRARY DESTINATION "${LWS_INSTALL_LIB_DIR}${LIB_SUFFIX}" COMPONENT libraries
		ARCHIVE DESTINATION "${LWS_INSTALL_LIB_DIR}${LIB_SUFFIX}" COMPONENT libraries
		RUNTIME DESTINATION "${LWS_INSTALL_BIN_DIR}" COMPONENT libraries # Windows DLLs
		PUBLIC_HEADER DESTINATION "${LWS_INSTALL_INCLUDE_DIR}" COMPONENT dev)
set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
set(CPACK_COMPONENT_DEV_DISPLAY_NAME "Development files")

# Install test apps.
if (NOT LWS_WITHOUT_TESTAPPS AND NOT LWS_WITHOUT_CLIENT)
	install(TARGETS test-client ${TEST_APP_LIST}
			RUNTIME DESTINATION ${LWS_INSTALL_EXAMPLES_DIR}
			COMPONENT examples)
	set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Example files")
endif()

# Programs shared files used by the test-server.
if (NOT LWS_WITHOUT_TESTAPPS AND NOT LWS_WITHOUT_SERVER)
	install(FILES ${TEST_SERVER_DATA}
			DESTINATION share/libwebsockets-test-server
			COMPONENT examples)
endif()

# Install exports for the install-tree.
install(EXPORT LibwebsocketsTargets
		DESTINATION "${LWS_INSTALL_CMAKE_DIR}" COMPONENT dev)

# build subdir is not part of sources
set(CPACK_SOURCE_IGNORE_FILES $(CPACK_SOURCE_IGNORE_FILES) ".git" "build" "tgz" "tar.gz")

# Most people are more used to "make dist" compared to "make package_source"
add_custom_target(dist COMMAND "${CMAKE_MAKE_PROGRAM}" package_source)

include(UseRPMTools)
if (RPMTools_FOUND)
	RPMTools_ADD_RPM_TARGETS(libwebsockets libwebsockets.spec)
endif()

message("---------------------------------------------------------------------")
message("  Settings:  (For more help do cmake -LH <srcpath>")
message("---------------------------------------------------------------------")
message(" LWS_WITH_SSL = ${LWS_WITH_SSL}  (SSL Support)")
message(" LWS_SSL_CLIENT_USE_OS_CA_CERTS = ${LWS_SSL_CLIENT_USE_OS_CA_CERTS}")
message(" LWS_USE_CYASSL = ${LWS_USE_CYASSL} (CyaSSL replacement for OpenSSL)")
if (LWS_USE_CYASSL)
	message("   LWS_CYASSL_LIB = ${LWS_CYASSL_LIB}")
	message("   LWS_CYASSL_INCLUDE_DIRS = ${LWS_CYASSL_INCLUDE_DIRS}")
endif()
message(" LWS_WITHOUT_BUILTIN_GETIFADDRS = ${LWS_WITHOUT_BUILTIN_GETIFADDRS}")
message(" LWS_WITHOUT_CLIENT = ${LWS_WITHOUT_CLIENT}")
message(" LWS_WITHOUT_SERVER = ${LWS_WITHOUT_SERVER}")
message(" LWS_LINK_TESTAPPS_DYNAMIC = ${LWS_LINK_TESTAPPS_DYNAMIC}")
message(" LWS_WITHOUT_TESTAPPS = ${LWS_WITHOUT_TESTAPPS}")
message(" LWS_WITHOUT_TEST_SERVER = ${LWS_WITHOUT_TEST_SERVER}")
message(" LWS_WITHOUT_TEST_SERVER_EXTPOLL = ${LWS_WITHOUT_TEST_SERVER_EXTPOLL}")
message(" LWS_WITHOUT_TEST_PING = ${LWS_WITHOUT_TEST_PING}")
message(" LWS_WITHOUT_TEST_CLIENT = ${LWS_WITHOUT_TEST_CLIENT}")
message(" LWS_WITHOUT_TEST_FRAGGLE = ${LWS_WITHOUT_TEST_FRAGGLE}")
message(" LWS_WITHOUT_DEBUG = ${LWS_WITHOUT_DEBUG}")
message(" LWS_WITHOUT_EXTENSIONS = ${LWS_WITHOUT_EXTENSIONS}")
message(" LWS_WITH_LATENCY = ${LWS_WITH_LATENCY}")
message(" LWS_WITHOUT_DAEMONIZE = ${LWS_WITHOUT_DAEMONIZE}")
message(" LWS_USE_LIBEV = ${LWS_USE_LIBEV}")
message(" LWS_IPV6 = ${LWS_IPV6}")
message(" LWS_WITH_HTTP2 = ${LWS_WITH_HTTP2}")
message("---------------------------------------------------------------------")

# These will be available to parent projects including libwebsockets using add_subdirectory()
set(LIBWEBSOCKETS_LIBRARIES websocket websockets_shared CACHE STRING "Libwebsocket libraries")
set(LIBWEBSOCKETS_LIBRARIES_STATIC websocket CACHE STRING "Libwebsocket static library")
set(LIBWEBSOCKETS_LIBRARIES_SHARED websockets_shared CACHE STRING "Libwebsocket shared library")

# This must always be last!
include(CPack)
