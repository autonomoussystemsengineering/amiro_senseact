// Usage:
//      #define INFO_MSG_
//      #define DEBUG_MSG_
//      #define SUCCESS_MSG_
//      #define WARNING_MSG_
//      #define ERROR_MSG_
//      #include <MSG.h>
//      int main (int argc, const char **argv){
//      	int iVar = 42;
//      	INFO_MSG("This is some variable " << iVar)
//      	DEBUG_MSG("This is some variable " << iVar)
//      	SUCCESS_MSG("This is some variable " << iVar)
//      	WARNING_MSG("This is some variable " << iVar)
//      	ERROR_MSG("This is some variable " << iVar)
//      	return EXIT_SUCCESS;
//      }

// Prints out:
//      /foo/bar INFO: This is some variable 42
//      /foo/bar DEBUG: This is some variable 42
//      /foo/bar SUCCESS: This is some variable 42
//      /foo/bar WARNING: This is some variable 42
//      /foo/bar ERROR: This is some variable 42

// Color codes
// Black       0;30     Dark Gray     1;30
// Blue        0;34     Light Blue    1;34
// Green       0;32     Light Green   1;32
// Cyan        0;36     Light Cyan    1;36
// Red         0;31     Light Red     1;31
// Purple      0;35     Light Purple  1;35
// Brown       0;33     Yellow        1;33
// Light Gray  0;37     White         1;37

#ifndef MSG_H
#define MSG_H

#include <iostream>
#include <unistd.h>

// Declare the external __progname variable, so that we know the name of the programm
// Ref.: http://www.qnx.com/developers/docs/6.5.0/index.jsp?topic=%2Fcom.qnx.doc.neutrino_lib_ref%2Fp%2F__progname.html
extern char * __progname;

// Define standard messages
#ifdef INFO_MSG_
#define INFO_MSG(str) { std::cout <<  __progname << " " << getpid() << " \033[1;37mINFO\033[0m: " << str << std::endl; }
#else
#define INFO_MSG(str) {}
#endif

// Define debug messages
#ifdef DEBUG_MSG_
#define DEBUG_MSG(str) { std::cout << __progname << " " << getpid() << " \033[1;35mDEBUG\033[0m: " << str << std::endl; }
#else
#define DEBUG_MSG(str) {}
#endif

// Define success messages
#ifdef SUCCESS_MSG_
#define SUCCESS_MSG(str) { std::cout << __progname << " " << getpid() << " \033[1;32mSUCCESS\033[0m: " << str << std::endl; }
#else
#define SUCCESS_MSG(str) {}
#endif

// Define warning messages
#ifdef WARNING_MSG_
#define WARNING_MSG(str) { std::cout << __progname << " " << getpid() << " \033[1;33mWARNING\033[0m: " << str << std::endl; }
#else
#define WARNING_MSG(str) {}
#endif

// Define error messages
#ifdef ERROR_MSG_
#define ERROR_MSG(str) { std::cout << __progname << " " << getpid() << " \033[1;31mERROR\033[0m: " << str << std::endl; }
#else
#define ERROR_MSG(str) {}
#endif


#endif /* MSG_H */
