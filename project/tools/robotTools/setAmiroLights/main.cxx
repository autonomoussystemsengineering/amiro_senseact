//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Setting lightring colors by commandline
//============================================================================


 #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>

#include <ControllerAreaNetwork.h>

#include <vector>
//#include "ControllerAreaNetwork.h"
#include <iostream>

#include <Types.h>  // types::position
//#include <boost/thread.hpp>
// For program options
#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>


//#include <rsb/Informer.h>
//#include <rsb/Factory.h>
//#include <rsb/Event.h>
//#include <rsb/Handler.h>
//#include <rsb/converter/Repository.h>


using namespace std;
using namespace amiro;

int g_red = 0;
int g_green = 0;
int g_blue = 0;


int main(int argc, char **argv)
{
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("red,r", po::value < int > (&g_red), "Red [0-255]")
    ("green,g", po::value < int > (&g_green), "Green [0-255]")
    ("blue,b", po::value < int > (&g_blue), "Blue [0-255]");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(
    po::command_line_parser(argc, argv).options(options).positional(p).run(),
    vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  ControllerAreaNetwork myCAN;
  
 for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
   INFO_MSG( "Set LED " << ledIdx << " to r: " << g_red << " g: " << g_green << " b: " << g_blue )
    myCAN.setLightColor(ledIdx, amiro::Color(g_red,g_green,g_blue));
    // sleep(1);
  }


 return 0;

}