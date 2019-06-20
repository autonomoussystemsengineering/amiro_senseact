#include <sick_tim/sick_tim_common_usb.h>
#include <sick_tim/sick_tim551_2050001_parser.h>

#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/misc/SignalWaiter.h>
#include <boost/program_options.hpp>
#include <iostream>
//#include <converter/vecIntConverter/main.hpp>

int main(int argc, char **argv)
{

  // Handle program options
  namespace po = boost::program_options;

  std::string port = "2112";
  int timelimit = 5;
  // float range_min = 0.05f;
  // float range_max = 25.0f;  // TIM 571
  float time_increment = -1.0f;
  std::string rsbOutScope = "/AMiRo/lidar";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope sending the LIDAR data")
    ("timelimit", po::value < int > (&timelimit), "Timelimit for LIDAR beeing idle")
    ("time_increment", po::value < float > (&time_increment), "Time increment between scans")
    ("port", po::value < std::string > (&port), "Port for TCP operation");

    // ("range_max", po::value < float > (&range_max), "Maximal distance to be detected")
    // ("range_min", po::value < float > (&range_min), "Minimal distance to be detected")

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Create SICK device
  sick_tim::SickTim5512050001Parser* parser = new sick_tim::SickTim5512050001Parser();

  sick_tim::SickTimCommon* s = NULL;

  int result = sick_tim::ExitError;
  rsc::misc::initSignalWaiter();
  while (rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED)
  {
    // Atempt to connect/reconnect
    delete s;
    s = new sick_tim::SickTimCommonUsb(parser, rsbOutScope);
    result = s->init();

    while((rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED) && (result == sick_tim::ExitSuccess)){
      result = s->loopOnce();
    }

    if (result == sick_tim::ExitFatal)
      return result;
  }

  delete s;
  delete parser;
  return result;
}
