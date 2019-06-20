#include "MSG.h"

#include <math.h>
#include <utils.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <rst/vision/LaserScan.pb.h>
// #include <types/LaserScan.pb.h>

// RSC
#include <rsc/misc/SignalWaiter.h>

#include <ControllerAreaNetwork.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
namespace po = boost::program_options;

int main(int argc, const char **argv){

  // Handle program options
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string switchInScope = "/AMiRo_Hokuyo/emergencySwitch";
  int cntMax = 5;  // Number of consecutive scans
  float minDistance = 0.4;  // Maximum distance for emergency halt
  uint delay_ms = 10, delay_us;
  std::string emergencyHaltOutScope = "/AMiRo_Hokuyo/emergencyHalt";
  bool doEmergencyBehaviour = false;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("switchinscope", po::value < std::string > (&switchInScope), "Scope for switching the emergency behaviour: Accepts <On,ON,on,1> or <Off,OFF,off,0> as string")
    ("cntMax", po::value < int > (&cntMax), "Number of consecutive scans which need to be less than the given distance")
    ("distance", po::value < float > (&minDistance), "Maximum distance for emergency halt in meter")
    ("delay", po::value < uint > (&delay_ms), "Loop periodicity in milliseconds")
    ("emergencyHaltOutScope", po::value < std::string > (&emergencyHaltOutScope)->default_value(emergencyHaltOutScope), "Scope for publishing wether an emergency halt is performed")
    ("doEmergencyBehaviour", po::bool_switch(&doEmergencyBehaviour)->default_value(false), "Enable emergency behaviour from start on");

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


  rsb::Factory& factory = rsb::getFactory();

  // Register
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LaserScan>(lidarQueue)));

  rsb::ListenerPtr switchListener = factory.createListener(switchInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>switchQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  switchListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(switchQueue)));

  // Informer that informs other programs when an emergency halt is performed
  rsb::Informer<std::string>::Ptr emergencyHaltInformer = factory.createInformer<std::string>(emergencyHaltOutScope);
  boost::shared_ptr<std::string> haltInformerMsg(new std::string("halt"));

  rsc::misc::initSignalWaiter();

  // Create the CAN interface
  ControllerAreaNetwork CAN;

  boost::shared_ptr<rst::vision::LaserScan> scan;
  bool doHalt = false;
  delay_us = delay_ms * 1e3;
  enum state {
    nothing,
    emergencyHalt
  };
  state currentState = doEmergencyBehaviour ? state::emergencyHalt : state::nothing;

  while( rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED ){
    // Check if we have to switch the behaviour of the halt
    if ( !switchQueue->empty() ) {
      const std::string switchReq = *switchQueue->pop();
      if ( !switchReq.compare("0") || !switchReq.compare("off") || !switchReq.compare("Off") || !switchReq.compare("OFF") || !switchReq.compare("init") ) { // HACK: "init" means "init following" (openChallenge2016)
        currentState = state::nothing;
      } else if ( !switchReq.compare("1") || !switchReq.compare("on") || !switchReq.compare("On") || !switchReq.compare("ON") || !switchReq.compare("stop") ) { // HACK: "stop" means "stop following" (openChallenge2016)
        currentState = state::emergencyHalt;
      } else {
        ERROR_MSG( "Switch command does not match. Don't change current state. Received message: " << switchReq );
      }
    }

    // Fetch a new scan and store it to scan
    if ( !lidarQueue->empty() ) {
      scan = lidarQueue->pop();
      // Check if n consecutive scans are less than d meter
      int cnt = 0;
      DEBUG_MSG( "Size: " << scan->scan_values().size() );
      const int minIdx = scan->scan_values().size() / 2 - scan->scan_values().size() / 4;
      const int maxIdx = scan->scan_values().size() / 2 + scan->scan_values().size() / 4;
      doHalt = false;
      for (int idx = minIdx; idx < maxIdx; ++idx) {
        DEBUG_MSG( "Distance: " << int(scan->scan_values(idx) * 1000) << " mm" );
        if( scan->scan_values(idx) < minDistance && scan->scan_values(idx) > 0.05f ) {
          if ( ++cnt >= cntMax ) {
            doHalt = true;
            break;
          } else {
            doHalt = false;
          }
        } else {
          cnt = 0;
          doHalt = false;
        }
      }
    }

    // Halt if necessary
    if ( doHalt && ( currentState == state::emergencyHalt )) {
      CAN.setTargetSpeed(int(0), int(0));
      emergencyHaltInformer->publish(haltInformerMsg);
      DEBUG_MSG( "HALT" );
    }

    usleep(delay_us);
  }

  return 0;
}
