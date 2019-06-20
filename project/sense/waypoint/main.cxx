// std includes
#include <iostream>
using namespace std;

#define INFO_MSG_
#define DEBUG_MSG_

#include "../../includes/MSG.h"

#include <math.h>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/Informer.h>
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
// #include <types/LocatedLaserScan.pb.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;

int main(int argc, const char ** argv) {
  DEBUG_MSG("Start waypoint")
  namespace po = boost::program_options;
  std::string lidarInScope   = "/AMiRo_Hokuyo/lidar";
  std::string stateOutScope  = "/waypoint/state";
  std::string commandInscope = "/waypoint/command";
  float range         = 1.0;
  float diffThreshold = 0.3;

  int scanStartIndex = 0, scanEndIndex = -1;
  int consecutiveRaysThreshold = 0;

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.") ("lidarinscope", po::value<std::string>(&lidarInScope), "Scope for receiving lidar data")
    ("startNow,s", "Initializes the waypoint immediately without waiting for inscope.")
    ("stateoutscope", po::value<std::string>(&stateOutScope), "Scope for sending states")
    ("commandinscope", po::value<std::string>(&commandInscope), "Scope for receiving commands")
    ("range,r", po::value<float>(&range), "Range of detection in m")
    ("diffThreshold", po::value<float>(&diffThreshold), "Difference threshold in m")
    ("scanStartIndex", po::value<int>(&scanStartIndex))
    ("scanEndIndex", po::value<int>(&scanEndIndex))
    ("consecutiveRaysThreshold", po::value<int>(&consecutiveRaysThreshold));

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
  std::cout << options << "\n";
  exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan> > scanConverter(
    new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan>());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

  // ---------------- Informer ---------------------

  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LaserScan> > > lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LaserScan> >(1));
  lidarListener->addHandler(
    rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LaserScan>(lidarQueue)));

  rsb::ListenerPtr commandListner = factory.createListener(commandInscope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  commandListner->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));

  rsb::Informer<std::string>::Ptr stateInformer = factory.createInformer<std::string>(stateOutScope);

  rst::vision::LaserScan scan, initscan, initscan_raw;

  bool enabled   = false;
  // bool triggered = false;

  while (true) {
    // check if the checkpoint is enabled/disabled by the stateMachine
    if (!enabled) {
      bool initNow = vm.count("startNow");
      if (!initNow) {
        std::string command(*commandQueue->pop());
        initNow = command.compare("init") == 0;
      }

      if (initNow) {
        DEBUG_MSG("init")
        enabled = true;
        // initialize scan
        initscan_raw = *lidarQueue->pop();
        initscan     = initscan_raw;

        if (scanEndIndex == -1) {
          scanEndIndex = initscan.scan_values_size();
        }

        for (int i = scanStartIndex; i < scanEndIndex; ++i) {
          // if (initscan_raw.scan_values(i) < initscan_raw.scan_values_min())
          initscan_raw.set_scan_values(i, 10000);
        }
        // filter initial scan
        // remove outliers
        for (int i = scanStartIndex + 1; i < scanEndIndex - 1; ++i) {
          initscan.set_scan_values(i,
            min(initscan_raw.scan_values(i - 1),
              min(initscan_raw.scan_values(i), initscan_raw.scan_values(i + 1))));
          if (initscan.scan_values(i) == 10000) {
            initscan.set_scan_values(i, 0);
          }
        }


        for (int i = scanStartIndex; i < scanEndIndex; ++i) {
          DEBUG_MSG("init " << i << ": " << initscan.scan_values(i));
        }
      }
    } else {
      if (!commandQueue->empty()) {
        std::string command(*commandQueue->pop());
        if (command.compare("stop") == 0) {
          enabled = false;
        }
      }
    }

    // Fetch a new scan and store it to scan
    scan = *lidarQueue->pop();

    if (scanEndIndex == -1) {
      scanEndIndex = initscan.scan_values_size();
    }
    float scanScanValueMin = scan.scan_values(0);
    float initScanScanValueMin = initscan.scan_values(0);
    for (int i = 0; i < scan.scan_values().size(); i++) {
      float value = scan.scan_values(i);
      scanScanValueMin = min(value, scanScanValueMin);
    }
    for (int i = 0; i < initscan.scan_values().size(); i++) {
      float value = initscan.scan_values(i);
      initScanScanValueMin = min(value, initScanScanValueMin);
    }

    // check if the waypoint is triggered
    bool triggered_new        = false;
    int consecutive_ray_count = 0;
    for (int i = scanStartIndex + 1; i < scanEndIndex - 1; ++i) {
      if ((scan.scan_values(i) > scanScanValueMin && initscan.scan_values(i) > initScanScanValueMin && scan.scan_values(i) < range && initscan.scan_values(i) - scan.scan_values(i) > diffThreshold && scan.scan_values(i) < range) || (initscan.scan_values(i) < initScanScanValueMin && scan.scan_values(i) > scanScanValueMin && scan.scan_values(i) < range))
      {
        // DEBUG_MSG(i << " " << initscan.scan_values(i) << " " <<scan.scan_values(i) << " " << (initscan.scan_values(i) - scan.scan_values(i)));
        consecutive_ray_count++;
      } else {
        if (consecutive_ray_count > consecutiveRaysThreshold) {
          triggered_new = true;
          break;
        } else {
          consecutive_ray_count = 0;
        }
      }
    }

    DEBUG_MSG("consecutive: " << consecutive_ray_count);

    if (consecutive_ray_count > consecutiveRaysThreshold) {
      triggered_new = true;
    }

    // send update state
    if (triggered_new) {
      stateInformer->publish(Informer<string>::DataPtr(new string("entered")));
      DEBUG_MSG("entered");
    } else {
      stateInformer->publish(Informer<string>::DataPtr(new string("left")));
      DEBUG_MSG("left");
    }

    usleep(100000);
  }

  return 0;
} // main
