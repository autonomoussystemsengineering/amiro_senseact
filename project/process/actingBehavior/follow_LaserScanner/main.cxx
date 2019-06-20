//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Drives to the next (closest) object by using the distance
//               data of a laser scanner and stops in front of it in a
//               specified distance. If the object moves away, the robot
//               will follow. If the object comes closer, the robot will
//               drive backwards to hold the specified distance.
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "MSG.h"

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include "libs/CoreSLAM/CoreSLAM.h"
#ifdef __cplusplus
}
#endif

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>

#include <ControllerAreaNetwork.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

// constants
#define METERS_TO_MM    1000
#define MM_TO_METERS    0.001
ControllerAreaNetwork myCAN;
bool reversedScan = false;

// scopes
std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
std::string inScope = "/following";
std::string commandInit = "init";
std::string commandStop = "stop";

// scan values
bool firstScan = true;
int laserCount;
float laserAngleDist;
int minValueScan;
float startAngle;
float endAngle;
int orientationLaser;
int neighbourLasers;


// following constants
float followMinDist = 300; // mm
float followMinBackDist = 200; // mm
float followDistSlowingDown = 200; // mm
int forwardSpeed = 500; // mm/s
int forwardMinSpeed = 150; // mm/s
int turningSpeed = 100; // mradian/s
int turnCorrectSpeed = 60; //mradian/s
float rotationTolarence = M_PI/36.0; // rad
int maxRange = 2000; // mm
int maxAngle = 360; // degree


void motorActionMilli(int speed, int turn) {
//  printf("Speeds: v=%imm/s, w=%imrad/s\n", speed, turn);
  myCAN.setTargetSpeed(speed*1000, turn*1000);
}


void convertDataToScan(boost::shared_ptr< rst::vision::LocatedLaserScan > data , rst::vision::LocatedLaserScan &rsbScan) {
  rsbScan = *data;
}


void convertScan(rst::vision::LocatedLaserScan &scan, ts_sensor_data_t &data) {
  if (scan.scan_angle_end() < scan.scan_angle_start()){
    // flip readings
    endAngle = scan.scan_angle_start();
    startAngle = scan.scan_angle_end();
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*METERS_TO_MM);
  } else {
    startAngle = scan.scan_angle_start();
    endAngle = scan.scan_angle_end();
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(i)*METERS_TO_MM);
  }
  if (firstScan) {
    laserCount = scan.scan_values_size();
    minValueScan = (int) (scan.scan_values_min()*METERS_TO_MM);
    laserAngleDist = (endAngle-startAngle)/(laserCount-1);
    orientationLaser = laserCount/2;
    neighbourLasers = (60 * M_PI/180)/laserAngleDist;
    printf("\nLaser data:\n - #Laser: %i\n - Start angle: %frad\n - End angle: %frad\n - laser distance: %frad\n - Min Value: %imm\n\n", laserCount, startAngle, endAngle, laserAngleDist, minValueScan);
  }
}


void calculateDrivingBehavior(ts_sensor_data_t &scan) {
  int shortId = -1;
  int shortDist = -1;
  int startLaser = orientationLaser-neighbourLasers;
  int endLaser = orientationLaser+neighbourLasers;
  if (startLaser < 0) {
    startLaser = 0;
  }
  if (endLaser >= laserCount) {
    endLaser = laserCount-1;
  }

  float sa = 0.0;
  float ea = (float)(laserCount*laserAngleDist);
  if (endAngle-startAngle > (float)(maxAngle*M_PI/180.0)) {
    float angleDiff = (endAngle-startAngle) - (float)(maxAngle*M_PI/180.0);
    sa += angleDiff/2.0;
    ea -= angleDiff/2.0;
  }
  for (int laser=startLaser; laser <= endLaser; laser++) {
    if ((shortId < 0 || scan.d[laser] < shortDist) && scan.d[laser] > minValueScan && (float)(laser*laserAngleDist) >= sa && (float)(laser*laserAngleDist) <= ea) {
      shortId = laser;
      shortDist = scan.d[laser];
    }
  }

  if (shortId < 0 || shortDist > maxRange) {
    motorActionMilli(0,0);
    printf("No minimum found!\n");
  } else {
    orientationLaser = shortId;

    float angle = 2.0*M_PI-(startAngle+shortId*laserAngleDist);

    printf("Orientation: id=%i (aus [0,%i]), angle=%f\n", orientationLaser, laserCount-1, angle);

//    printf("Distance: %i mm, angle: %f rad\n", shortDist, angle);

    // Behavior calculation
    float alpha = angle;
    float beta = M_PI/2 - angle;
    float drivingRadius = shortDist*sin(beta)/sin(alpha);
    float drivingDist = 2*drivingRadius*M_PI * alpha/(2*M_PI);

    int rotationSpeed = (int)forwardSpeed*-alpha/drivingDist*1000;

//    printf("RotationSpeed=%imrad\n", rotationSpeed);

    if (shortDist > followMinDist) {
      int speed = forwardSpeed;
      if (shortDist < followMinDist+followDistSlowingDown) {
        float part = (shortDist-followMinDist)/followDistSlowingDown;
        speed = forwardMinSpeed + (int)((forwardSpeed-forwardMinSpeed)*part*part); //sqrt(part*part*part));
//      } else if (dist < followMinDistSide+followDistSlowingDown) {
//        float part = (dist-followMinDistSide)/followDistSlowingDown;
//        speed = forwardMinSpeed + (int)((forwardSpeed-forwardMinSpeed)*part*part); //sqrt(part*part*part));
      }
      motorActionMilli(speed, rotationSpeed);
    } else if (shortDist < followMinBackDist) {
      int speed = forwardSpeed/2;
      if (shortDist < followMinBackDist-followDistSlowingDown) {
        float part = (followMinBackDist-shortDist)/followDistSlowingDown;
        speed = forwardMinSpeed/2 + (int)((forwardSpeed/2-forwardMinSpeed)*part*part); //sqrt(part*part*part));
      }
      motorActionMilli(-speed, 0);
    } else if (abs(angle) > rotationTolarence/2.0) {
      if (rotationSpeed > 0) {
        rotationSpeed = min(rotationSpeed, turnCorrectSpeed);
      } else {
        rotationSpeed = max(rotationSpeed, -turnCorrectSpeed);
      }
      motorActionMilli(0, rotationSpeed);
    } else {
      motorActionMilli(0, 0);
    }
  }
}


int main(int argc, const char **argv){
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("startNow,s", "Starts the following immediately without waiting of scopes.")
    ("debug","Activates debugging information.")
    ("lidarinscope,l", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data.")
    ("followMinDist", po::value < float > (&followMinDist), "")
    ("followMinBackDist", po::value < float > (&followMinBackDist), "")
    ("followDistSlowingDown", po::value < float > (&followDistSlowingDown), "")
    ("forwardSpeed", po::value < int > (&forwardSpeed), "")
    ("forwardMinSpeed", po::value < int > (&forwardMinSpeed), "")
    ("turningSpeed", po::value < int > (&turningSpeed), "")
    ("turnCorrectSpeed", po::value < int > (&turnCorrectSpeed), "")
    ("maxRange", po::value < int > (&maxRange), "")
    ("maxAngle", po::value < int > (&maxAngle), "Maximum angle the following behavior shall be focus on in degrees (default: 360°).")
    ("rotationTolarence", po::value < float > (&rotationTolarence), "");

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

  bool followNow = vm.count("startNow");
  bool debugging = vm.count("debug");

  INFO_MSG("Lidar scope: " << lidarInScope);
  INFO_MSG("Command scope: " << inScope << " (with commands '" << commandInit << "' and '" << commandStop << "')");
  if (!followNow && debugging) {
    INFO_MSG("Waiting for init command.");
  }

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();
  
  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ///////////////////////////////////////////////////////////////////////////////
  // Get the global participant config as a template
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
        {
          // disable socket transport
          rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
          tmpPropSocket["enabled"] = boost::any(std::string("0"));

          // Get the options for spread transport, because we want to change them
          rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

          // enable socket transport
          tmpPropSpread["enabled"] = boost::any(std::string("1"));

          // Change the config
          tmpPropSpread["host"] = boost::any(std::string("localhost"));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string("4823"));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);


  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));


  // Prepare listener for input commands
  rsb::ListenerPtr commandListener = factory.createListener(inScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  commandListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));

  rst::vision::LocatedLaserScan laser;
  ts_sensor_data_t scan;
  while( true ){
    if (!commandQueue->empty()) {
      std::string command (*commandQueue->pop());
      if (command.compare(commandInit) == 0) {
        followNow = true;
        firstScan = true;
        if (debugging) {
          INFO_MSG("Init command received.");
        }
      } else if (command.compare(commandStop) == 0) {
        followNow = false;
        motorActionMilli(0,0);
        if (debugging) {
          INFO_MSG("Stop command received.");
        }
      }
    }
    if (followNow) {
      INFO_MSG("Next Meas");
      // Fetch a new scan and store it to scan
      convertDataToScan(lidarQueue->pop(), laser);
      // We can't initialize CoreSLAM until we've got the first scan
      convertScan(laser, scan);
      calculateDrivingBehavior(scan);
    }
    usleep(100000);
  }

  return 0;
}

