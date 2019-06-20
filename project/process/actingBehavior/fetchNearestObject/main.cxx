//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : The object, which is the closest to the AMiRo's normal,
//               will be delivered to the start position of the AMiRo
//               (simple fetch task). Colors:
//                 * blue: robot isn't fetching and it isn't on the table
//                 * yellow (blinking): robot has been set and will start
//                                      fetching soon
//                 * red: robot drives around the object
//                 * yellow (const): robot delivers object to start position
//                 * green: robot has finished
//============================================================================

//#define TRACKING
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>


#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include "libs/CoreSLAM/CoreSLAM.h"
#ifdef __cplusplus
}
#endif

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

using namespace std;

#include <stdint.h>  // int32

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>
#include <types/twbTracking.pb.h>

#include <ControllerAreaNetwork.h>
#include <Constants.h>
#include <sensorModels/VCNL4020Models.h>

using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;
using namespace amiro;

// constants
#define METERS_TO_MM    1000
#define MM_TO_METERS    0.001

// commands
std::string COMMAND_START = "START";
std::string COMMAND_QUIT = "QUIT";

// radius of the AMiRo in m
float amiroRadius = 0.05;
float amiroRadiusAddition = 0.0;
float secureDist = 0.05;

// driving constants
float forwardSpeed = 8.0; // cm/s
float forwardMinSpeed = 3.0; // cm/s
float turningSpeed = 25.0; // cradian/s
float turnCorrectSpeed = 25.0; //cradian/s
float rotationTolarence = M_PI/36.0; // rad
int turningAddition = 0;
int drivingAddition = 0;
float minAngle = 20.0*M_PI/180.0; // rad

// floor proximity constants
int floorMinValue = 15000;

// scan values
bool firstScan = true;
float watchAngleFromNormal = 30.0;
int laserCount;
float laserAngleDist;
int minValueScan;
float startAngle;
float endAngle;
int centerLaser;
int neighbourLasers;

// scopenames for rsb
std::string proxSensorInscope = "/rir_prox/obstacle";
std::string lidarInscope = "/AMiRo_Hokuyo/lidar";
std::string commandInscope = "/delivery/commands";

// flags
bool waitForCommand = false;
bool useFakeObject = false;
bool rgbdCameraIsUsed = false;
int angleDirectionPositive = 1;

// fake object
float fakeObjectRadius = 0.06; // m
float fakeObjectPosX = 1.0; // m
float fakeObjectPosY = 0.0; // m
float fakeRobotPosition = 0.0; // m


// method prototypes
void motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN);
void motorDrivePosition(float distanceM, float distanceRad, ControllerAreaNetwork &CAN);


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
		centerLaser = laserCount/2;
		neighbourLasers = (watchAngleFromNormal * M_PI/180.0)/laserAngleDist;
	}
}

bool calculateObjectOrientation(ts_sensor_data_t &scan, twbTracking::proto::Pose2D *pose2D) {
	int shortId = -1;
	int shortDist = -1;
	int startLaser, endLaser;
	if (rgbdCameraIsUsed) {
		startLaser = 0;
		endLaser = laserCount-1;
	} else {
		startLaser = centerLaser-neighbourLasers;
		endLaser = centerLaser+neighbourLasers;
		if (startLaser < 0) {
			startLaser = 0;
		}
		if (endLaser >= laserCount) {
			endLaser = laserCount-1;
		}
	}

	for (int laser=startLaser; laser <= endLaser; laser++) {
		if ((shortId < 0 || scan.d[laser] < shortDist) && scan.d[laser] > minValueScan) {
			shortId = laser;
			shortDist = scan.d[laser];
		}
	}

	if (shortId < 0) {
		WARNING_MSG("No object in range!");
		return false;
	} else {
		float angle = startAngle+shortId*laserAngleDist;
		pose2D->set_x(shortDist*MM_TO_METERS);
		pose2D->set_y(angle*angleDirectionPositive);
		INFO_MSG("Object: " << (shortDist*MM_TO_METERS) << " m, " << (angle*180.0/M_PI*angleDirectionPositive) << "°");
		return true;
	}
}


int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			     ("proximityScope,p", po::value<std::string>(&proxSensorInscope), "Scope for receiving floor proximity sensor values.")
			     ("lidarScope,l", po::value<std::string>(&lidarInscope), "Scope for receiving laser scans of the Hokuyo laser scanner.")
			     ("commandScope,c", po::value<std::string>(&commandInscope), "Scope for receiving commands.")
			     ("floorMinValue,f", po::value<int>(&floorMinValue), "Minimal value, which has to be measured by the floor sensors for detecting the table (default: 15000).")
			     ("watchAngle,w", po::value<float>(&watchAngleFromNormal), "Angular range in degrees for the laser scanner, how far to the sides it shall watch for objects (default: 30°).")
			     ("objectRadius,r", po::value<float>(&fakeObjectRadius), "Radius of the object in meters (default: 0.06 m).")
			     ("amiroRadiusAddition,a", po::value<float>(&amiroRadiusAddition), "Radius addition due to bigger cameras, etc. in meters (default: 0.0 m).")
                             ("anglePositiveToRight", "Flag, if the angle is not counted positive left side, but right side.")
                             ("rgbdLaser,d", "Flag, if not a laser scanner, but a RGBD camera is used for laser scans.")
                             ("waitForCommand", "Flag, if the robot should wait until the start command is given.")
                             ("useFakeObject", "Flag, if the fake object shall be used.");

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

	waitForCommand = vm.count("waitForCommand");
	useFakeObject = vm.count("useFakeObject");
	rgbdCameraIsUsed = vm.count("rgbdLaser");
	if (vm.count("anglePositiveToRight")) {
		angleDirectionPositive = -1;
	}

        INFO_MSG("Listening to the scopes:");
        INFO_MSG(" - Floor Proximity Sensors: " << proxSensorInscope);
	INFO_MSG(" - Laser Data:              " << lidarInscope);
        INFO_MSG(" - Command Input:           " << commandInscope);
        INFO_MSG("");

	INFO_MSG("Initialize RSB");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// Register new converter for rst::vision::LocatedLaserScan
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
	rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);


	// ------------ Listener ----------------------

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// Prepare RSB listener for incomming lidar scans
	rsb::ListenerPtr lidarListener = factory.createListener(lidarInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
	lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));

	// Create and start the command listener
	rsb::ListenerPtr listener = factory.createListener(commandInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
	listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));


	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	// initialize flags and counters
	bool startDelivery = false;
	bool justDelivered = false;
	int sensorErrorCounter = 0;
	boost::shared_ptr<std::vector<int>> sensorValues;
	rst::vision::LocatedLaserScan laser;
	ts_sensor_data_t scan;

	// initialize color
	for(int led=0; led<8; led++) {
		myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
	}

	INFO_MSG("Ready for Fetching Procedure");
	bool exitProg = false;
	while (!exitProg) {
		if (!proxQueue->empty()) {
			sensorErrorCounter = 0;

			// read floor proximity values
			sensorValues = boost::static_pointer_cast<std::vector<int> >(proxQueue->pop());

			if (justDelivered) {
				// check if robot has been taken
				justDelivered = false;
				for (int i=0; i<4; i++) {
					if (sensorValues->at(i) > floorMinValue) {
						justDelivered = true;
					}
				}
				if (!justDelivered) {
					DEBUG_MSG("Robot has been taken");
					for(int led=0; led<8; led++) {
						myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
					}
					sleep(1);
				}

			} else if (!startDelivery && !waitForCommand) {
				// check for starting procedure
				startDelivery = true;
				for (int i=0; i<4; i++) {
					if (sensorValues->at(i) < floorMinValue) {
						startDelivery = false;
					}
				}
				if (startDelivery) {
					DEBUG_MSG("Robot has been set");
					for(int led=0; led<8; led++) {
						myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
					}
				}
			}

		} else {
			if (sensorErrorCounter < 4) {
				sensorErrorCounter++;
			} else {
				ERROR_MSG("Didn't received actual floor sensor values for more than 500 ms!");
				sensorErrorCounter = 0;
			}
		}

		if (startDelivery && !justDelivered) {
			startDelivery = false;

			// wait a few seconds
			int waitStart = 3000; //ms
			for(int counter=0; counter < waitStart/200; counter++) {
				for(int led=0; led<8; led++) {
					if (led < 4 && counter % 2 == 0 || led >= 4 && counter % 2 > 0) {
						myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
					} else {
						myCAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
					}
				}
				usleep(300000);
			}

			for(int led=0; led<8; led++) {
				myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
			}

			// check for object position and size
			float objectRadius = fakeObjectRadius;
			float objectPosX = fakeObjectPosX;
			float objectPosY = fakeObjectPosY;
			float robotDirection = fakeRobotPosition;

			if (!useFakeObject) {
				twbTracking::proto::Pose2D objectPose;
				convertDataToScan(lidarQueue->pop(), laser);
				convertScan(laser, scan);
				bool found = calculateObjectOrientation(scan, &objectPose);
				if (found) {
					objectPosX = objectPose.x() + objectRadius;
					robotDirection = (-1.0) * objectPose.y();
				} else {
					objectPosX = 0.0;
				}
			}

			if (objectPosX != 0.0) {

				DEBUG_MSG("Own position: 0.0/0.0 m, " << (robotDirection*180.0/M_PI) << "°");
				DEBUG_MSG("Object position: " << objectPosX << "/0.0 m");

				// calculate path positions
				float direction = 1.0;
				if (robotDirection < 0.0 || robotDirection > M_PI) {
					direction = -1.0;
				}
				float saveDistance = amiroRadius + objectRadius + secureDist;

				twbTracking::proto::Pose2DList path;
				// add actual position
				twbTracking::proto::Pose2D *pose2D = path.add_pose();
				pose2D->set_x(0);
				pose2D->set_y(0);
				// add first edge position
				pose2D = path.add_pose();
				pose2D->set_x(objectPosX - saveDistance);
				pose2D->set_y(objectPosY + direction*saveDistance);
				// add second edge position
				pose2D = path.add_pose();
				pose2D->set_x(objectPosX + saveDistance);
				pose2D->set_y(objectPosY + direction*saveDistance);
				// add delivery start position
				pose2D = path.add_pose();
				pose2D->set_x(objectPosX + saveDistance);
				pose2D->set_y(0);
				// add final position
				pose2D = path.add_pose();
				pose2D->set_x(saveDistance);
				pose2D->set_y(0);

				// driving path
				twbTracking::proto::Pose2D actPos = path.pose(0);
				for (int i=1; i<5; i++) {
					if (i==4) {
						for(int led=0; led<8; led++) {
							myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
						}
					}
					twbTracking::proto::Pose2D focusPos = path.pose(i);
					INFO_MSG("Driving to position " << focusPos.x() << "/" << focusPos.y());
					// calculate distance and angle to position
					float xDiff = focusPos.x() - actPos.x();
					float yDiff = focusPos.y() - actPos.y();
					float drivingDist = sqrt(xDiff*xDiff + yDiff*yDiff);
					float drivingAngle = atan2(yDiff, xDiff) - robotDirection;
					// normalize angle and define turning direction
					if (drivingAngle > M_PI) drivingAngle -= 2*M_PI;
					if (drivingAngle < -M_PI) drivingAngle = 2*M_PI + drivingAngle;
					float direction = 1.0;
					if (drivingAngle < 0.0) direction = -1.0;

					// drive by odometry
					motorDrivePosition(drivingDist, drivingAngle, myCAN);

					actPos = path.pose(i);
					robotDirection = atan2(yDiff, xDiff);
				}
			}

			justDelivered = true;
			for(int led=0; led<8; led++) {
				myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
			}
		}

		if(!commandQueue->empty()) {
			std::string command = *commandQueue->pop().get();
			if (command == COMMAND_START) {
				startDelivery = true;
			} else if (command == COMMAND_QUIT) {
				exitProg = true;
			} else {
				WARNING_MSG("Received unknown command.");
			}
		}

		usleep(100000);
	}

	INFO_MSG("Closing following behavior.");

	return EXIT_SUCCESS;
}

void motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN) {
 	CAN.setTargetSpeed(speed*10000, turn*10000);
}

void motorDrivePosition(float distanceM, float angleRad, ControllerAreaNetwork &CAN) {
	float turnDirection = 1.0;
	if (angleRad < 0.0) turnDirection = -1.0;
	types::position robotPosition;
	float angleTolerance = turningSpeed*0.125/100.0; // rad
	float distTolerance = forwardSpeed*0.125/100.0; // m

	DEBUG_MSG("Driving action: " << distanceM << " m, " << (angleRad*180.0/M_PI) << " degree");
	DEBUG_MSG(" -> Turning " << (angleRad*180.0/M_PI) << " degree with " << (turningSpeed/100.0*180.0/M_PI) << " degree/s (about " << (angleTolerance*180.0/M_PI) << " degree tolerance)");
	DEBUG_MSG(" -> Driving " << distanceM << " m with " << (forwardSpeed/100.0) << " m/s (about " << distTolerance << " m tolerance)");

	// reset odometry
	robotPosition.x = 0;
	robotPosition.y = 0;
	robotPosition.f_z = 0;
	CAN.setOdometry(robotPosition);

	// turn
	CAN.setTargetSpeed(0, turnDirection*turningSpeed*10000);
	do {
		usleep(125000);
		robotPosition = CAN.getOdometry();
		// normalize angle
		if (robotPosition.f_z > M_PI*1000000.0) robotPosition.f_z -= 2*M_PI*1000000.0;
		if (robotPosition.f_z < -M_PI*1000000.0) robotPosition.f_z = 2*M_PI*1000000.0 + robotPosition.f_z;
	} while (abs(robotPosition.f_z) + angleTolerance < abs(angleRad*1000000.0));
	CAN.setTargetSpeed(0,0);

	usleep(300000);

	// reset odometry
	robotPosition.x = 0;
	robotPosition.y = 0;
	robotPosition.f_z = 0;
	CAN.setOdometry(robotPosition);

	// turn
	CAN.setTargetSpeed(forwardSpeed*10000, 0);
	do {
		usleep(125000);
		robotPosition = CAN.getOdometry();
	} while (robotPosition.x < distanceM*1000000.0);
	CAN.setTargetSpeed(0,0);

	usleep(300000);
}
