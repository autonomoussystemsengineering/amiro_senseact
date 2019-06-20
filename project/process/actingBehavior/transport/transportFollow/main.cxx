//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Following behavior of the transport scenario.
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

// estimation correction
#define CORRECTION_RANGESTART 0.01
#define CORRECTION_RANGEEND 0.06
#define CORRECTION_DISTSTART 0.01
#define CORRECTION_DISTGRADIANT 0.02 //0.069441
#define CORRECTION_ANGLESTART 0.0713
#define CORRECTION_ANGLEGRADIANT 0.805021 //0.0205021
#define CORRECTION_SIDEMEASSTART 0.004878
#define CORRECTION_SIDEMEASGRADIANT 0.67324


// Guide-Follower protocol
#define PROTOCOL_ERROR 0
#define PROTOCOL_OK 1
#define PROTOCOL_DISCONNECT 2


#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <sensorModels/VCNL4020Models.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;



// following commands
std::string FOLLOWCOMMAND_START = "START";
std::string FOLLOWCOMMAND_STOP = "STOP";
std::string FOLLOWCOMMAND_QUIT = "QUIT";

// radius of the AMiRo in m
float amiroRadius = 0.05;

// number of sensors
int numSensors = 8;
// maximal range in m
float maxSensorRange = 0.175;
// maximal angle in radian
float maxSensorAngle = M_PI * 60.0 / 180.0;
// sensor position offset from front in radian
float sensorPositionOffset = M_PI * 7.0 / 8.0;

// position of the Guide from tracking
float guideDiameter = 0.1; // m
float guideDistFix = 0.015; // m
float guideAngleFix = 135.0 * M_PI/180.0; // rad
float guideDirFix = 10.0 * M_PI/180.0; // rad

int trackingMarkerID = 0;
int finalePosMarker = 9;

float meterPerPixel = 1.0/400.0;

// following constants
float followMinDist = 0.04; // meters
float followMinDistSide = 0.09; // meters
float followDistSlowingDown = 0.1; // meters
int forwardSpeed = 8; // cm/s
int forwardMinSpeed = 3; // cm/s
int turningSpeed = 60; // cradian/s
int turnCorrectSpeed = 25; //cradian/s
float rotationTolarence = M_PI/36.0; // rad

// motor contorl
std::vector<int> motorCmd(3,0);



// scopenames for rsb
std::string proxSensorInscope = "/rir_prox/obstacle";
std::string followOutscope = "/follow/proximitysensors";
std::string trackingInscope = "/murox/roboterlocation";
std::string FollowerInscope = "/Transport/Follow";
std::string GuideInscope = "/Transport/Guide";
std::string mapServerScope = "/mapGenerator";
std::string rsbOutScope = "/motor/02";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";


// method prototypes
twbTracking::proto::Pose2D readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID);
bool isBeingTracked(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingID);
int motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN, rsb::Informer< std::vector<int> >::Ptr motorCmdInformer);

/*
 * Algorithm:
 * - Get transport command
 * - Calculate path to final position
 * - Get start position s on path with distance d = diameter + 1 cm
 * - Transmit start position s to Guide
 * - Wait until Guide transmits OK
 * - Initialize orientation ahead Guide
 * - Transmit final position to Guide
 * - Follow Guide until Guide transmits OK
 * - End transport command
 */

int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			     ("id", po::value<int>(&trackingMarkerID), "Tracking ID for AMiRo (default = 0)")
			     ("mpp", po::value<float>(&meterPerPixel), "Meter per pixel (default = 1/400 m/pixel)")
			     ("finaleID", po::value<int>(&finalePosMarker), "Tracking ID for finale position (default = 9)")
			     ("initializeMin,m", "Initialize the direction for the following task by using the minimum of the proximity sensors")
			     ("initializePos,p", "Initialize the direction for the following task by using the Guide's position")
			     ("host", po::value<std::string>(&spreadhost), "Host for Programatik Spread.")
			     ("port", po::value<std::string>(&spreadport), "Port for Programatik Spread.");

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

	// test plausibility
	if (vm.count("initializeMin") && vm.count("initializePos")) {
		std::cout << "ERROR: You can not use both initializations. Please choose between -m and -p" << std::endl;
		exit(1);
	}


	// afterwards, let program options handle argument errors
	po::notify(vm);

	INFO_MSG("Initialize RSB");

	boost::shared_ptr<twbTracking::proto::Pose2D> finalePos(new twbTracking::proto::Pose2D());

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register new converter for Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Listener ----------------------

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare rsb listener for progress data
        rsb::ListenerPtr progressListener = factory.createListener(GuideInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>progressQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	progressListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(progressQueue)));

	// ---------------- Informer ---------------------

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	// create rsb informer to publish progress data
	rsb::Informer<twbTracking::proto::Pose2D>::Ptr progressInformer = factory.createInformer<twbTracking::proto::Pose2D>(FollowerInscope, extspreadconfig);

	// create rsb informer for motor control
	rsb::Informer< std::vector<int> >::Ptr motorCmdInformer = factory.createInformer< std::vector<int> > (rsbOutScope);

	// create rsb informer for following behavior
	rsb::Informer<std::string>::Ptr informerCommandFollowing = factory.createInformer<std::string> (followOutscope);





	INFO_MSG("Initialize rest:");

	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	int state = 0;
	bool positionAvailable = true;
	bool oldPositionWithoutMoving = true;
	bool justWritten = false;
	bool continueFollowing = true;
	bool blink;
	boost::shared_ptr<twbTracking::proto::Pose2D> guidePos(new twbTracking::proto::Pose2D());
	twbTracking::proto::Pose2D ownPos;
	boost::shared_ptr<twbTracking::proto::Pose2DList> data;
	float guideDist, guideAngle;

	float sensorPos[numSensors][3];
	for (int i = 0; i < numSensors; ++i) {
		// calculate the position and angle of sensor i
		float sensorTheta = -2.0 * M_PI * i / numSensors + sensorPositionOffset;
		while (sensorTheta < 0) {
			sensorTheta += 2 * M_PI;
		}
		while (sensorTheta >= 2 * M_PI) {
			sensorTheta -= 2 * M_PI;
		}
		// add robot position offset to sensorposition
		float sensorX = cos(sensorTheta) * amiroRadius;
		float sensorY = sin(sensorTheta) * amiroRadius;

		sensorPos[i][0] = sensorX;
		sensorPos[i][1] = sensorY;
		sensorPos[i][2] = sensorTheta;
	}

	if (vm.count("initializeMin")) {

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
		}

		INFO_MSG("1) Waiting for given finale position");
		bool gotPosition = false;	
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
				if (isBeingTracked(data, finalePosMarker)) {
					gotPosition = true;
					twbTracking::proto::Pose2D readPos = readTracking(data, finalePosMarker);
					finalePos->set_x(readPos.x());
					finalePos->set_y(readPos.y());
				}
			}
			usleep(250000);
		} while (!gotPosition);
		INFO_MSG(" -> Finale position: " << finalePos->x() << "/" << finalePos->y());

		// wait until getting tracking data		
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
				if (isBeingTracked(data, trackingMarkerID)) {
					ownPos = readTracking(data, trackingMarkerID);
				}
			}
			usleep(250000);
		} while (!isBeingTracked(data, trackingMarkerID));

		// wait until lost tracking data
		INFO_MSG("2) Waiting for loosing tracking position");
//		data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
//		ownPos = readTracking(data, trackingMarkerID);

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
		}
		
		gotPosition = false;	
		do {
			if (!trackingQueue->empty()) {
				gotPosition = true;
				data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
				if (isBeingTracked(data, trackingMarkerID)) {
					ownPos = readTracking(data, trackingMarkerID);
				}
			}
			usleep(250000);
		} while (!gotPosition || isBeingTracked(data, trackingMarkerID));
		INFO_MSG(" -> own position (" << ownPos.x() << "/" << ownPos.y() << ")");

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
		}

		INFO_MSG("3) Create path");
		boost::shared_ptr<twbTracking::proto::Pose2DList> path = mapServer->call<twbTracking::proto::Pose2DList>("getPath", finalePos);

		// calculate start position
		twbTracking::proto::Pose2D step = path->pose(path->pose_size()-1);
		INFO_MSG("4) Next position (" << step.x() << "/" << step.y() << ")");
//		float dist = sqrt((ownPose.x()-step.x())*(ownPose.x()-step.x()) + (ownPose.y()-step.y())*(ownPose.y()-step.y()));
		float angle = atan2(step.y()-ownPos.y(), step.x()-ownPos.x());
		float dist = guideDiameter+0.02;
		guidePos->set_x(ownPos.x() + dist*cos(angle));
		guidePos->set_y(ownPos.y() + dist*sin(angle));
		INFO_MSG("5) Start position (" << guidePos->x() << "/" << guidePos->y() << ")");

		// send start position
		progressInformer->publish(guidePos);
		
		// wait for answer of guide
		INFO_MSG("6) Waiting for answer of guide");
		while (progressQueue->empty()) {
			usleep(500000);
		}
		progressQueue->pop();

		// send own position
		guidePos->set_x(ownPos.x());
		guidePos->set_y(ownPos.y());
		progressInformer->publish(guidePos);
		
		// wait for arrival of guide
		INFO_MSG("7) Waiting for arrival of guide");
		blink = true;
		while (progressQueue->empty()) {
			if(blink) {
				for(int led=0; led<8; led++) {
					myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
				}
			} else {
				for(int led=0; led<8; led++) {
					myCAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
				}
			}
			blink = !blink;				

			usleep(250000);
		}
		progressQueue->pop();

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
		}


		INFO_MSG("8) Initialization");

/*		// wait until gotten tracking data
		do {
			usleep(250000);
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
			}
		} while (!isBeingTracked(data, trackingMarkerID));

		INFO_MSG("Waiting for arrival of guide");

		// wait until lost tracking data
		do {
			usleep(250000);
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop());
			}
		} while (isBeingTracked(data, trackingMarkerID));
*/

	}

	
	bool goOn = vm.count("initializeMin");

	while (continueFollowing && goOn) {
		// check for new inputs
		if (!proxQueue->empty()) {

			// read proximity values
			boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int> >(proxQueue->pop());

			// do initialization part by minimum of proximity sensors
			float minDist = VCNL4020Models::obstacleModel(0, sensorValues->at(0));
			float minIdx = 0;
			for (int idx=1; idx<8; idx++) {
				float newMin = VCNL4020Models::obstacleModel(0, sensorValues->at(idx));
				if (minDist > newMin) {
					minDist = newMin;
					minIdx = idx;
				}
			}
			if (minIdx == 3 || minIdx == 4) {
				// position reached
				motorActionMilli(0, 0, myCAN, motorCmdInformer);
				goOn = false;
				progressInformer->publish(finalePos);
			} else if (minIdx < 3) {
				motorActionMilli(0, turningSpeed, myCAN, motorCmdInformer);
			} else {
				motorActionMilli(0, -turningSpeed, myCAN, motorCmdInformer);
			}

			if(!progressQueue->empty()) {
				guidePos = progressQueue->pop();
				continueFollowing = guidePos->orientation() != PROTOCOL_OK;
			}
		}
	}

	// start following behavior
	boost::shared_ptr<std::string> StringPtr(new std::string(FOLLOWCOMMAND_START));
	informerCommandFollowing->publish(StringPtr);

	do {
		usleep(200000);
		if(!progressQueue->empty()) {
			guidePos = progressQueue->pop();
			continueFollowing = guidePos->orientation() != PROTOCOL_OK;
		}
	} while (continueFollowing);

	// stop following behavior
	StringPtr = boost::shared_ptr<std::string>(new std::string(FOLLOWCOMMAND_STOP));
	informerCommandFollowing->publish(StringPtr);

	INFO_MSG("Finale position reached. Closing connection.");

	guidePos->set_x(0);
	guidePos->set_y(0);
	guidePos->set_orientation(PROTOCOL_DISCONNECT);
	progressInformer->publish(guidePos);

	for(int led=0; led<8; led++) {
		myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
	}

	// quit following application
	sleep(1);
	StringPtr = boost::shared_ptr<std::string>(new std::string(FOLLOWCOMMAND_QUIT));
	informerCommandFollowing->publish(StringPtr);

	// stop motor (if necesary)
	motorActionMilli(0, 0, myCAN, motorCmdInformer);

	return EXIT_SUCCESS;
}

twbTracking::proto::Pose2D readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			pose = data->pose(i);
			break;
		}
	}
	pose.set_x(pose.x()*meterPerPixel);
	pose.set_y(pose.y()*meterPerPixel);
	return pose;
}

bool isBeingTracked(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingID) {
	bool isIn = false;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingID == data->pose(i).id()) {
			if (data->pose(i).x() != 0 && data->pose(i).y() != 0) {
				isIn = true;
			}
			break;
		}
	}
	return isIn;
}

int motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN, rsb::Informer< std::vector<int> >::Ptr motorCmdInformer) {
  // Init motor vector
  motorCmd[0] = speed*10000;
  motorCmd[1] = turn*10000;
  motorCmd[2] = 5000000; // 5 s
  boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
  motorCmdInformer->publish(motorCmdData);
//  CAN.setTargetSpeed(speed*10000, turn*10000);
  return EXIT_SUCCESS;
}
