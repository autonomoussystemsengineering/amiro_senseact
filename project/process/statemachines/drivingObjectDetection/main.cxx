//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : The robot drives to every object and tries to detect it. If
//               the detection failed, the robot will try the detection from
//               another position.
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>


#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <Eigen/Geometry>

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
//#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

#include <ControllerAreaNetwork.h>

using namespace rsb;
using namespace rsb::patterns;

float diameter = 0.1;
int trackingMarkerID = 0;
float meterPerPixel = 1.0/400.0;

#define VEL_TURNING 40
#define VEL_FORWARD 8

float robotRadius = 0.05; // m
float robotObjectDist = 0.1; // m
float turnThreshold = 5 * M_PI/180; // rad

int objCount = 0;
int objOffset = 2;

// scopenames for rsb
std::string slamLocalizationScope = "/exploration";
std::string progressInscope = "/objectDetectionMain/command";
std::string progressOutscope = "/objectDetectionMain/answer";
std::string odometryInscope = "/localization";
std::string trackingInscope = "/murox/roboterlocation";
std::string sPathRequestOutput = "/pathReq/request";
std::string sPathRequestInput = "/pathReq/answer";
std::string pathResponseInscope = "/pathResponse";
std::string pathOutScope = "/path";
std::string mapServerScope = "/mapGenerator";
std::string pathRequestFunc = "getPath";
std::string objectOutscope = "/objectDetection/command";
std::string objectInscope = "/objectDetection/detected";
std::string rectInscope = "/rectangledata";

std::string outputRSBObjectDetection = "COMP";
std::string SLIdle = "idle";
std::string SLActive = "localization";

// string publisher
boost::shared_ptr<std::string> stringPublisher(new std::string);

std::string remoteHost = "localhost";
std::string remotePort = "4823";


// terminal flags
bool skipPP = false;
bool skipLP = false;
bool skipFR = false;
bool skipOD = false;
bool skipVC = false;
bool skipLocal = false;
bool useTrackingData = false;
bool isBigMap = false;
bool mapServerIsRemote = false;


// method prototypes
void getOwnPosition(types::position& ts_pose, rst::geometry::PoseEuler odomInput);
void readTracking(types::position& pose, boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID);
void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN);
float normAngle(float angle);
int mymcm(int mym);
int robotID = 0;


int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("robotID", po::value <int> (&robotID), "ID of robot for communication (default=0)")
			("progressInscope", po::value <std::string> (&progressInscope), "Inscope for progress data.")
			("progressOutscope", po::value <std::string> (&progressOutscope), "Outscope for progress data.")
			("positionInscope", po::value <std::string> (&odometryInscope), "Inscope for position data of SLAM localization.")
			("trackingInscope", po::value <std::string> (&trackingInscope), "Inscope for tracking data.")
			("pathResponseInscope", po::value <std::string> (&pathResponseInscope), "Inscope for Local Planner response.")
			("pathOutScope", po::value <std::string> (&pathOutScope), "Outscope for Local Planner.")
			("mapServerScope", po::value <std::string> (&mapServerScope), "Scope for Map Server.")
			("pathRequest", po::value <std::string> (&pathRequestFunc), "Function name for path request.")
			("objectOutscope", po::value <std::string> (&objectOutscope), "Outscope for object detection.")
			("objectInscope", po::value <std::string> (&objectInscope), "Inscope for object detection.")
			("rectInscope", po::value <std::string> (&rectInscope), "Inscope for rectangle data.")
			("trackingID", po::value <int> (&trackingMarkerID), "ID of tracking data (default: 0).")
			("meterPerPixel", po::value <float> (&meterPerPixel), "Meter per pixel of tracking data (default: 0.01).")
			("useTrackingData", "Use Tracking Data instead of PoseEuler for incomming position data.")
			("bigMap", "Flag if the map is very big (the pathplanner needs more than 25 seconds).")
			("mapServerIsRemote", "Flag, if the map server is a remote server (otherwise it uses local connection).")
			("skipPathPlanner", "Skipping Path Planner.")
			("skipLocalPlanner", "Skipping Local Planner.")
			("skipFinalRotation", "Skipping Final Rotation towards the object.")
			("skipDetection", "Skipping Object Detection.")
			("skipCorrection", "Skipping View Correction after detection failure.")
			("skipLocalization", "Skipping Localization. The robot then will be always at 0/0.");

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

	skipPP = vm.count("skipPathPlanner");
	skipLP = vm.count("skipLocalPlanner");
	skipFR = vm.count("skipFinalRotation");
	skipOD = vm.count("skipDetection");
	skipVC = vm.count("skipCorrection");
	skipLocal = vm.count("skipLocalization");
	useTrackingData = vm.count("useTrackingData");
        isBigMap = vm.count("bigMap");
	mapServerIsRemote = vm.count("mapServerIsRemote");
	if (vm.count("robotID")) {
		mapServerScope.append(std::to_string(robotID));
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
          tmpPropSpread["host"] = boost::any(std::string(remoteHost));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string(remotePort));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////

	// ------------ Converters ----------------------

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register converter for Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

	// Register converter for Pose
        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
        rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

	// Register converter for PoseEuler
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
	rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);

	// ------------ Listener ----------------------

	// prepare RSB listener for path request answer
	rsb::ListenerPtr pathRequestListener = factory.createListener(sPathRequestInput);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>pathRequestQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	pathRequestListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(pathRequestQueue)));

	// prepare RSB listener for path responses
	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare rsb listener for progress data
        rsb::ListenerPtr progressListener = factory.createListener(progressInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>progressQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	progressListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(progressQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare rsb listener for rectangle data
	rsb::ListenerPtr rectListener = factory.createListener(rectInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>rectQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	rectListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(rectQueue)));

	// Prepare RSB async listener for localization messages
	rsb::ListenerPtr odomListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>>odomQueue(new rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>(1));
	odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::PoseEuler>(odomQueue)));

        // Prepare RSB listener for object detection
        rsb::ListenerPtr objDetListener = factory.createListener(objectInscope);
        boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> objDetQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
        objDetListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(objDetQueue)));

	// ---------------- Informer ---------------------

	rsb::Informer<std::string>::Ptr informerSlamLocalization = factory.createInformer< std::string > (slamLocalizationScope);

        rsb::Informer<std::string>::Ptr objDetInformer = factory.createInformer< std::string > (objectOutscope);

	// create rsb informer to publish the robots path request
	rsb::Informer<twbTracking::proto::Pose2D>::Ptr pathRequestInformer = factory.createInformer<twbTracking::proto::Pose2D>(sPathRequestOutput);

	// create rsb informer to publish the robots path
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// map server
	RemoteServerPtr mapServer;
	if (mapServerIsRemote) {
		mapServer = factory.createRemoteServer(mapServerScope, tmpPartConf, tmpPartConf);
	} else {
		mapServer = factory.createRemoteServer(mapServerScope);
	}

	// create rsb informer to publish progress data
	rsb::Informer<twbTracking::proto::Pose2D>::Ptr progressInformer = factory.createInformer<twbTracking::proto::Pose2D>(progressOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

        // do algorithm
	rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);
	boost::shared_ptr<twbTracking::proto::Pose2DList> rectPositionsPtr(new twbTracking::proto::Pose2DList());
	boost::shared_ptr<twbTracking::proto::Pose2DList> objectPositionsPtr(new twbTracking::proto::Pose2DList());
	boost::shared_ptr<twbTracking::proto::Pose2DList> objectPositions(new twbTracking::proto::Pose2DList());
	boost::shared_ptr<twbTracking::proto::Pose2D> detectionPositionPtr(new twbTracking::proto::Pose2D());
	types::position ownPos;

	/*
	 * For every object, do:
         * 1) Calculate final detection position (fdp)
         * 2) Get path to fdp
         * 3) Drive to fdp
         * 4) Rotate towards object
         * 5) Do object detection
         * 6) If not detected:
         * 6.1) Drive away from object
         * 6.2) Continue with 2)
         * 7) Publish result
         */
        INFO_MSG("Start progress");

	if (skipLocal) {
		ownPos.x = 0;
		ownPos.y = 0;
		ownPos.f_z = 0;
	}

	while (rectQueue->empty()) {
		usleep(500000);
	}
	rectPositionsPtr = rectQueue->pop();
	float angle = M_PI/2.0 - atan2(rectPositionsPtr->pose(1).y(), rectPositionsPtr->pose(1).x());
	INFO_MSG("Table turned for " << angle << " rad.");
/*	for (int i=1; i<rectPositionsPtr->pose_size(); i++) {
		rectPositionsPtr->mutable_pose(i)->set_x(rectPositionsPtr->pose(i).x()*cos(angle));
		rectPositionsPtr->mutable_pose(i)->set_y(rectPositionsPtr->pose(i).y()*sin(angle));
	}*/
	float borders[5];
	borders[0] = angle;
/*	borders[1] = rectPositionsPtr->pose(0).x()*cos(angle) - rectPositionsPtr->pose(0).y()*sin(angle);
	borders[2] = rectPositionsPtr->pose(1).x()*cos(angle) - rectPositionsPtr->pose(1).y()*sin(angle);
	borders[3] = rectPositionsPtr->pose(0).x()*sin(angle) + rectPositionsPtr->pose(0).y()*cos(angle);
	borders[4] = rectPositionsPtr->pose(2).x()*sin(angle) + rectPositionsPtr->pose(2).y()*cos(angle);
*/	borders[1] = rectPositionsPtr->pose(0).x();
	borders[2] = rectPositionsPtr->pose(1).x();
	borders[3] = rectPositionsPtr->pose(0).y();
	borders[4] = rectPositionsPtr->pose(2).y()*(-1);
        INFO_MSG("Edge positions:");
        for (int i=0; i<3; i++) {
		INFO_MSG(" - Pose " << i << ": " << rectPositionsPtr->pose(i).x() << "/" << rectPositionsPtr->pose(i).y());
	}

	while (true) {
		if (!progressQueue->empty()) {
			objectPositionsPtr = progressQueue->pop();

			if (useTrackingData && !skipLocal) {
				readTracking(ownPos, boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()), trackingMarkerID); // u[m, rad]
			} else if (!skipLocal) {
				getOwnPosition(ownPos, *odomQueue->pop()); // u[m, rad]
			}
			bool allObjectsChoosen = false;
			bool objectChoosen[objectPositionsPtr->pose_size()];
			for (int i=0; i<objectPositionsPtr->pose_size(); i++) {
				objectChoosen[i] = false;
			}
			INFO_MSG("Object position list with " << objectPositionsPtr->pose_size() << " objects:");
			while (!allObjectsChoosen) {
				float maxDist = 0;
				int maxIdx = -1;
				for (int i=0; i<objectPositionsPtr->pose_size(); i++) {
					if (!objectChoosen[i]) {
						float px = objectPositionsPtr->pose(i).x();
						float py = objectPositionsPtr->pose(i).y() * (-1);
						INFO_MSG("Original position: " << px << "/" << py);
						float pxA = px * cos(borders[0]) - py * sin(borders[0]);
						float pyA = px * sin(borders[0]) + py * cos(borders[0]);
						if (borders[1] <= px && px <= borders[2] && borders[3] <= py && py <= borders[4]) {
							float diffY = py - ((float)ownPos.y)/1000000.0;
							float diffX = px - ((float)ownPos.x)/1000000.0;
							float dist = sqrt(diffY*diffY + diffX*diffX);
							if (dist > maxDist) {
								maxDist = dist;
								maxIdx = i;
							}
						} else {
							objectChoosen[i] = true;
							WARNING_MSG("Object " << i << " is not on the table (" << borders[1] << " <= " << px << " <= " << borders[2] << ", " << borders[3] << " <= " << py << " <= " << borders[4] << ").");
						}
					}
				}
				if (maxIdx < 0) {
					WARNING_MSG("No objects found!");
				} else {

					objectChoosen[maxIdx] = true;

//				for (int i=0; i<objectPositionsPtr->pose_size(); i++) {
					twbTracking::proto::Pose2D *pose2D = objectPositions->add_pose();
					pose2D->set_x(objectPositionsPtr->pose(maxIdx).x()); // m
					pose2D->set_y(objectPositionsPtr->pose(maxIdx).y()); // m
					pose2D->set_orientation(objectPositionsPtr->pose(maxIdx).orientation()); // m (radius)
					pose2D->set_id(objectPositionsPtr->pose(maxIdx).id());
					INFO_MSG(" - obj " << pose2D->id() << " at " << pose2D->x() << "/" << pose2D->y() << " with r=" << pose2D->orientation() << " m");
				}

				allObjectsChoosen = true;
				for (int i=0; i<objectPositionsPtr->pose_size(); i++) {
					if (!objectChoosen[i]) {
						allObjectsChoosen = false;
					}
				}
			}

			if (objectPositions->pose_size() == 0) {
				WARNING_MSG("Add fake object.")
				twbTracking::proto::Pose2D *pose2D = objectPositions->add_pose();
				pose2D->set_x(0.1);
				pose2D->set_y(-0.1);
				pose2D->set_orientation(0.0);
				pose2D->set_id(0);
			}

			detectionPositionPtr->set_x(0);
			detectionPositionPtr->set_y(0);
			detectionPositionPtr->set_orientation(0);
			detectionPositionPtr->set_id(objectPositions->pose_size());
			progressInformer->publish(detectionPositionPtr);

			// check all objects
			for (int i=objectPositions->pose_size()-1; i >= 0; i--) {

				INFO_MSG("Try to categorize object number "  << i);

				// load object position
				twbTracking::proto::Pose2D objectPosition = objectPositions->pose(i); // m

        	                bool objectDetected = false;
//				float detectionDist = robotRadius + robotObjectDist + objectPosition.orientation()/2.0; // m
				float detectionDist = robotObjectDist + objectPosition.orientation()/2.0; // m
				INFO_MSG("Focussing on object at position " << objectPosition.x() << "/" << objectPosition.y() << " with a radius of " << objectPosition.orientation() << " m");

				// if the object hasn't been detected yet
        	                while(!objectDetected) {

					// calculate final detection position
					if (useTrackingData && !skipLocal) {
						readTracking(ownPos, boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()), trackingMarkerID); // u[m, rad]
					} else if (!skipLocal) {
						getOwnPosition(ownPos, *odomQueue->pop()); // u[m, rad]
					}
					INFO_MSG("Find oneself at position " << (float)ownPos.x/1000000.0 << "/" << (float)ownPos.y/1000000.0);
					float angleToObject = atan2(objectPosition.y()-((float)ownPos.y)/1000000.0, objectPosition.x()-((float)ownPos.x)/1000000.0); // rad
					detectionPositionPtr->set_x(objectPosition.x()-detectionDist*cos(angleToObject)); // m
					detectionPositionPtr->set_y(objectPosition.y()-detectionDist*sin(angleToObject)); // m
					detectionPositionPtr->set_orientation(0.0); // rad
					INFO_MSG("Final detection position is " << detectionPositionPtr->x() << "/" << detectionPositionPtr->y());
/*
					rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
					twbTracking::proto::Pose2D *pose2D_1 = pose2DList->add_pose();
					pose2D_1->set_x(((float)ownPos.x)/1000000.0);
					pose2D_1->set_y(((float)ownPos.y)/1000000.0);
					pose2D_1->set_orientation(0);
					pose2D_1->set_id(0);
					twbTracking::proto::Pose2D *pose2D_2 = pose2DList->add_pose();
					pose2D_2->set_x(((float)detectionPositionPtr->x())/1000000.0);
					pose2D_2->set_y(((float)detectionPositionPtr->y())/1000000.0);
					pose2D_2->set_orientation(0);
					pose2D_2->set_id(0);
					pathInformer->publish(pose2DList);
					sleep(1);
					if (!pathResponseQueue->empty()) {
						pathResponseQueue->pop();
					}
					INFO_MSG("Waiting for Local Planner.");
					while(pathResponseQueue->empty());
					INFO_MSG("Local Planner finished.");
*/

					// calculate path to final detection position


					boost::shared_ptr<twbTracking::proto::Pose2DList> path;
					if (!skipPP) {
//						detectionPositionPtr->set_x(0.0);
//						detectionPositionPtr->set_y(0.0);
//						detectionPositionPtr->set_orientation(0.0);
						if (isBigMap) {
							if (!pathRequestQueue->empty()) {
								pathRequestQueue->pop();
							}
							pathRequestInformer->publish(detectionPositionPtr);
							INFO_MSG("Waiting for path:");
							while (pathRequestQueue->empty()) {
								sleep(1);
								INFO_MSG("Still waiting ...");
							}
							pathRequestQueue->pop();
						}
						path = mapServer->call<twbTracking::proto::Pose2DList>(pathRequestFunc, detectionPositionPtr);
						INFO_MSG("Calculated path (#items: " << path->pose_size() << "):");
						for (int i=0; i<path->pose_size(); i++) {
							INFO_MSG(" -> " << path->pose(i).x() << "/" << path->pose(i).y());
						}
					}

					if (!skipLP) {
						// drive to final detection position
						if (!pathResponseQueue->empty()) {
							pathResponseQueue->pop();
						}
						pathInformer->publish(path);
						while (pathResponseQueue->empty()) {
							usleep(500);
						}
						pathResponseQueue->pop();
						INFO_MSG("Finished driving.");
					}

					// rotate towards object
					if (!skipFR) {
						if (useTrackingData && !skipLocal) {
							readTracking(ownPos, boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()), trackingMarkerID); // u[m, rad]
						} else if (!skipLocal) {
							getOwnPosition(ownPos, *odomQueue->pop()); // u[m, rad]
						}
						float ownAngle = normAngle(((float)ownPos.f_z)/1000000.0); // rad
        		                        DEBUG_MSG("Own Position is " << ownPos.x << "/" << ownPos.y << " (" << ownAngle << " rad)");
						float angle = atan2(objectPosition.y()-ownPos.y, objectPosition.x()-ownPos.x); // rad
						float turnAngle = 0;
						float angleDiff = abs(angle-ownAngle); // rad
						if (angleDiff < M_PI && angleDiff > turnThreshold) {
							turnAngle = angle-ownAngle;
						} else {
							turnAngle = angle-ownAngle+2*M_PI;
						}
        		                        DEBUG_MSG("Angle = " << angle << ", angle diff = " << angleDiff << ", turn angle = " << turnAngle);
						float fac = 1;
						if (turnAngle < 0) fac = -1;
						int waitingTime_us = (int)(((turnAngle*1000.0) / ((float)VEL_TURNING*10.0*fac)) * 1000000.0); // us
						INFO_MSG("Turning for " << turnAngle << " rad for " << (waitingTime_us/1000) << " ms with a speed of " << fac*VEL_TURNING/100 << " rad/s");
						sendMotorCmd(0, mymcm(fac*VEL_TURNING), myCAN);
						usleep(waitingTime_us);
						sendMotorCmd(0, 0, myCAN);
						sleep(5);
					}

					boost::shared_ptr<std::string> stringPublisher(new std::string);
					*stringPublisher = SLIdle;
					informerSlamLocalization->publish(stringPublisher);
					if (!skipOD) {
						// do object detection
						if (!objDetQueue->empty()) {
							objDetQueue->pop();
						}
						*stringPublisher = outputRSBObjectDetection;
						objDetInformer->publish(stringPublisher);
						while (!objDetQueue->empty()) {
							usleep(500000);
						}
						std::string objInput = *objDetQueue->pop();
	
						// progress input
						if (!objInput.compare("null") == 0) {
							// object has been detected
							int objNum = std::stoi(objInput);
							INFO_MSG("Object " << objNum << " has been detected (position " << objectPosition.x() << "/" << objectPosition.y() << " and radius " << objectPosition.orientation() << ").");
							objectDetected = true;
							detectionPositionPtr->set_x(objectPosition.x());
							detectionPositionPtr->set_y(objectPosition.y());
							detectionPositionPtr->set_orientation(((float)objectPosition.orientation())/1000000.0);
							detectionPositionPtr->set_id((float)objNum);
							progressInformer->publish(detectionPositionPtr);

						} else if (!skipVC) {
							// correct object view if not detected
							WARNING_MSG("Object couldn't be detected -> do correction movement.");

							// turn left for 90 degrees
							float movement = M_PI/2.0;
							int waiting_us = (int)(((movement*1000.0) / ((float)VEL_TURNING*10.0)) * 1000000.0);
							INFO_MSG(" -> Turning for " << movement << " rad for " << (waiting_us/1000) << " ms with a speed of " << VEL_TURNING/100 << " rad/s");
							sendMotorCmd(0, mymcm(VEL_TURNING), myCAN);
							usleep(waiting_us);
							sendMotorCmd(0, 0, myCAN);

							// drive forward for 5 cm
							movement = 5;
							waiting_us = (int)((movement / ((float)VEL_FORWARD)) * 1000000.0);
							INFO_MSG(" -> Moving for " << movement << " cm for " << (waiting_us/1000) << " ms with a speed of " << VEL_FORWARD << " cm/s");
							sendMotorCmd(mymcm(VEL_FORWARD), 0, myCAN);
							usleep(waiting_us);
							sendMotorCmd(0, 0, myCAN);
						}
					} else {
						// object has been detected (fake detection)
						INFO_MSG("DETECTING");
						sleep(10);
						int objNum = objCount+objOffset+robotID+1;
						objCount++;
						INFO_MSG("Object " << objNum << " has been fake detected (position " << objectPosition.x() << "/" << objectPosition.y() << " and radius " << objectPosition.orientation() << ").");
						objectDetected = true;
						detectionPositionPtr->set_x(objectPosition.x());
						detectionPositionPtr->set_y(objectPosition.y());
						detectionPositionPtr->set_orientation(objectPosition.orientation());
						detectionPositionPtr->set_id(objNum);
						progressInformer->publish(detectionPositionPtr);
					}
					*stringPublisher = SLActive;
					informerSlamLocalization->publish(stringPublisher);
					
				} // end while not detected
			} // end for each object	
		} else {
			// just wait for new input
			usleep(500000);
		}
	}

	return EXIT_SUCCESS;
}


void getOwnPosition(types::position& pose, rst::geometry::PoseEuler odomInput) {
	// Save data
	pose.x = odomInput.mutable_translation()->x()*1000000;
	pose.y = odomInput.mutable_translation()->y()*1000000;
	pose.f_z = odomInput.mutable_rotation()->yaw()*1000000;
}

void readTracking(types::position& pose, boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D poseSave;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			poseSave = data->pose(i);
			break;
		}
	}
	pose.x = poseSave.x() * meterPerPixel * 1000000;
	pose.y = poseSave.y() * meterPerPixel * 1000000;
	pose.f_z = poseSave.orientation() * M_PI/180.0 * 1000000;
}


float normAngle(float angle) {
	while (angle >= 2*M_PI) {
		angle -= 2*M_PI;
	}
	while (angle < 0) {
		angle += 2*M_PI;
	}
	return angle;
}


void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed(speed, angle);
	DEBUG_MSG( "v: " << speed << "w: " << angle);
}

int mymcm(int mym) {
	return mym*10000;
}


