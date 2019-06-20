//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Central control of the robots map building and exploration.
//============================================================================

// std includes
#include <iostream>
using namespace std;

// opencv
#include <opencv2/core/core.hpp>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
namespace po = boost::program_options;

// rsb
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/util/QueuePushHandler.h>
using namespace rsb;
using namespace rsb::patterns;

// converters
#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include "extspread.hpp"
#ifdef USECAN
#include <ControllerAreaNetwork.h>
#endif
// camera parameter
float meterPerPixel = 1.0 / 400.0;

// Constants
enum SENSOR_POS {
	MIDDLE_RIGHT = 0, RIGHT = 1, LEFT = 2, MIDDLE_LEFT = 3
};
enum EDGE_SIDE {
	UNKNOWN, LEFTSIDE, RIGHTSIDE
};
enum State {
	FRONTIER, EDGE, ABORTEDGE
};

static const int V = 30000;
static const int W = 300000;
static const int DT = std::numeric_limits<int>::max();

int floorProxMinValues[4] = { 3292, 3803, 3256, 3039 };
int floorProxMaxValues[4] = { 27291, 25145, 24887, 26233 };

bool lineBelow[4] = { false, false, false, false };
// Datastructure for the CAN messages
const boost::shared_ptr<std::vector<int>> vecSteering(new std::vector<int>(3));

rsb::Informer<std::vector<int>>::Ptr steeringInformer;
float floorProxValuesNormalized[4];

void updateSensors(std::vector<int> floorProxValues) {

	for (uint8_t sensorIdx = 0; sensorIdx < floorProxValues.size(); sensorIdx++) {
		floorProxValuesNormalized[sensorIdx] = (floorProxValues[sensorIdx] - floorProxMinValues[sensorIdx])
				/ ((float) floorProxMaxValues[sensorIdx]);

		lineBelow[sensorIdx] = floorProxValuesNormalized[sensorIdx] < 0.7;
	}
}

// send new steering commands to the motorControl
// only send the new steering command if it differs from the last steering command
// returns true if a new command was send
void setSteering(int v, int w, int duration) {
	if (v != vecSteering->at(0) || w != vecSteering->at(1)) {
		vecSteering->at(0) = v;
		vecSteering->at(1) = w;
		vecSteering->at(2) = duration;
		steeringInformer->publish(vecSteering);
	}
}
// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose;
	pose.set_x(0);
	pose.set_y(0);
	pose.set_orientation(0);
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			pose = data->pose(i);
			break;
		}
	}
	return cv::Point3f(pose.x() * meterPerPixel, pose.y() * meterPerPixel, pose.orientation() * M_PI / 180.0);
}

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string proxSensorInscope = "/rir_prox";
	std::string trackingInscope = "/murox/roboterlocation";
	std::string floorInscope = "/prox/floor";
	std::string pathResponseInscope = "/pathResponse";
	std::string pathOutScope = "/path";
	std::string edgeOutscope = "/edge";
	std::string edgePoseOutscope = "/edgePose";
	std::string steeringOutScope = "/motor/04";
	std::string mapServerScope = "/mapGenerator";
	std::string frontierCommandScope = "/exploration/command";
	std::string frontierResponseScope = "/exploration/answer";

	bool advancedEdge = false, predictPaths = false;

	std::string spreadhost = "127.0.0.1";
	std::string spreadport = "4803";

	// id of the tracking marker
	int trackingMarkerID = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("id", po::value<int>(&trackingMarkerID),
			"ID of the tracking marker")("edgeout", po::value<std::string>(&edgeOutscope),
			"Outscope for edge found signals")("pathRe", po::value<std::string>(&pathOutScope),
			"Inscope for path responses.")("pathOut", po::value<std::string>(&pathOutScope),
			"Outscope for the robots path.")("irin", po::value<std::string>(&proxSensorInscope),
			"Inscope for ring prox sensors.")("mapServer", po::value<std::string>(&mapServerScope),
			"Scope for the mapGenerator server")("host", po::value<std::string>(&spreadhost),
			"Host for Programatik Spread.")("port", po::value<std::string>(&spreadport), "Port for Programatik Spread.")(
			"meterPerPixel,mpp", po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel")(
			"frontierCommandScope", po::value<std::string>(&frontierCommandScope), "frontierCommandScope")(
			"frontierResponseScope", po::value<std::string>(&frontierResponseScope), "frontierResponseScope");
	;

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

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// register converter for cv::Mat
	boost::shared_ptr<MatConverter> matConverter(new MatConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// ---------- Listener ---------------

	// prepare RSB listener for the floor prox data
	rsb::ListenerPtr floorListener = factory.createListener(floorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >floorQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	floorListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(floorQueue)));

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare RSB listener for path responses
	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare RSB listern for start signa;
	rsb::ListenerPtr startListener = factory.createListener(frontierCommandScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>startQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	startListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(startQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer = factory.createInformer<
			twbTracking::proto::Pose2DList>(pathOutScope);

	// create rsb informer to publish edge signals
	rsb::Informer<bool>::Ptr edgeDetectionInformer = factory.createInformer<bool>(edgeOutscope);

	// create rsb informer to publish edge poses
	rsb::Informer<twbTracking::proto::Pose2D>::Ptr edgePoseInformer =
			factory.createInformer<twbTracking::proto::Pose2D>(edgePoseOutscope);

	// create rsb informer to publish steering commands
	steeringInformer = factory.createInformer<std::vector<int>>(steeringOutScope);

	// create rsb informer to publish a signal when the exploration is finished
	rsb::Informer<string>::Ptr finishedInformer = factory.createInformer<string>(frontierResponseScope);

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	// initialize variables
	cv::Point3f edgeApproachPose, robotPose, lastPose;
	cv::Point2f pathEnd(-100, -100);
	cv::Mat map;
	bool running = true, unknownEdge = false, edgeLocked = false;
	EDGE_SIDE edgeSide = UNKNOWN;
	State currentState = FRONTIER;

	// ------------------------- Here the actual algorithm begins -----------------------------------------

	// wait for start signal
	startQueue->pop();

	// wait for tracking
	while (robotPose == cv::Point3f(0, 0, 0)) {
		robotPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),
				trackingMarkerID);
		lastPose = robotPose;
	}

	// send initial path
	std::vector<cv::Point2f> initialPath = { cv::Point2f(robotPose.x + cos(robotPose.z) * 0.05,
			robotPose.y + sin(robotPose.z) * 0.05) };
	rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr initialP2DList(new twbTracking::proto::Pose2DList);
	for (Point2f p : initialPath) {
		twbTracking::proto::Pose2D *pose2D = initialP2DList->add_pose();
		pose2D->set_x(p.x);
		pose2D->set_y(p.y);
		pose2D->set_orientation(0);
		pose2D->set_id(0);
	}
	pathInformer->publish(initialP2DList);

#ifdef USECAN
	ControllerAreaNetwork myCAN;

	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
#endif

	while (running) {

		// read floor data
		if (!floorQueue->empty()) {
			const boost::shared_ptr<std::vector<int>> floorValues = boost::static_pointer_cast<std::vector<int> >(
					floorQueue->pop());
			updateSensors(*floorValues);
		}

		// read tracking data
		if (!trackingQueue->empty()) {
			cv::Point3f newRobotPose = readTracking(
					boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()), trackingMarkerID);
			if (newRobotPose != cv::Point3f(0, 0, 0)) {
				robotPose = newRobotPose;
			} else {
				setSteering(0, 0, 1);
				continue;
			}
		}

		// choose action dependend on state
		switch (currentState) {
		case FRONTIER: {

			// check if there is an edge
			if ((lineBelow[LEFT] || lineBelow[MIDDLE_LEFT] || lineBelow[MIDDLE_RIGHT] || lineBelow[RIGHT])) {
				setSteering(0, 0, DT);

#ifdef USECAN
				for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::RED));
#endif

				// pause the local planner
				pathInformer->publish(
						boost::shared_ptr<twbTracking::proto::Pose2DList>(new twbTracking::proto::Pose2DList()));

				// save current pose
				edgeApproachPose = robotPose;
				lastPose = robotPose;
				// save a copy of the current map
				map = *mapServer->call<cv::Mat>("getObstacleMap");

				// switch state
				currentState = EDGE;

			} else {

				// normal frontier exploration
				float goaldistance = cv::norm(cv::Point2f(pathEnd.x - robotPose.x, pathEnd.y - robotPose.y));
				float lastDistance = cv::norm(cv::Point2f(lastPose.x - robotPose.x, lastPose.y - robotPose.y));
				if (!pathResponseQueue->empty() || goaldistance < 0.05 || lastDistance > 0.05) {
					if (!pathResponseQueue->empty()) {
						pathResponseQueue->pop();
					}
					try {

						// get a new path
						boost::shared_ptr<twbTracking::proto::Pose2DList> path = mapServer->call<
								twbTracking::proto::Pose2DList>("getFrontierPath");

						if (path->pose_size() > 0) {
							twbTracking::proto::Pose2D pEnd = path->pose(0);
							pathEnd = cv::Point2f(pEnd.x(), pEnd.y());
							// send the path to the local planner

							pathInformer->publish(path);
							lastPose = robotPose;
						} else {

							running = false;
						}
					} catch (const rsc::threading::FutureTimeoutException & e) {
						cerr << "MapGenerator not responding! FrontierExploration shutting down." << endl;
						running = false;
					}
				}
			}
		}
			break;
		case EDGE: {
			// check if edge should be aborted

			// distance between edge start position and current position
			float edgeDistance = cv::norm(
					cv::Point2f(robotPose.x - edgeApproachPose.x, robotPose.y - edgeApproachPose.y));
			float deltaEdge = cv::norm(cv::Point2f(robotPose.x - lastPose.x, robotPose.y - lastPose.y));

			// check if the robot reenters know space
			if (robotPose != cv::Point3f(0, 0, 0)) {
				if (map.at<uchar>(cv::Point2i(robotPose.x / 0.01, robotPose.y / 0.01)) == 128) {
					unknownEdge = true;
				} else if (unknownEdge) {
					currentState = ABORTEDGE;
					continue;
				}

				if (!unknownEdge && edgeDistance > 0.35) {
					currentState = ABORTEDGE;
					continue;
				}
			}

			// check if there is an obstacle in front
			if (!proxQueue->empty()) {
				const boost::shared_ptr<std::vector<int>> proxValues = boost::static_pointer_cast<std::vector<int> >(
						proxQueue->pop());
				if (proxValues->at(3) > 500 || proxValues->at(4) > 500) {
					currentState = ABORTEDGE;
					continue;
				}
			}

			// calculate steering and edge side depending on the sensors
			if (!lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0, 0, DT);
				currentState = ABORTEDGE;
				continue;
			} else if (!lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(0.5 * V, -W, DT);
				edgeSide = RIGHTSIDE;
			} else if (!lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0, W, DT);
				edgeSide = RIGHTSIDE;
			} else if (!lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(V, 0, DT);
				edgeSide = RIGHTSIDE;
			} else if (!lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0, -W, DT);
				edgeSide = LEFTSIDE;
			} else if (!lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(0, W, DT);
			} else if (!lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0, floorProxValuesNormalized[MIDDLE_LEFT] > floorProxValuesNormalized[MIDDLE_LEFT] ? W : -W,
						DT);
			} else if (!lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(0.3 * V, W, DT);
				edgeSide = RIGHTSIDE;
			} else if (lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0.5 * V, W, DT);
				edgeSide = LEFTSIDE;
			} else if (lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(0, floorProxValuesNormalized[LEFT] > floorProxValuesNormalized[RIGHT] ? W : -W, DT);
			} else if (lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0, -W, DT);
			} else if (lineBelow[LEFT] && !lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(V, 0, DT);
			} else if (lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(V, 0, DT);
				edgeSide = LEFTSIDE;
			} else if (lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && !lineBelow[MIDDLE_RIGHT] && lineBelow[RIGHT]) {
				setSteering(V, 0, DT);
			} else if (lineBelow[LEFT] && lineBelow[MIDDLE_LEFT] && lineBelow[MIDDLE_RIGHT] && !lineBelow[RIGHT]) {
				setSteering(0.3 * V, -W, DT);
				edgeSide = LEFTSIDE;
			} else {
				setSteering(0, floorProxValuesNormalized[LEFT] > floorProxValuesNormalized[RIGHT] ? W : -W, DT);
			}

			// publish information to include the edge in the map
			if (edgeDistance > 0.05 && edgeSide != UNKNOWN && deltaEdge > 0.02) {
				lastPose = robotPose;
				float offset = edgeSide == RIGHTSIDE ? -M_PI / 2 : M_PI / 2;
				boost::shared_ptr<twbTracking::proto::Pose2D> edgePose(new twbTracking::proto::Pose2D);
				edgePose->set_x(robotPose.x - 0.05 * cos(robotPose.z));
				edgePose->set_y(robotPose.y - 0.05 * sin(robotPose.z));
				edgePose->set_orientation(robotPose.z + offset);
				edgePoseInformer->publish(edgePose);
				if (!edgeLocked) {
					edgeLocked = true;
					edgePose->set_x(edgeApproachPose.x - 0.05 * cos(robotPose.z));
					edgePose->set_y(edgeApproachPose.y - 0.05 * sin(robotPose.z));
					edgePoseInformer->publish(edgePose);
				}

			}
		}
			break;
		case ABORTEDGE: {
			if (lineBelow[LEFT] || lineBelow[MIDDLE_LEFT] || lineBelow[MIDDLE_RIGHT] || lineBelow[RIGHT]) {
				// turn away from edge till no sensor senses the edge
				setSteering(0, edgeSide == LEFTSIDE ? -W : W, DT);

				if ((lineBelow[LEFT] || lineBelow[MIDDLE_LEFT]) && !lineBelow[MIDDLE_RIGHT]) {
					edgeSide = LEFTSIDE;
				} else if ((lineBelow[RIGHT] || lineBelow[MIDDLE_RIGHT]) && !lineBelow[MIDDLE_LEFT]) {
					edgeSide = RIGHTSIDE;
				}

				float deltaRot = abs(lastPose.z - robotPose.z);
				if (edgeSide != UNKNOWN && deltaRot > M_PI / 18) {
					lastPose = robotPose;
					float offset = edgeSide == RIGHTSIDE ? -M_PI / 2 : M_PI / 2;
					boost::shared_ptr<twbTracking::proto::Pose2D> edgePose(new twbTracking::proto::Pose2D);
					edgePose->set_x(robotPose.x - 0.05 * cos(robotPose.z));
					edgePose->set_y(robotPose.y - 0.05 * sin(robotPose.z));
					edgePose->set_orientation(robotPose.z + offset);
					edgePoseInformer->publish(edgePose);
				}
			} else {
				setSteering(0, 0, 0);
				edgeSide = UNKNOWN;
				unknownEdge = false;
				edgeLocked = false;
				lastPose = robotPose;
				// clear pathresponsequeue
				if (!pathResponseQueue->empty()) {
					pathResponseQueue->pop();
				}
				// get a new path
				boost::shared_ptr<twbTracking::proto::Pose2DList> path =
						mapServer->call<twbTracking::proto::Pose2DList>("getFrontierPath");

				if (path->pose_size() > 0) {
					// send the path to the local planner
					pathInformer->publish(path);
				} else {
					cout << "Exploration finished" << endl;
					running = false;
				}
#ifdef USECAN
				for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::BLUE));
#endif
				// switch to frontier exploration
				currentState = FRONTIER;
			}
		}
			break;
		}
	}

#ifdef USECAN
	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
#endif

	// pause the local planner
	pathInformer->publish(boost::shared_ptr<twbTracking::proto::Pose2DList>(new twbTracking::proto::Pose2DList()));

	// send rsb signal to state that the exploration is finished
	boost::shared_ptr<string> finishedResponse(new string("finish"));
	finishedInformer->publish(finishedResponse);

	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
	return EXIT_SUCCESS;
}
