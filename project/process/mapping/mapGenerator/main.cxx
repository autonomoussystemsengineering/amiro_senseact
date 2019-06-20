//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Map generation and path planning.
//============================================================================

// std includes
#include <iostream>
using namespace std;

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
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
#include <converter/mapUpdateConverter/mapUpdateConverter.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include "mapUpdate.hpp"

// project includes
#include "mapGenerator.hpp"
#include "pathPlanner.hpp"
#include "extspread.hpp"

// size of the map
const cv::Size mapSize = cv::Size(600, 600);

// enable repell effect
bool repel = false;

// cellSize
const float cellSize = 0.01;

// micrometer to meter
const float YM_2_M = 1000000.0;

// camera parameter
float meterPerPixel = 1.0 / 400.0;

// radius of the robot
float robotRadius = 0.05;

// obect used to update the map from sensorvalues
MapGenerator mapGenerator(cellSize);

// object that calculates paths
PathPlanner pathPlanner(cellSize);

// initialize the maps with 0
cv::Mat gridmap = cv::Mat::zeros(mapSize, CV_8SC1);
cv::Mat edgeMap(mapSize, CV_8SC1, Scalar(0));
cv::Mat combinedMap(mapSize, CV_8SC1, Scalar(0));

// the robots pose
cv::Point3f robotPose(0, 0, 0);
std::vector<cv::Point3f> otherPoses;

// callBack for the map
class MapCallback: public LocalServer::Callback<void, cv::Mat> {
	boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
		boost::shared_ptr<cv::Mat> frame(new cv::Mat(combinedMap));
		return frame;
	}
};

// callBack for the obstacleMap
class ObstacleMapCallback: public LocalServer::Callback<void, cv::Mat> {
	boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {

		Mat obstacleMap;
		combinedMap = gridmap + edgeMap + edgeMap;
		mapGenerator.generateObstacleMap(combinedMap, obstacleMap);
		boost::shared_ptr<cv::Mat> frame(new cv::Mat(obstacleMap));
		return frame;

	}
};

// callBack for frontier path
class frontierPathCallback: public LocalServer::Callback<void, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/) {

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(combinedMap, obstacleMap);

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToFrontier(obstacleMap, robotPose, otherPoses);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}

		return pose2DList;
	}
};

// callBack for path to point
class pathCallback: public LocalServer::Callback<twbTracking::proto::Pose2D, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
			boost::shared_ptr<twbTracking::proto::Pose2D> pose) {

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(combinedMap, obstacleMap);

		// convert pose to cv::point2f
		// note: 3. coordinate is ignored
		Point2f target(pose.get()->x(), pose.get()->y());

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToTarget(obstacleMap, robotPose, target);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		return pose2DList;
	}
};

// callBack for path to point
class objectsCallback: public LocalServer::Callback<void, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/) {

		vector<vector<cv::Point2i> > contours;
		cv::Mat map, mask, thresholded;

		mapGenerator.generateObstacleMap(combinedMap, map);

		// Get area inside of walls
		cv::threshold(map, thresholded, 250, 255, cv::THRESH_BINARY);
		thresholded.copyTo(mask);
		cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		mask = Mat::zeros(map.size(), CV_8UC1);
		cv::drawContours(mask, contours, -1, cv::Scalar(255), -1);
		cv::Mat help = Mat::ones(map.size(), CV_8UC1) * 255;
		thresholded.copyTo(help, mask);

		// Switch black & white
		cv::Mat objects = Mat::ones(map.size(), CV_8UC1) * 255;
		cv::subtract(objects, help, objects);

		// Find objects
		cv::findContours(objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		int numObjects = contours.size();
		cv::Point2f centers[numObjects];
		float objectRadius[numObjects];
		cv::Moments moments;
		boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList(new twbTracking::proto::Pose2DList);

		for (int i = 0; i < numObjects; ++i) {
			if (cv::contourArea(contours[i]) < 5)
				continue;

			// Calculate objects center of gravity
			moments = cv::moments(contours[i], true);
			centers[i] = Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

			// Get objects radius
			cv::Point2f center;
			cv::minEnclosingCircle(contours[i], center, objectRadius[i]);

			// Add object as pose
			twbTracking::proto::Pose2D *pose2D1 = pose2DList->add_pose();
			pose2D1->set_x(centers[i].x * cellSize);
			pose2D1->set_y(centers[i].y * cellSize);
			pose2D1->set_orientation(objectRadius[i] * cellSize);
			pose2D1->set_id(0);
		}
		return pose2DList;
	}
};

// callBack for path to point
class pushingPathCallback: public LocalServer::Callback<twbTracking::proto::Pose2DList, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
			boost::shared_ptr<twbTracking::proto::Pose2DList> inputPointList) {

		// convert pose to cv::point2f
		Point3f startPos(inputPointList->pose(0).x(), inputPointList->pose(0).y(), 0);
		Point2f target(inputPointList->pose(1).x(), inputPointList->pose(1).y());

		cv::Point2f center(inputPointList->pose(2).x() / cellSize, inputPointList->pose(2).y() / cellSize);
		cv::circle(combinedMap, center, inputPointList->pose(2).orientation() / cellSize, cv::Scalar(255), -1);

		// generate the obstacle map
		Mat obstacleMap;
		mapGenerator.generateObstacleMap(combinedMap, obstacleMap);

		// calculate a path
		std::vector<cv::Point2f> path = pathPlanner.getPathToTarget(obstacleMap, startPos, target);

		// convert that path to a pose2DList
		rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
		for (Point2f p : path) {
			twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
			pose2D->set_x(p.x);
			pose2D->set_y(p.y);
			pose2D->set_orientation(0);
			pose2D->set_id(0);
		}
		return pose2DList;
	}
};

void insertObject(boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr) {
	cv::circle(gridmap, cv::Point2f(objectPtr->x() / cellSize, objectPtr->y() / cellSize),
			(objectPtr->orientation() - robotRadius) / cellSize, cv::Scalar(-255), -1);
	combinedMap = gridmap + edgeMap + edgeMap;
}

void deleteObject(boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr) {
	cv::circle(gridmap, cv::Point2f(objectPtr->x() / cellSize, objectPtr->y() / cellSize),
			(objectPtr->orientation() - robotRadius) / cellSize, cv::Scalar(255), -1);
	combinedMap = gridmap + edgeMap + edgeMap;
}

// save the map to a file
void saveMap(rsb::EventPtr event) {
	boost::shared_ptr<string> fileName = static_pointer_cast<string>(event->getData());
	cv::Mat saveMap;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	gridmap.convertTo(saveMap, CV_16SC1);
	saveMap += 128;
	saveMap.convertTo(saveMap, CV_8UC1);
	cv::imwrite(*fileName, saveMap, compression_params);
}

void saveEdgeMap(rsb::EventPtr event) {
	boost::shared_ptr<string> fileName = static_pointer_cast<string>(event->getData());
	cv::Mat saveMap;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	edgeMap.convertTo(saveMap, CV_16SC1);
	saveMap += 128;
	saveMap.convertTo(saveMap, CV_8UC1);
	cv::imwrite(*fileName, saveMap, compression_params);
}

// get information from tracking
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID);

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string proxSensorInscope = "/rir_prox";
	std::string trackingInscope = "/murox/roboterlocation";
	std::string odometryInscope = "/odometrydata";
	std::string edgeInscope = "/edge";
	std::string edgePoseInscope = "/edgePose";
	std::string enableInscope = "/enableMapGenerator";
	std::string occupancyGridOutscope = "/maps/ogm";
	std::string mapUpdateScope = "/mapUpdate";
	std::string serverScope = "/mapGenerator";
	std::string saveMapInscope = "/saveMap";
	std::string saveEdgeMapInscope = "/saveEdgeMap";
	std::string insertObjectInscope = "/mapGenerator/insertObject";
	std::string deleteObjectInscope = "/mapGenerator/deleteObject";
	std::string mapFileName = "";
	std::string edgeMapFileName = "";

	std::string spreadhost = "127.0.0.1";
	std::string spreadport = "4803";

	// id of the tracking marker
	int trackingMarkerID = 0;

	// frequency of sending the map
	int mapFreq = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("verbose,v", "Print the robots pose and sensorvalues.")(
			"single,s", "Single robot mapbuilding, no map updates will be send.")("repel,r",
			"Enable repel effect for multi exploration.")("id", po::value<int>(&trackingMarkerID),
			"ID of the tracking marker")("freq,f", po::value<int>(&mapFreq),
			"Frequency to publish the map (0 = nothing will be published)")("meterPerPixel,mpp",
			po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel")("loadMap,l",
			po::value<std::string>(&mapFileName), "Load map from file with the given name.")("loadEdgeMap,e",
			po::value<std::string>(&edgeMapFileName), "Load map from file with the given name.")("irin",
			po::value<std::string>(&proxSensorInscope), "Inscope for IR values")("odoin",
			po::value<std::string>(&odometryInscope), "Inscope for odometry data")("trackingin",
			po::value<std::string>(&trackingInscope), "Inscope for tracking pose")("edgein",
			po::value<std::string>(&edgeInscope), "Inscope for edge found signals")("enable",
			po::value<std::string>(&enableInscope), "Inscope for enable signals")("updateinout",
			po::value<std::string>(&mapUpdateScope), "Scope for map updates")("mapout",
			po::value<std::string>(&occupancyGridOutscope), "Outscope for the occupancy grid map")("saveMapScope",
			po::value<std::string>(&saveMapInscope), "Scope for filenames to save the map.")("saveEdgeMapScope",
			po::value<std::string>(&saveEdgeMapInscope), "Scope for filenames to save the edge map.")("serverScope",
			po::value<std::string>(&serverScope), "Scope to setup the server for remote procedure calls.")("host",
			po::value<std::string>(&spreadhost), "Host for Programatik Spread.")("port",
			po::value<std::string>(&spreadport), "Port for Programatik Spread.");

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

	if (vm.count("repel")) {
		repel = true;
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// register converter for MapUpdate
	boost::shared_ptr<MapUpdateConverter> mapUpdateConverter(new MapUpdateConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(mapUpdateConverter);

	// register converter for cv::Mat
	boost::shared_ptr<MatConverter> matConverter(new MatConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);

	// Register new converter for std::vector<int>
	boost::shared_ptr<muroxConverter::vecIntConverter> converterVecInt(new muroxConverter::vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// ---------- Listener ---------------

	// prepare RSB listener for enable signals
	rsb::ListenerPtr enableListener = factory.createListener(enableInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>enableQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	enableListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(enableQueue)));

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// prepare rsb listener for odometry data
	rsb::ListenerPtr odometryListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>odoQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	odometryListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(odoQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare RSB listener for edge found signals
	rsb::ListenerPtr edgeListener = factory.createListener(edgeInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>edgeQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	edgeListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(edgeQueue)));

	// prepare RSB listener for edge poses
	rsb::ListenerPtr edgePoseListener = factory.createListener(edgePoseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>edgePoseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	edgePoseListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(edgePoseQueue)));

	// prepare RSB listener for mapUpdate
	rsb::ListenerPtr mapUpdateListener = factory.createListener(mapUpdateScope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<MapUpdate>>>mapUpdateQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<MapUpdate>>(10));
	mapUpdateListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<MapUpdate>(mapUpdateQueue)));

	// prepare RSB listener for commands to save the map
	rsb::ListenerPtr saveMapListener = factory.createListener(saveMapInscope);
	saveMapListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&saveMap)));

	// prepare RSB listener for commands to save the map
	rsb::ListenerPtr saveEdgeMapListener = factory.createListener(saveEdgeMapInscope);
	saveEdgeMapListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&saveEdgeMap)));

	// prepare RSB listener for commands to insert an object in the map
	rsb::ListenerPtr insertObjectListener = factory.createListener(insertObjectInscope);
	insertObjectListener->addHandler(
			rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&insertObject)));

	// prepare RSB listener for commands to delete an object from the map
	rsb::ListenerPtr deleteObjectListener = factory.createListener(deleteObjectInscope);
	deleteObjectListener->addHandler(
			rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&deleteObject)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish MapUpdates
	rsb::Informer<MapUpdate>::Ptr mapUpdateInformer = factory.createInformer<MapUpdate>(mapUpdateScope,
			extspreadconfig);

	// create rsb informer to publish maps
	rsb::Informer<cv::Mat>::Ptr mapInformer = factory.createInformer<cv::Mat>(occupancyGridOutscope, extspreadconfig);

	// ---------------- Remote Procedure calls ---------------------

	LocalServerPtr server = factory.createLocalServer(serverScope);

	// Register method with name and implementing callback object.
	server->registerMethod("getMap", LocalServer::CallbackPtr(new MapCallback()));
	server->registerMethod("getObstacleMap", LocalServer::CallbackPtr(new ObstacleMapCallback()));
	server->registerMethod("getFrontierPath", LocalServer::CallbackPtr(new frontierPathCallback()));
	server->registerMethod("getPath", LocalServer::CallbackPtr(new pathCallback()));
	server->registerMethod("getObjectsList", LocalServer::CallbackPtr(new objectsCallback()));
	server->registerMethod("getPushingPath", LocalServer::CallbackPtr(new pushingPathCallback()));

	cv::Point3f lastRobotPose = robotPose;

	// load a map
	if (mapFileName != "") {
		cv::Mat loadMap = cv::imread(mapFileName, -1);
		loadMap.convertTo(loadMap, CV_16SC1);
		loadMap -= 128;
		loadMap.convertTo(gridmap, CV_8SC1);
		combinedMap = gridmap + edgeMap + edgeMap;
	}

	if (edgeMapFileName != "") {
		cv::Mat loadMap = cv::imread(edgeMapFileName, -1);
		loadMap.convertTo(loadMap, CV_16SC1);
		loadMap -= 128;
		loadMap.convertTo(edgeMap, CV_8SC1);
		combinedMap = gridmap + edgeMap + edgeMap;
	}

	int count = 0;

	bool enable = true;

	while (true) {
		// receive incoming mapUpdates and apply them
		while (!mapUpdateQueue->empty()) {
			MapUpdate mapUpdate = *mapUpdateQueue->pop().get();
			if (mapUpdate.theta == 0) {
				mapGenerator.updateMap(mapUpdate, edgeMap);
			} else {
				mapGenerator.updateMap(mapUpdate, gridmap);
			}
		}

		// check if enable value has changed
		if (!enableQueue->empty()) {
			enable = *enableQueue->pop();
		}

		// dont produce any mapupdates if disabled
		if (!enable) {
			continue;
		}
		if (vm.count("verbose")) {
			cout << "prox Queue " << !proxQueue->empty() << "tracking " << !trackingQueue->empty() << endl;
		}
		// read sensor values
		if (!proxQueue->empty() && !(trackingQueue->empty() && odoQueue->empty())) {
			const boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int> >(
					proxQueue->pop());

			// check if there is trackingdata otherwise take odometrydata
			if (!trackingQueue->empty()) {

				// get the position from tracking
				robotPose = readTracking(
						boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),
						trackingMarkerID);

				// ignore faulty tracking
				if (robotPose == cv::Point3f(0, 0, 0)) {
					robotPose = lastRobotPose;
					continue;
				}

			} else {
				boost::shared_ptr<twbTracking::proto::Pose2D> odometryPose = boost::static_pointer_cast<
						twbTracking::proto::Pose2D>(odoQueue->pop());
				robotPose.x = gridmap.cols / 2.0 * cellSize + odometryPose->x() / YM_2_M;
				robotPose.y = gridmap.rows / 2.0 * cellSize + odometryPose->y() / YM_2_M;
				robotPose.z = odometryPose->orientation() / YM_2_M;
			}

			// calculate the moved distance/the robots speed
			float dx = lastRobotPose.x - robotPose.x;
			float dy = lastRobotPose.y - robotPose.y;
			float movedDistance = sqrt(dx * dx + dy * dy);
			lastRobotPose = robotPose;

			// print the robots pose and sensorvalues
			if (vm.count("verbose")) {
				std::cout << robotPose.x << " " << robotPose.y << " " << robotPose.z;
				for (int i = 0; i < 8; i++) {
					std::cout << " " << sensorValues->at(i);
				}
				std::cout << std::endl;
			}

			// generate a mapUpdate
			MapUpdate mapUpdate = mapGenerator.createMapUpdate(sensorValues, robotPose, movedDistance);

			if (vm.count("single")) {
				// update the map
				mapGenerator.updateMap(mapUpdate, gridmap);
			} else {
				// publish the mapUpdate
				mapUpdateInformer->publish(boost::shared_ptr<MapUpdate>(new MapUpdate(mapUpdate.clone())));

			}

			combinedMap = gridmap + edgeMap + edgeMap;

			// publish visualization information
			if (mapFreq > 0) {
				if (count < mapFreq) {
					count++;
				} else {

					// publish the current map
					cv::Mat finalMap = gridmap + edgeMap + edgeMap;
					boost::shared_ptr<cv::Mat> frame(new cv::Mat(finalMap));
					mapInformer->publish(frame);

					count = 0;
				}
			}

		}

		// check if there is an edge
		if (!edgeQueue->empty()) {

			MapUpdate edgeUpdate = mapGenerator.createEdgeUpdate(robotPose, *edgeQueue->pop());
			if (vm.count("single")) {
				// update the map
				mapGenerator.updateMap(edgeUpdate, edgeMap);
			} else {
				// publish the mapUpdate
				mapUpdateInformer->publish(boost::shared_ptr<MapUpdate>(new MapUpdate(edgeUpdate.clone())));
			}

			combinedMap = gridmap + edgeMap + edgeMap;
		}

		// check if there is an edge
		if (!edgePoseQueue->empty()) {

			twbTracking::proto::Pose2D edgePose = *edgePoseQueue->pop();
			MapUpdate edgeUpdate = mapGenerator.createEdgePoseUpdate(
					cv::Point3f(edgePose.x(), edgePose.y(), edgePose.orientation()));
			if (vm.count("single")) {
				// update the map
				mapGenerator.updateMap(edgeUpdate, edgeMap);
			} else {
				// publish the mapUpdate
				mapUpdateInformer->publish(boost::shared_ptr<MapUpdate>(new MapUpdate(edgeUpdate.clone())));
			}

			combinedMap = gridmap + edgeMap + edgeMap;
		}
	}

	return EXIT_SUCCESS;
}

// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose, rpose;
	otherPoses.clear();
	pose.set_x(0);
	pose.set_y(0);
	pose.set_orientation(0);
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			rpose = data->pose(i);
		} else if (repel) {
			pose = data->pose(i);
			otherPoses.push_back(
					cv::Point3f(pose.x() * meterPerPixel, pose.y() * meterPerPixel, pose.orientation() * M_PI / 180.0));
		}
	}
	return cv::Point3f(rpose.x() * meterPerPixel, rpose.y() * meterPerPixel, rpose.orientation() * M_PI / 180.0);
}

