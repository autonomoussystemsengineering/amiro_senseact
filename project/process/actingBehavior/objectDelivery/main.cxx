// std includes
#include <iostream>
using namespace std;

// opencv
#include <opencv2/core/core.hpp>

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
#include <rsc/threading/SynchronizedQueue.h>

using namespace rsb;
using namespace rsb::patterns;

// converters
//#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>
#include <extspread.hpp>
#include <ControllerAreaNetwork.h>
ControllerAreaNetwork myCAN;

// keyboard
#include <kbhit.hpp>

// camera parameter
float meterPerPixel = 1.0 / 400.0;
const float cellSize = 0.01;

float objectRadius = 0;
int pathIdx = -2;
bool debug = false;

bool useSLAMData = false;
bool mapServerIsRemote = false;
bool isBigMap = false;
bool isPushing = false;

boost::shared_ptr<twbTracking::proto::Pose2DList> pathToStartPosTmp(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pathToStartPos(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingPath(new twbTracking::proto::Pose2DList);
cv::Point2f destObjectPos(164, 52), nextDestObjectPos = destObjectPos, currObjectPos, startOwnPos, destOwnPos;
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer;

//debug
boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList4PathRequest(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingPath4Drawing(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> path4Drawing(new twbTracking::proto::Pose2DList);
boost::shared_ptr<twbTracking::proto::Pose2DList> pushingArrow4Drawing(new twbTracking::proto::Pose2DList);

rsb::Informer<std::string>::Ptr stateMachineInformer;

rsb::Informer<twbTracking::proto::Pose2DList>::Ptr drawObjectsInformer, drawPathInformer, drawArrowsInformer;
boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr(new twbTracking::proto::Pose2D());
rsb::Informer<twbTracking::proto::Pose2D>::Ptr insertObjectInformer, deleteObjectInformer;
cv::Point3i color_blue(255,0,0), color_red(0,0,255), color_green(0,255,0), color_orange(36,127,255);
twbTracking::proto::Pose2D object;
cv::Point2f currOwnPos;
RemoteServerPtr mapServer;

boost::shared_ptr<std::string> finishStr(new string("finish"));

rsb::Informer<twbTracking::proto::Pose2D>::Ptr pathRequestInformer;
boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>pathRequestQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));


boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));

// for motor control
//rsb::Informer< std::vector<int> >::Ptr motorCmdInformer;
std::vector<int> motorCmd(3,0);

// search the pose-list received from tracking for the robots id and get the corresponding pose.
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID) {
	twbTracking::proto::Pose2D pose;
	for (int i = 0; i < data->pose_size(); i++) {
		if (trackingMarkerID == data->pose(i).id()) {
			pose = data->pose(i);
			break;
		}
	}
	return cv::Point3f(pose.x() * meterPerPixel, pose.y() * meterPerPixel, pose.orientation() * M_PI / 180.0);
}

cv::Point3f getSLAMPosition(rst::geometry::PoseEuler odomInput) {
	// Save data
	return cv::Point3f(odomInput.mutable_translation()->x(),
			odomInput.mutable_translation()->y(),
			odomInput.mutable_rotation()->yaw());
}

void addPos2PoseList(boost::shared_ptr<twbTracking::proto::Pose2DList> &pose2DList, cv::Point2f pos2D, float orientation=0) {
	twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
	pose2D->set_x(pos2D.x);
	pose2D->set_y(pos2D.y);
	pose2D->set_orientation(orientation);
	pose2D->set_id(0);
}

void calcAndPublishNextPathSegment(boost::shared_ptr<bool> pathResponse) {
	cout << "ObjectDelivery: Steps left: " << pathIdx << endl;
	if (!pathResponse) {
		cout << "Path error!" << endl;
		return;
	}

	if (pathIdx < pushingPath->pose_size()-1 && pathIdx >= 0) {
		motorCmd[0] = -40000;
		motorCmd[1] = 0;
		motorCmd[2] = 2000000;
		boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
//	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
//	motorCmdInformer->publish(motorCmdData);
		myCAN.setTargetSpeed(motorCmd[0], motorCmd[1]);
		if (pathIdx == 0) {
			usleep(5000000);
		} else {
			usleep(3000000);
		}
		myCAN.setTargetSpeed(0, 0);
	}
	if (pathIdx<0 || (isPushing && pathIdx==0)) {
		for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
		cout << "Pushing finished!" << endl;
		stateMachineInformer->publish(finishStr);
		pathIdx=-2;
		isPushing = false;
		return;
	}

	cout << "ObjectDelivery: Length Pushing Path List: " << pushingPath->pose_size() << endl;
	currObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x(),pushingPath->pose(pathIdx).y());
	pathIdx--;
	nextDestObjectPos = cv::Point2f(pushingPath->pose(pathIdx).x(),pushingPath->pose(pathIdx).y());
	cv::Point2f objectShiftVector = nextDestObjectPos - currObjectPos;
	cv::Point2f objectRobotVector = objectShiftVector * (objectRadius/cv::norm(objectShiftVector));
	startOwnPos = currObjectPos - objectRobotVector;
	destOwnPos = nextDestObjectPos - objectRobotVector*0.5f;

/*	if(debug){
		pushingArrow4Drawing->Clear();
		addPos2PoseList(pushingArrow4Drawing, startOwnPos, 3);
		addPos2PoseList(pushingArrow4Drawing, destOwnPos, 3);
		addPos2PoseList(pushingArrow4Drawing, cv::Point2f(color_red.x, color_red.y), color_red.z);
	}*/

	cout << "ObjectDelivery: Starting position calculated. Retrieving path thereto." << endl;

	boost::shared_ptr<twbTracking::proto::Pose2D> startOwnPose2D(new twbTracking::proto::Pose2D());
	startOwnPose2D->set_x(startOwnPos.x);
	startOwnPose2D->set_y(startOwnPos.y);
	startOwnPose2D->set_orientation(0);
	startOwnPose2D->set_id(0);

	insertObjectInformer->publish(objectPtr);

//	try {
		if (isBigMap) {
			if (!pathRequestQueue->empty()) {
				pathRequestQueue->pop();
			}
			pathRequestInformer->publish(startOwnPose2D);
			cout << "ObjectDelivery: Waiting for path answer ..." << endl;
			while (pathRequestQueue->empty()) {
				cout << "ObjectDelivery: Still waiting ..." << endl;
				sleep(1);
			}
			cout << "ObjectDelivery: Planner for path seems to be ready." << endl;
			pathRequestQueue->pop();
		}
		pathToStartPosTmp = mapServer->call<twbTracking::proto::Pose2DList>("path", startOwnPose2D);
		cout << "Try Block finished." << endl;
//	} catch (const rsc::threading::FutureTimeoutException & e) {
//		cerr << "ObjectDelivery: MapGenerator not responding when trying to retrieve path to start position! CleanUpTask shutting down." << endl;
//		return;
//	}

	pathToStartPos->Clear();
	addPos2PoseList(pathToStartPos, destOwnPos, 0);
	for(int i = 0; i < pathToStartPosTmp->pose_size(); ++i) {
		addPos2PoseList(pathToStartPos, cv::Point2f(pathToStartPosTmp->pose(i).x(),pathToStartPosTmp->pose(i).y()), 0);
	}

	deleteObjectInformer->publish(objectPtr);

	cout << "ObjectDelivery: Path to starting position retrieved (length: " << pathToStartPos->pose_size() << ")" << endl;

	cout << "ObjectDelivery: Pushing Path:" << endl;
	for (int idx=0; idx<pathToStartPos->pose_size(); idx++) {
		cout << "ObjectDelivery:  - " << pathToStartPos->pose(idx).x() << "/" << pathToStartPos->pose(idx).y() << endl;
	}

	if(debug){
		path4Drawing->Clear();
		path4Drawing->CopyFrom(*pathToStartPosTmp);
		addPos2PoseList(path4Drawing, currOwnPos, 3);
		addPos2PoseList(path4Drawing, cv::Point2f(color_orange.x, color_orange.y), color_orange.z);
		//currOwnPos = destOwnPos;
	}

	pathInformer->publish(pathToStartPos);
	cout << "ObjectDelivery: Path to starting position published." << endl;

	for(int led=0; led<8; led++) myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
}

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string trackingInscope = "/murox/roboterlocation";
	std::string odometryInscope = "/localization";
	std::string pathOutScope = "/path";
	std::string mapServerScope = "/mapGenerator";
	std::string pathResponseInScope = "/pathResponse";
	std::string rsbMotoOutScope = "/motor/03";
	std::string stateMachineCmdScope = "/objectDelivery/command";
	std::string stateMachineAnswerScope = "/objectDelivery/answer";
	std::string insertObjectOutscope = "/mapGenerator/insertObject";
	std::string deleteObjectOutscope = "/mapGenerator/deleteObject";
	std::string drawObjectsOutscope = "/draw/objects";
	std::string drawPointsOutscope = "/draw/points";
	std::string drawPathOutscope = "/draw/path";
	std::string drawArrowsOutscope = "/draw/arrows";
	std::string sPathRequestOutput = "/pathReq/request";
	std::string sPathRequestInput = "/pathReq/answer";
	std::string sPushingPathRequestOutput = "/pushPathServer/request";
	std::string sPushingPathRequestInput = "/pushPathServer/answer";

	std::string spreadhost = "localhost";
	std::string spreadport = "4823";

	// id of the tracking marker
	int trackingMarkerID = 0;
	int destinationMarkerID = 4;
	int robotID = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()
			("help,h", "Display a help message.")
			("robotID", po::value <int> (&robotID), "ID of robot for communication (default=0)")
			("id", po::value<int>(&trackingMarkerID), "ID of the tracking marker")
			("destId", po::value<int>(&destinationMarkerID), "ID of the destination marker")
			("positionInscope", po::value <std::string> (&odometryInscope), "Inscope for position data of SLAM localization.")
//			("edgeout", po::value<std::string>(&edgeOutscope), "Outscope for edge found signals")
			("pathRe", po::value<std::string>(&pathResponseInScope), "Inscope for path responses.")
			("pathOut", po::value<std::string>(&pathOutScope), "Outscope for the robots path.")
			("mapServer", po::value<std::string>(&mapServerScope), "Scope for the mapGenerator server")
			("host", po::value<std::string>(&spreadhost), "Host for Programatik Spread.")
			("port", po::value<std::string>(&spreadport), "Port for Programatik Spread.")
			("meterPerPixel,mpp", po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel")
			("debugVis", "Debug mode: object selection via keyboard & visualization")
			("bigMap", "Flag if the map is very big (the pathplanner needs more than 25 seconds).")
			("mapServerIsRemote", "Flag, if the map server is a remote server (otherwise it uses local connection).")
			("useSLAMData", "Use SLAM Localization Data (PoseEuler) instead of Tracking Data for incomming position data.");

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

	debug = vm.count("debugVis") > 0;
	if(debug) cout << "ObjectDelivery: Debug mode: object selection via keyboard (number as Id). Visualization data will be published for showMap." << endl;
	if (vm.count("robotID")) {
		mapServerScope.append(std::to_string(robotID));
	}

	mapServerIsRemote = vm.count("mapServerIsRemote");
	useSLAMData = vm.count("useSLAMData");
        isBigMap = vm.count("bigMap");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

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
          tmpPropSpread["host"] = boost::any(std::string(spreadhost));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string(spreadport));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////

	// ------------ Converters ----------------------

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// register converter for cv::Mat
	boost::shared_ptr<MatConverter> matConverter(new MatConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);

/*	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);*/

	// Register converter for PoseEuler
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
	rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);

	// ---------- Listener ---------------

	// prepare RSB listener for path request answer
	rsb::ListenerPtr pathRequestListener = factory.createListener(sPathRequestInput);
//	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>pathRequestQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	pathRequestListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(pathRequestQueue)));

	// prepare RSB listener for pushing path request answer
	rsb::ListenerPtr pushingPathRequestListener = factory.createListener(sPushingPathRequestInput);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>pushingPathRequestQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	pushingPathRequestListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(pushingPathRequestQueue)));

	// prepare RSB listener for path responses
/*	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));*/

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// Prepare RSB async listener for localization messages
	rsb::ListenerPtr odomListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>>odomQueue(new rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>(1));
	odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::PoseEuler>(odomQueue)));

	// prepare RSB listener for localPlanner responses
	rsb::ListenerPtr pathPlannerListener = factory.createListener(pathResponseInScope);
	pathPlannerListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<bool>(&calcAndPublishNextPathSegment)));

	rsb::ListenerPtr stateMachineListener = factory.createListener(stateMachineCmdScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> > > stateMachineCmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> >(1));
	stateMachineListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(stateMachineCmdQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// create rsb informer to publish the robots path request
	pathRequestInformer = factory.createInformer<twbTracking::proto::Pose2D>(sPathRequestOutput);

	// create rsb informer to publish the robots pushing path request
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pushingPathRequestInformer = factory.createInformer<twbTracking::proto::Pose2DList>(sPushingPathRequestOutput);

	// Prepare RSB informer
//	motorCmdInformer = factory.createInformer< std::vector<int> > (rsbMotoOutScope);

	if(debug){
		drawObjectsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawObjectsOutscope, extspreadconfig);
		//	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr  drawPointsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawPointsOutscope);
		drawPathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawPathOutscope, extspreadconfig);
		drawArrowsInformer = factory.createInformer<twbTracking::proto::Pose2DList>(drawArrowsOutscope, extspreadconfig);
	}

	boost::shared_ptr<twbTracking::proto::Pose2DList> objectsList;
	stateMachineInformer = factory.createInformer< std::string > (stateMachineAnswerScope);
	insertObjectInformer = factory.createInformer<twbTracking::proto::Pose2D>(insertObjectOutscope);
	deleteObjectInformer = factory.createInformer<twbTracking::proto::Pose2D>(deleteObjectOutscope);

	// map server
//	RemoteServerPtr mapServer;
	if (mapServerIsRemote) {
		mapServer = factory.createRemoteServer(mapServerScope, tmpPartConf, tmpPartConf);
	} else {
		mapServer = factory.createRemoteServer(mapServerScope);
	}

	// constants
	boost::shared_ptr<bool> truePtr(new bool(true));

	if(debug){
		destObjectPos = cv::Point2f(135,139)*cellSize;
		try {
			objectsList = mapServer->call<twbTracking::proto::Pose2DList>("getObjectsList");
		} catch (const rsc::threading::FutureTimeoutException & e) {
			cerr << "ObjectDelivery: ERROR - MapGenerator not responding when trying to retrieve objects list! Exiting deliverObject." << endl;
			return EXIT_FAILURE;
		}
		addPos2PoseList(objectsList, cv::Point2f(color_green.x, color_green.y), color_green.z);
		cout << "ObjectDelivery: Objects list retrieved: " << objectsList->pose_size() << " objects found." << endl;
	} else {
		cout << "ObjectDelivery: Waiting for destination position." << endl;
		cv::Point3f destObjectPose;
		if (useSLAMData) {
			while(odomQueue->empty()) usleep(250000);
			destObjectPose = getSLAMPosition(*odomQueue->pop());
			destObjectPose.x += 0.05;
		} else {
			while(trackingQueue->empty()) usleep(250000);
			destObjectPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),destinationMarkerID);
		}
		destObjectPos = cv::Point2f(destObjectPose.x, destObjectPose.y);
		cout << "ObjectDelivery: Destination position set on " << destObjectPos.x << "/" << destObjectPos.y << endl;
	}

	while(true) {

		cout << "ObjectDelivery: Waiting for next object request ..." << endl;

		while((!debug && stateMachineCmdQueue->empty()) || (debug && !kbhit())){
			if(debug){
				drawObjectsInformer->publish(objectsList);
				drawPathInformer->publish(pushingPath4Drawing);
				drawPathInformer->publish(path4Drawing);
				drawArrowsInformer->publish(pushingArrow4Drawing);
			}
			usleep(125000);
			if (pathIdx == -1) {
				stateMachineInformer->publish(finishStr);
				pathIdx = -2;
			}
		}
		if(debug){
			// select object via keyboard (number as Id)
			char c;
			cin >> c;
			object = objectsList->pose(c-'0');
		} else object = *stateMachineCmdQueue->pop();

		objectRadius = object.orientation();
		currObjectPos = cv::Point2f(object.x(), object.y());

		objectPtr->set_x(object.x());
		objectPtr->set_y(object.y());
		objectPtr->set_orientation(object.orientation());
		objectPtr->set_id(0);

		cout << "ObjectDelivery: Deliver command received. Start delivering object ..." << endl;

		if(debug){
			currOwnPos = cv::Point2f(129,57)*cellSize;
		} else {
			cout << "ObjectDelivery: Waiting for position of robot ..." << endl;
			cv::Point3f robotPose;
			if (useSLAMData) {
				while(odomQueue->empty()) usleep(250000);
				robotPose = getSLAMPosition(*odomQueue->pop());
			} else {
				while(trackingQueue->empty()) usleep(250000);
				robotPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop()),destinationMarkerID);
			}
			currOwnPos = cv::Point2f(robotPose.x, robotPose.y);
		}

		pose2DList4PathRequest->Clear();
		addPos2PoseList(pose2DList4PathRequest, currObjectPos, 4);
		addPos2PoseList(pose2DList4PathRequest, destObjectPos, 4);
		*pose2DList4PathRequest->add_pose() = object;

		try {
			cout << "ObjectDelivery: Deliver object from " << currObjectPos.x << "/" << currObjectPos.y << " to position " << destObjectPos.x << "/" << destObjectPos.y << endl;

			if (isBigMap) {
				if (!pushingPathRequestQueue->empty()) {
					pushingPathRequestQueue->pop();
				}
				pushingPathRequestInformer->publish(pose2DList4PathRequest);
				cout << "ObjectDelivery: Waiting for pushing path answer ..." << endl;
				while (pushingPathRequestQueue->empty()) {
					cout << "ObjectDelivery: Still waiting ..." << endl;
					sleep(1);
				}
				cout << "ObjectDelivery: Planner for pushing path seems to be ready." << endl;
				pushingPathRequestQueue->pop();
			}
			pushingPath = mapServer->call<twbTracking::proto::Pose2DList>("getPushingPath", pose2DList4PathRequest);
		} catch (const rsc::threading::FutureTimeoutException & e) {
			cerr << "ObjectDelivery: ERROR - Map Server not responding when trying to retrieve pushing path! Exiting deliverObject." << endl;
			return EXIT_FAILURE;
		}
		addPos2PoseList(pushingPath, currObjectPos, 0);

		if(debug){
			cout << "ObjectDelivery: Pushing path retrieved (length: " << pushingPath->pose_size() << ")." << endl;
			pushingPath4Drawing->CopyFrom(*pushingPath);
			addPos2PoseList(pushingPath4Drawing, cv::Point2f(color_blue.x, color_blue.y), color_blue.z);
		}

		if (pushingPath->pose_size() > 1) {
			cout << "ObjectDelivery: Object Path:" << endl;
			for (int idx=0; idx<pushingPath->pose_size(); idx++) {
				cout << "ObjectDelivery:  - " << pushingPath->pose(idx).x() << "/" << pushingPath->pose(idx).y() << endl;
			}
			pathIdx = pushingPath->pose_size()-1;
			isPushing = true;
			calcAndPublishNextPathSegment(truePtr);
		} else cout << "ObjectDelivery: ERROR - No pushing path found!" << endl;
	}

	return EXIT_SUCCESS;
}
