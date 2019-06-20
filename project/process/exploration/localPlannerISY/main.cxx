//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Local planner used for the isy-project 2014/2015
//============================================================================

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
using namespace rsb;

// converters
#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

// types
#include <types/twbTracking.pb.h>

// project includes
#include "localPlanner.hpp"
#include "extspread.hpp"

// camera parameter
float meterPerPixel = 1.0 / 400.0;

// get information from tracking
cv::Point3f readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID);

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string trackingInscope = "/murox/roboterlocation";
	std::string odometryInscope = "/odometrydata";
	std::string steeringOutScope = "/motor/05";
	std::string pathInScope = "/path";
	std::string pathResponseOutScope = "/pathResponse";

	std::string spreadhost = "127.0.0.1";
	std::string spreadport = "4803";

	// id of the tracking marker
	int trackingMarkerID = 0;

	// frequency of sending the map
	int mapFreq = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("id", po::value<int>(&trackingMarkerID),
			"ID of the tracking marker")("odoin", po::value<std::string>(&odometryInscope), "Inscope for odometry data")(
			"trackingin", po::value<std::string>(&trackingInscope), "Inscope for tracking pose")("steeringOut",
			po::value<std::string>(&steeringOutScope), "Outscope for steering commands.")("pathIn",
			po::value<std::string>(&pathInScope), "Inscope for the path (default: /path).")("pathRe",
			po::value<std::string>(&pathInScope), "Outscope for path responses (default: /pathResponse).")("host",
			po::value<std::string>(&spreadhost), "Host for Programatik Spread.")("port",
			po::value<std::string>(&spreadport), "Port for Programatik Spread.")("meterPerPixel,mpp",po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel");

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

	std::string pathOutscope = "/pathUpdate/"+boost::lexical_cast<std::string>(trackingMarkerID);

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();


	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ---------- Listener ---------------

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

	// prepare RSB listener for paths
	rsb::ListenerPtr pathListener = factory.createListener(pathInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>pathQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	pathListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(pathQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish steering commands
	rsb::Informer<std::vector<int>>::Ptr steeringInformer = factory.createInformer<std::vector<int>>(steeringOutScope);

	// create rsb informer to publish path responses
	rsb::Informer<bool>::Ptr pathResponseInformer = factory.createInformer<bool>(pathResponseOutScope);

	// create rsb informer to publish path updates
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathUpdateInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutscope, extspreadconfig);

	// initialize the robots pose
	cv::Point3f robotPose(0, 0, 0);
	cv::Point3f lastRobotPose = robotPose;

	// object to drive exploration paths
	LocalPlanner localPlanner(steeringInformer, pathUpdateInformer);

	bool pathInProgress = false;

	while (true) {

		try {
			// get the position from tracking
			robotPose = readTracking(boost::static_pointer_cast<twbTracking::proto::Pose2DList>(trackingQueue->pop(1000)),
				trackingMarkerID);
		} catch (const rsc::threading::QueueEmptyException & e) {
			robotPose = cv::Point3f(0,0,0);
		}

		// if tracking fails stop the robot
		if (robotPose == cv::Point3f(0,0,0)) {
			localPlanner.stopRobot();
			continue;
		}

		// update the path planner
		PATH_STATUS pathStatus = localPlanner.updatePose(robotPose);

		// if the path was finished or if an error occured publish this information
		if (pathInProgress) {
			switch (pathStatus) {
			case PATH_FINISHED: {
				boost::shared_ptr<bool> pathResponse(new bool(true));
				pathResponseInformer->publish(pathResponse);
				pathInProgress = false;
			}
				break;
			case PATH_ERROR: {
				boost::shared_ptr<bool> pathResponse(new bool(false));
				pathResponseInformer->publish(pathResponse);
				pathInProgress = false;
			}
				break;
			default: {
			}
				break;
			}
		}

		// read new path
		if (!pathQueue->empty()) {
			localPlanner.setPath(*pathQueue->pop());
			pathInProgress = true;
		}

	}

	return EXIT_SUCCESS;
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

