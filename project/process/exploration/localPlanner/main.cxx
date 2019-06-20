//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>,
//               mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Central control of the robots map building and exploration.
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

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

// types
#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

// project includes
#include "localPlanner.hpp"

// camera parameter
float meterPerPixel = 1.0 / 400.0;

// get information from tracking
cv::Point3f getPosition(rst::geometry::PoseEuler odomInput);

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string odometryInscope = "/localization";
	std::string pathInScope = "/path";
	std::string pathResponseOutScope = "/pathResponse";

	// id of the tracking marker
	int trackingMarkerID = 0;

	// frequency of sending the map
	int mapFreq = 0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("posin,p", po::value<std::string>(&odometryInscope), "Inscope for position data (default: /localization).")
			("pathIn,i", po::value<std::string>(&pathInScope), "Inscope for the path (default: /path).")
			("pathRe,r", po::value<std::string>(&pathResponseOutScope), "Outscope for path responses (default: /pathResponse).")
			("meterPerPixel,mpp", po::value<float>(&meterPerPixel), "Camera parameter: Meter per Pixel");

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

	// ------------ Converters ----------------------

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	// Register converter for PoseEuler
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
	rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);

	// ---------- Listener ---------------

	// prepare rsb listener for position data
	rsb::ListenerPtr odometryListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>>odoQueue(new rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>(1));
	odometryListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::PoseEuler>(odoQueue)));

	// prepare RSB listener for paths
	rsb::ListenerPtr pathListener = factory.createListener(pathInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>pathQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	pathListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(pathQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish path responses
	rsb::Informer<bool>::Ptr pathResponseInformer = factory.createInformer<bool>(pathResponseOutScope);

	// initialize the robots pose

	cv::Point3f robotPose(0, 0, 0);
	cv::Point3f lastRobotPose = robotPose;

	// object to drive exploration paths
	LocalPlanner localPlanner;

	bool pathInProgress = false;

	while (true) {

		// if the path was finished or if an error occured publish this information
		if (pathInProgress) {

			// get robot position
			robotPose = getPosition(*odoQueue->pop());

			// update the path planner
			PATH_STATUS pathStatus = localPlanner.updatePose(robotPose);

			DEBUG_MSG("Check Local Planner Status with actual position " << robotPose.x << "/" << robotPose.y << ", " << robotPose.z << " rad");
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
		} else {
			DEBUG_MSG("Waiting for path.");
			usleep(500000);
		}

		// read new path
		if (!pathQueue->empty()) {
			DEBUG_MSG("Path received.");
			localPlanner.setPath(*pathQueue->pop());
			pathInProgress = true;
		}

	}

	return EXIT_SUCCESS;
}

// Read position data
cv::Point3f getPosition(rst::geometry::PoseEuler odomInput) {
	// Save data
	return cv::Point3f(odomInput.mutable_translation()->x(), odomInput.mutable_translation()->y(), odomInput.mutable_rotation()->yaw());
}

/*
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
*/
