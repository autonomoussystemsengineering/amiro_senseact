//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Guide behavior of the transport scenario.
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

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <ControllerAreaNetwork.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;

float diameter = 0.1;
int trackingMarkerID = 0;
float meterPerPixel = 1.0/400.0;

// scopenames for rsb
std::string trackingInscope = "/murox/roboterlocation";
std::string pathResponseInscope = "/pathResponse";
std::string pathOutScope = "/path";
std::string mapServerScope = "/mapGenerator";
std::string FollowerInscope = "/Transport/Follow";
std::string GuideInscope = "/Transport/Guide";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";


// method prototypes
twbTracking::proto::Pose2D readTracking(boost::shared_ptr<twbTracking::proto::Pose2DList> data, int trackingMarkerID);

/*
 * Algorithm:
 * - Get guide command
 * - Get start position s
 * - Calculate path to start position s
 * - Drive to start position s
 * - Transmit OK to Follower
 * - Get final position
 * - Calculate path to final position
 * - Drive to final position
 * - Transmit OK to Follower
 * - End guide command
 */


int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			     ("id", po::value<int>(&trackingMarkerID), "Tracking ID for AMiRo (default = 0)")
			     ("mpp", po::value<float>(&meterPerPixel), "Meter per pixel (default = 1/400 m/pixel)")
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

	// Register new converter for Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

	// ------------ Listener ----------------------

	// prepare RSB listener for path responses
	rsb::ListenerPtr pathResponseListener = factory.createListener(pathResponseInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>>pathResponseQueue(
			new rsc::threading::SynchronizedQueue<boost::shared_ptr<bool>>(1));
	pathResponseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<bool>(pathResponseQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(trackingInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(trackingQueue)));

	// prepare rsb listener for progress data
        rsb::ListenerPtr progressListener = factory.createListener(FollowerInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>progressQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	progressListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(progressQueue)));

	// ---------------- Informer ---------------------

	// create rsb informer to publish the robots path
	rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathInformer = factory.createInformer<twbTracking::proto::Pose2DList>(pathOutScope);

	// mapGenertor server
	RemoteServerPtr mapServer = factory.createRemoteServer(mapServerScope);

	// create rsb informer to publish progress data
	rsb::Informer<twbTracking::proto::Pose2D>::Ptr progressInformer = factory.createInformer<twbTracking::proto::Pose2D>(GuideInscope, extspreadconfig);





	// Init the CAN interface
	ControllerAreaNetwork myCAN;

        // do algorithm
	boost::shared_ptr<twbTracking::proto::Pose2D> positionPtr(new twbTracking::proto::Pose2D());
	boost::shared_ptr<twbTracking::proto::Pose2D> followerPos(new twbTracking::proto::Pose2D());
	twbTracking::proto::Pose2D ownPos;


	for(int led=0; led<8; led++) {
		myCAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
	}

	/*
	 * at step 0: driving to start position
	 * at step 1: driving to final position
	 */
	for (int step=0; step<2; step++) {

		
/*		for (int i=0; i<5; i++) {
			myCAN.setTargetSpeed(0,0);
			usleep(250000);
		}
*/
		// wait for position
		if (step == 0) {
			INFO_MSG("1) Waiting for start position");
		} else {
			INFO_MSG("4) Waiting for finale position");
		}
		while (progressQueue->empty()) {
			usleep(500000);
		}
		positionPtr = progressQueue->pop();

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
		}

		if (step == 0) {
			followerPos->set_x(0);
			followerPos->set_y(0);
			followerPos->set_orientation(PROTOCOL_OK);
			progressInformer->publish(followerPos);
			INFO_MSG("1.2) Waiting for Follower position");
			while (progressQueue->empty()) {
				usleep(500000);
			}
			followerPos = progressQueue->pop();
		}


		// drive to position
		INFO_MSG((step*3+2) << ") Driving to position " << positionPtr->x() << "/" << positionPtr->y());
/*		positionPtr->set_x(position.x());
		positionPtr->set_y(position.y());
*/		boost::shared_ptr<twbTracking::proto::Pose2DList> path_ori = mapServer->call<twbTracking::proto::Pose2DList>("getPath", positionPtr);
		INFO_MSG("Calculated path (#items: " << path_ori->pose_size() << "):");
		for (int i=0; i<path_ori->pose_size(); i++) {
			INFO_MSG(" -> " << path_ori->pose(i).x() << "/" << path_ori->pose(i).y());
		}

		// create copy of path
		boost::shared_ptr<twbTracking::proto::Pose2DList> path(new twbTracking::proto::Pose2DList);

		// add last step
		twbTracking::proto::Pose2D *pose2D = path->add_pose();
		pose2D->set_x(path_ori->pose(0).x());
		pose2D->set_y(path_ori->pose(0).y());

		// calculate steps around the Follower
		if (step==0) {
			float angleS = atan2(path_ori->pose(0).y()-followerPos->y(), path_ori->pose(0).x()-followerPos->x());
			float angleP;
			if (path_ori->pose_size() < 2) {
				while (trackingQueue->empty()) {
					usleep(250000);
				}
				ownPos = readTracking(trackingQueue->pop(), trackingMarkerID);
				angleP = atan2(ownPos.y()-followerPos->y(), ownPos.x()-followerPos->x());
			} else {
				angleP = atan2(path_ori->pose(1).y()-followerPos->y(), path_ori->pose(1).x()-followerPos->x());
			}
			float angleDiff = angleS-angleP;
			if (angleDiff < 0) {
				angleDiff += 2*M_PI;
			}
			if (angleDiff > M_PI/4.0 && angleDiff < 2*M_PI-M_PI/4.0) {
				int circPos = (int)(2.0/M_PI * angleDiff);
				if (circPos == 1) {
					twbTracking::proto::Pose2D *pose2D = path->add_pose();
					pose2D->set_x((diameter+0.04)*cos(3*M_PI/4.0+angleP) + followerPos->x());
					pose2D->set_y((diameter+0.04)*sin(3*M_PI/4.0+angleP) + followerPos->y());
				} else if (circPos == 2) {
					twbTracking::proto::Pose2D *pose2D = path->add_pose();
					pose2D->set_x((diameter+0.04)*cos(2*M_PI-3*M_PI/4.0+angleP) + followerPos->x());
					pose2D->set_y((diameter+0.04)*sin(2*M_PI-3*M_PI/4.0+angleP) + followerPos->y());
				}				
				if (circPos < 2) {
					twbTracking::proto::Pose2D *pose2D = path->add_pose();
					pose2D->set_x((diameter+0.04)*cos(M_PI/4.0+angleP) + followerPos->x());
					pose2D->set_y((diameter+0.04)*sin(M_PI/4.0+angleP) + followerPos->y());
				} else {
					twbTracking::proto::Pose2D *pose2D = path->add_pose();
					pose2D->set_x((diameter+0.04)*cos(2*M_PI-M_PI/4.0+angleP) + followerPos->x());
					pose2D->set_y((diameter+0.04)*sin(2*M_PI-M_PI/4.0+angleP) + followerPos->y());
				}	
			} else {
				INFO_MSG("No correction of path needed.");
			}
		}

		// copy first path without last step
		for (int idx=1; idx<path_ori->pose_size(); idx++) {
			twbTracking::proto::Pose2D *pose2D = path->add_pose();
			pose2D->set_x(path_ori->pose(idx).x());
			pose2D->set_y(path_ori->pose(idx).y());
		}


		INFO_MSG("Corrected path (#items: " << path->pose_size() << "):");
		for (int i=0; i<path->pose_size(); i++) {
			INFO_MSG(" -> " << path->pose(i).x() << "/" << path->pose(i).y());
		}

		// drive path
//		for (int i=0; i<5; i++) {
			pathInformer->publish(path);
//			usleep(250000);
//		}
		if (!pathResponseQueue->empty()) {
			pathResponseQueue->pop();
		}
		bool blink = true;
		INFO_MSG("Path sent.");
		do {
			for(int led=0; led<8; led++) {
				if(blink && led<4 || !blink && led>3) {
					myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
				} else {
					myCAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
				}
			}
			blink = !blink;

			usleep(500000);
			INFO_MSG(" -> Waiting for local planner response");
		} while (pathResponseQueue->empty());
		pathResponseQueue->pop();

		for(int led=0; led<8; led++) {
			myCAN.setLightColor(led, amiro::Color(amiro::Color::YELLOW));
		}

/*		for (int i=0; i<5; i++) {
			myCAN.setTargetSpeed(0,0);
			usleep(250000);
		}
*/			

		// send OK
		INFO_MSG((step*3+3) << ") Position reached. Sending OK to Follower.");
		positionPtr->set_x(0);
		positionPtr->set_y(0);
		positionPtr->set_orientation(PROTOCOL_OK);
		progressInformer->publish(positionPtr);
	}

	for(int led=0; led<8; led++) {
		myCAN.setLightColor(led, amiro::Color(amiro::Color::GREEN));
	}

	// wait for follower
	INFO_MSG("Waiting for follower");
	while (progressQueue->empty()) {
		usleep(500000);
	}
	progressQueue->pop();
	
	// send DISCONNECT
	INFO_MSG("Follower has finished. Sending DISCONNECT to Follower.");
	positionPtr->set_x(0);
	positionPtr->set_y(0);
	positionPtr->set_orientation(PROTOCOL_DISCONNECT);
	progressInformer->publish(positionPtr);

	for(int led=0; led<8; led++) {
		myCAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
	}

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


