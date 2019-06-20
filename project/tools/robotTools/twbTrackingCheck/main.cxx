//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Prints tracking data.
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// std includes
#include <iostream>
using namespace std;

// boost
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
namespace po = boost::program_options;
using namespace boost::chrono;

// rsb
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Handler.h>
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>
#include <boost/shared_ptr.hpp>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Event.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
using namespace rsb;

// types
#include <types/enum.pb.h>
#include <types/loc.pb.h>
#include <types/pose.pb.h>
#include <types/rotation.pb.h>
#include <types/shapes.pb.h>
#include <types/vertex.pb.h>

#include <Types.h>

#include <twb/TwbTracking.h>

// constants
int markerID = -1;
float meterPerPixel = 1.0; // m/pixel


// flags
bool specificMarker = false;
bool showVar = false;


float angleDiff(float angle1, float angle2) {
	float diff = fmod(angle1-angle2+M_PI, 2.0*M_PI);
	if (diff > M_PI) {
		diff = -1.0 * (diff-M_PI);
	}
	return diff;
}

int main(int argc, char **argv) {
	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("markerId,i", po::value<int>(&markerID), "ID of the marker for robot detection (if not given, all detected IDs will be shown).")
			("var,v", "Flag, if the variance of all the data of a specific marker shall be shown.");

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

	specificMarker = vm.count("markerId");
	if (specificMarker && markerID < 0) {
		std::cout << "The marker ID has to be 0 or greater!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}

	showVar = vm.count("var");

	if (showVar && !specificMarker) {
		std::cout << "Please set the marker ID, if the variance shall be shown!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}

	// +++++ RSB Initalization +++++

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// ------------ Converters ----------------------

	twbTrackingProcess::registerTracking();

	// ------------ Listener ---------------------

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(twbTrackingProcess::getTrackingScope(), twbTrackingProcess::getTrackingRSBConfig());
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));


	bool initialized = false;
	float xMinVar = 0, xMaxVar = 0, yMinVar = 0, yMaxVar = 0, tMinVar = 0, tMaxVar = 0, tOri = 0;

	while(true) {
		if (specificMarker) {
			twbTrackingProcess::TrackingObject trObj = twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID);
			if (twbTrackingProcess::isErrorTracking(trObj)) {
				ERROR_MSG("The marker " << markerID << " could not be tracked for the last " << twbTrackingProcess::TRACKING_TIMEOUT << " milliseconds!");
				return EXIT_FAILURE;
			}
			float posX = trObj.pos.x;
			float posY = trObj.pos.y;
			float posT = trObj.pos.theta;

			INFO_MSG("Current Position:");
			if (showVar) {
				if (!initialized) {
					xMinVar = posX;
					yMinVar = posY;
					xMaxVar = posX;
					yMaxVar = posY;
					tOri = posT;
					tMinVar = 0;
					tMaxVar = 0;
					initialized = true;
				} else {
					if (posX < xMinVar) xMinVar = posX;
					if (posY < yMinVar) yMinVar = posY;
					if (posX > xMaxVar) xMaxVar = posX;
					if (posY > yMaxVar) yMaxVar = posY;
					float angDiff = angleDiff(tOri, posT);
					if (fabs(angDiff) < M_PI/2.0) {
						if (angDiff < tMinVar) tMinVar = angDiff;
						if (angDiff > tMaxVar) tMaxVar = angDiff;
					}
				}
				float minAngle = tOri+tMinVar;
				float maxAngle = tOri+tMaxVar;
				INFO_MSG(" x: " << posX << " m (" << xMinVar << " m <-> " << xMaxVar << " m, " << (xMaxVar-xMinVar) << " m)");
				INFO_MSG(" y: " << posY << " m (" << yMinVar << " m <-> " << yMaxVar << " m, " << (yMaxVar-yMinVar) << " m)");
				INFO_MSG(" Ø: " << (posT*180.0/M_PI) << "° (" << (maxAngle*180.0/M_PI) << "° <-> " << (minAngle*180.0/M_PI) << "°, " << ((maxAngle-minAngle)*180.0/M_PI) << "°)");
			} else {
				INFO_MSG(" x: " << posX << " m");
				INFO_MSG(" y: " << posY << " m");
				INFO_MSG(" Ø: " << (posT*180.0/M_PI) << "°");
			}
		} else {
			std::vector<twbTrackingProcess::TrackingObject> positions = twbTrackingProcess::getNextTrackingObjects(trackingQueue);
			if (positions.size() > 0) {
				if (twbTrackingProcess::isErrorTracking(positions[0])) {
					ERROR_MSG("There wasn't any tracking data for the last " << twbTrackingProcess::TRACKING_TIMEOUT << " milliseconds!");
					return EXIT_FAILURE;
				}
				INFO_MSG("Current positions:");
				for (int i=0; i<positions.size(); i++) {
					INFO_MSG(" " << positions[i].id << ": " << positions[i].pos.x << "/" << positions[i].pos.y << " m, " << (positions[i].pos.theta*180.0/M_PI) << "°");
				}
			} else {
				WARNING_MSG("List is empty!");
			}
		}

		usleep(500000);
	}

	return EXIT_SUCCESS;
}
