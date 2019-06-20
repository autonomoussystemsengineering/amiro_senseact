//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Tool for sending choreo corrections.
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
#include <converter/vecIntConverter/main.hpp>
using namespace rsb;
using namespace muroxConverter;

// types
//#include <types/twbTracking.pb.h>
#include <types/enum.pb.h>
#include <types/loc.pb.h>
#include <types/pose.pb.h>
#include <types/poselist.pb.h>
#include <types/rotation.pb.h>
#include <types/shapes.pb.h>
#include <types/vertex.pb.h>

#include <twb/TwbTracking.h>
#include <Types.h>

// constants
int markerID = 0;
int amiroID = 0;


// scopenames for rsb
std::string choreoOutscope = "/twbchoreo/choreocorrection";
std::string goalInscope = "/twbchoreo/goal";

// global positions
typedef std::array<float,3> position_t;
position_t curPos;
position_t goalPos;

float normAngle(float angle) {
	float normed = fmod(angle, 2.0*M_PI);
	if (normed < 0) normed += 2.0*M_PI;
	return normed;
}

float angleDiff(float angle1, float angle2) {
	angle1 = normAngle(angle1);
	angle2 = normAngle(angle2);
	float direction = 1.0;
	if (angle1 > angle2) {
		float tmp = angle1;
		angle1 = angle2;
		angle2 = tmp;
		direction = -1.0;
	}
	float diff = angle2-angle1;
	if (diff > M_PI) {
		diff = 2.0*M_PI-diff;
		direction *= -1.0;
	}
	return direction*diff;
}


position_t toOwnPos(twbTrackingProcess::TrackingObject trackingObject) {
	position_t pos;
	pos[0] = trackingObject.pos.x;
	pos[1] = trackingObject.pos.y;
	if (twbTrackingProcess::isErrorTracking(trackingObject)) {
		pos[2] = -1;
	} else {
		pos[2] = trackingObject.pos.theta;
	}
	return pos;
}

void newGoal(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Object>>>goalQueue) {
	while (!goalQueue->empty()) {
		boost::shared_ptr<twbTracking::proto::Object> obj = boost::static_pointer_cast<twbTracking::proto::Object>(goalQueue->pop());
		if (obj->id() == amiroID) {
			goalPos[0] = obj->position().translation().x();
			goalPos[1] = obj->position().translation().y();
			goalPos[2] = obj->position().rotation().z() * M_PI/180.0;
		}
	}
}

int main(int argc, char **argv) {
	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("amiroID,a", po::value<int>(&amiroID), "ID of the AMiRo, which has to be unique! Flag must be set!")
			("markerId,m", po::value<int>(&markerID), "ID of the marker for robot detection. Flag must be set!")
			("goalIn", po::value<std::string>(&goalInscope), "Goal position inscope.")
			("choreoOut", po::value<std::string>(&choreoOutscope), "Choreography correction outscope.");

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

	// check for given AMiRo ID
	if (!vm.count("amiroID")) {
		std::cout << "The AMiRo ID has to be given! Please check the options.\n\n" << options << "\n";
		exit(1);
	}

	if (!vm.count("markerId")) {
		std::cout << "Please set the marker ID!\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}

	// +++++ RSB Initalization +++++

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	twbTrackingProcess::registerTracking();

	rsb::converter::ProtocolBufferConverter<twbTracking::proto::Object>::Ptr converterObject(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Object>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converterObject);

	rsb::converter::ProtocolBufferConverter<twbTracking::proto::PoseList>::Ptr converterPoseList(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::PoseList>);
	rsb::converter::converterRepository<std::string>()->registerConverter(converterPoseList);

	// ------------ ExtSpread Config ----------------

	// Generate the programatik Spreadconfig for extern communication
//	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER
	// Get the global participant config as a template
	rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig(); {
		// disable socket transport
		rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
		tmpPropSocket["enabled"] = boost::any(std::string("0"));

		// Get the options for spread transport, because we want to change them
		rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

		// enable socket transport
		tmpPropSpread["enabled"] = boost::any(std::string("1"));

		// Change the config
		tmpPropSpread["host"] = boost::any(std::string("localhost"));

		// Change the Port
		tmpPropSpread["port"] = boost::any(std::string("4823"));

		// Write the tranport properties back to the participant config
		tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
		tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
	}

	// ------------ Listener ---------------------

	// prepare RSB listener for goal positions
	rsb::ListenerPtr goalListener = factory.createListener(goalInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Object>>>goalQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Object>>(100));
	goalListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Object>(goalQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(twbTrackingProcess::getTrackingScope(), twbTrackingProcess::getTrackingRSBConfig());
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));

	// ------------ Informer -----------------

	// prepare RSB informer for sending choreo correction
	rsb::Informer<twbTracking::proto::PoseList>::Ptr correctionInformer = factory.createInformer<twbTracking::proto::PoseList> (choreoOutscope, tmpPartConf);


	curPos[0] = 0.0;
	curPos[1] = 0.0;
	curPos[2] = -1.0;
	goalPos[0] = 0.0;
	goalPos[1] = 0.0;
	goalPos[2] = -1.0;
	uint32_t correctionCounter = 0;
	INFO_MSG("Initializing finished. Press enter for sending corrections.")
	INFO_MSG("");
	while (true) {
		// wait for command
		while (true) {
			char keyInput = getchar();
			if (char(keyInput) == '\n') break;
		}

		// check goal position
		newGoal(goalQueue);
		if (goalPos[2] < 0) {
			ERROR_MSG("Didn't received the goal position of amiro " << amiroID << " yet!");
			continue;
		}

		// get current position
		curPos = toOwnPos(twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID));
		if (curPos[2] < 0) {
			ERROR_MSG("Marker " << markerID << " could not be detected!");
			continue;
		}

		// Rise correction counter
		correctionCounter++;

		// calculate correction
		boost::shared_ptr<twbTracking::proto::PoseList> poses(new twbTracking::proto::PoseList);
		poses->set_id(amiroID);
		twbTracking::proto::Pose *pose1 = poses->add_pose();
		twbTracking::proto::Pose *pose2 = poses->add_pose();
		twbTracking::proto::Pose *pose3 = poses->add_pose();
		twbTracking::proto::Translation *trans1 = pose1->mutable_translation();
		twbTracking::proto::Translation *trans2 = pose2->mutable_translation();
		twbTracking::proto::Translation *trans3 = pose3->mutable_translation();
		twbTracking::proto::Rotation *rot1 = pose1->mutable_rotation();
		twbTracking::proto::Rotation *rot2 = pose2->mutable_rotation();
		twbTracking::proto::Rotation *rot3 = pose3->mutable_rotation();
		trans1->set_x(curPos[0]);
		trans1->set_y(curPos[1] - 0.3);
		trans1->set_z(0.0);
		rot1->set_x(0.0);
		rot1->set_y(0.0);
		rot1->set_z(0.0);
		trans2->set_x(curPos[0]);
		trans2->set_y(curPos[1] + 0.3);
		trans2->set_z(0.0);
		rot2->set_x(0.0);
		rot2->set_y(0.0);
		rot2->set_z(180.0);
		trans3->set_x(curPos[0]);
		trans3->set_y(curPos[1]);
		trans3->set_z(0.0);
		rot3->set_x(0.0);
		rot3->set_y(0.0);
		rot3->set_z(curPos[3]);

		// send new positions
		correctionInformer->publish(poses);

		// print info
		INFO_MSG(correctionCounter << ". Correction:")
		INFO_MSG("a) Robot drives");
		INFO_MSG("    from " << curPos[0] << "/" << curPos[1] << " (" << (curPos[2]*180.0/M_PI) << "°)");
		INFO_MSG("    to " << goalPos[0] << "/" << goalPos[1] << " (" << (goalPos[2]*180.0/M_PI) << "°)");
		INFO_MSG("b) New positions:");
		for (int i=0; i<poses->pose_size(); i++) {
			float x = poses->pose(i).translation().x();
			float y = poses->pose(i).translation().y();
			float t = poses->pose(i).rotation().z();
			INFO_MSG("    " << (i+1) << ". " << x << "/" << y << ", " << t << "°");
		}
		INFO_MSG("");
	}

	return EXIT_SUCCESS;
}
