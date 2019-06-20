//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Gets the robot position and sensor values over RSB and
//               saves them into the file "RingProximityAndOdometry.txt".
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

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

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

#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

// micrometer to meter
const float YM_2_M = 1000000.0;

// scopenames for rsb
std::string proxSensorInscope = "/rir_prox";
std::string odometryInscope = "/odometrydata";

int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
		("rirScope,r", po::value<std::string> (&proxSensorInscope), "Scope for receiving proximity values for obstacle.")
		("odoScope,o", po::value<std::string> (&odometryInscope), "Scope for receiving odometry data.")
		("init,i", "Just initializes data file.");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	if (vm.count("init")) {
		FILE *file;
		file = fopen("RingProximityAndOdometry.txt", "w");
		for (int step=0; step<8; step++) {
			fprintf(file, "PRV %i\t", step);
		}
		fprintf(file, "X [ym]\tY [ym]\tTheta [yrad]\n");
		fclose(file);

        	INFO_MSG("File has been initialized");

		return EXIT_SUCCESS;
	}

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	converterRepository<std::string>()->registerConverter(converterVecInt);

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// register converter for twbTracking::proto::Pose2D
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);

	// prepare rsb listener for odometry data
	rsb::ListenerPtr odometryListener = factory.createListener(odometryInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>odoQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
	odometryListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(odoQueue)));

	// Register new converter for the pose list
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(
			new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
	rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

	FILE *file;
	file = fopen("RingProximityAndOdometry.txt", "a");

	// read sensor values

	while (proxQueue->empty() || odoQueue->empty()) {
		// Sleep for a while
		boost::this_thread::sleep( boost::posix_time::milliseconds(200) ); 
	}       

	boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int> >(
			proxQueue->pop());

	// get odometry data
	boost::shared_ptr<twbTracking::proto::Pose2D> robotPosition = boost::static_pointer_cast<
			twbTracking::proto::Pose2D>(odoQueue->pop());

	// calculate the robots pose from odomety data (starting in the middle of the global map)
	int posX = robotPosition->x();
	int posY = robotPosition->y();
	int theta = robotPosition->orientation();

	for (int sensorIdx = 0; sensorIdx < sensorValues->size(); sensorIdx++) {
		fprintf(file, "%i\t", sensorValues->at(sensorIdx));
	}
	fprintf(file, "%i\t%i\t%i\n", posX, posY, theta);

	fclose(file);

	INFO_MSG("Data read");

	return EXIT_SUCCESS;
}
