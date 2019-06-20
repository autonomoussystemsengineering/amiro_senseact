//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Gets the color set via console and sends it to the setLights
//               tool via RSB.
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
#include <rsb/util/EventQueuePushHandler.h>
#include <rsb/MetaData.h>


#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <actModels/lightModel.h>

using namespace rsb;
using namespace rsb::patterns;


// scopenames and spread data for rsb
std::string commandOutscope = "/amiro/lights";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	int commandType, periodTime;
	int colorRed = 0;
	int colorGreen = 0;
	int colorBlue = 0;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("commandScope,c", po::value<std::string>(&commandOutscope), "Inscope for commands (default: '/amiro/lights').")
			("host,h", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port,p", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').")
			("type,t", po::value<int>(&commandType), "Command type (has to be given).")
			("red,r", po::value<int>(&colorRed), "Color channel red in [0,255] (has to be given if not initial colors set).")
			("green,g", po::value<int>(&colorGreen), "Color channel green in [0,255] (has to be given if not initial colors set).")
			("blue,b", po::value<int>(&colorBlue), "Color channel blue in [0,255] (has to be given if not initial colors set).")
			("init,i", "Set intial colors  (has to be given if not red, green and blue channel are given).")
			("period,d", po::value<int>(&periodTime), "Period time in ms (has to be given).");

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

	if ((!vm.count("type") || !vm.count("red") || !vm.count("green") || !vm.count("blue") || !vm.count("period"))
	 && (!vm.count("type") || !vm.count("init") || !vm.count("period"))) {
		std::cout << "Some parameters have to be set! Please check the options!\n\n" << options << "\n";
		exit(1);
	}

	if (colorRed < 0 || colorRed > 255 || colorGreen < 0 || colorGreen > 255 || colorBlue < 0 || colorBlue > 255) {
		std::cout << "The color channels have to be given in [0,255]! Please check the options!\n\n" << options << "\n";
		exit(1);
	}

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Informer ----------------------

	rsb::Informer< std::vector<int> >::Ptr commandInformer = factory.createInformer< std::vector<int> > (commandOutscope);


	// prepare publication
	boost::shared_ptr<std::vector<int>> commandVector;
	std::string infoColor = "";
	if (!vm.count("init")) {
		std::vector<int> command(5,0);
		command[0] = commandType;
		command[1] = colorRed;
		command[2] = colorGreen;
		command[3] = colorBlue;
		command[4] = periodTime;
		commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(command.begin(),command.end()));
		infoColor = "the RGB color (" + std::to_string(colorRed) + ", " + std::to_string(colorGreen) + ", " + std::to_string(colorBlue) + ")";
	} else {
		std::vector<int> command = LightModel::setLights2Vec(commandType, LightModel::initColors, periodTime);
		commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(command.begin(),command.end()));
		infoColor = "the initial color set";
	}

	// publish command
	commandInformer->publish(commandVector);
	std::string cType = "";
	std::string minTimeType = "";
	if (LightModel::lightTypeIsKnown(commandType)) {
		cType = "lighting type " + LightModel::LightTypeName[commandType];
		minTimeType = " (minimal period time: " + std::to_string(LightModel::LightTypeMinPeriodTime[commandType]) + " ms)";
	} else {
		cType = "unknown lighting type (" + std::to_string(commandType) + ")";
	}
	INFO_MSG("Sending " << cType << " with " << infoColor << " and the period time " << periodTime << " ms" << minTimeType << ".");

	return EXIT_SUCCESS;
}
