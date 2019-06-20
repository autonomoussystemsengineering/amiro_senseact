//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Sets the lights based on RSB commands.
//============================================================================

//#define TRACKING
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
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
#include <rsb/util/QueuePushHandler.h>

#include <rst/generic/Value.pb.h>

using namespace std;

#include <Types.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <extspread.hpp>
#include <actModels/lightModel.h>

using namespace rsb;
using namespace rsb::patterns;


// scopenames and spread data for rsb
std::string commandInscope = "/amiro/lights";
std::string recScope = "/amiro/lightsrec";
std::string spreadhost = "127.0.0.1";
std::string spreadport = "4803";

// recognition answers
std::string REC_OK = "OK";
std::string REC_ERROR = "ERROR";

// color set
std::vector<amiro::Color> colorsSet = { LightModel::initColors[0],
					LightModel::initColors[1],
					LightModel::initColors[2],
					LightModel::initColors[3],
					LightModel::initColors[4],
					LightModel::initColors[5],
					LightModel::initColors[6],
					LightModel::initColors[7]};

std::vector<amiro::Color> curColors = {	amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK),
					amiro::Color(amiro::Color::BLACK)};
// lighting type
bool init = true;
bool shine = false;
bool blinking = false;
bool blinkWarning = false;
bool crossed = false;
bool circleLeft = false;
bool circleRight = false;
bool singleColor = true;

int commandTypeSet = LightModel::LightType::SINGLE_INIT;
int periodTimeSet = 0;

bool exitProg = false;


// method prototypes
bool getCommand(boost::shared_ptr<rst::generic::Value> commandValueArray, rsb::Informer<std::string>::Ptr recInformer);
void setColor(bool colorChanged, int counter10ms, int colorCounter, ControllerAreaNetwork &myCAN);
bool sameColors(amiro::Color c1, amiro::Color c2);


// main function
int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("commandScope,c", po::value<std::string>(&commandInscope), "Inscope for commands (default: '/amiro/lights').")
			("recScope,r", po::value<std::string>(&recScope), "Outscope for recognition answers (default: '/amiro/lightsrec').")
			("host", po::value<std::string>(&spreadhost), "Host for Programatik Spread (default: '127.0.0.1').")
			("port", po::value<std::string>(&spreadport), "Port for Programatik Spread (default: '4803').");

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

	INFO_MSG("Initialize RSB");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
//	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
//	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);
boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	// ------------ Listener ----------------------

	// prepare RSB listener for commands
	rsb::ListenerPtr commandListener = factory.createListener(commandInscope, extspreadconfig);
  //boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::generic::Value>> >commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::generic::Value>>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::generic::Value>(commandQueue)));

	// ------------ Informer -----------------------

	// prepare RSB informer for recognition
	rsb::Informer<std::string>::Ptr recInformer = factory.createInformer<std::string> (recScope, extspreadconfig);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;
	boost::shared_ptr<rst::generic::Value> commandVector;
	int timeCounter = 0;
	uint changeColorCounter = 0;
	bool colorChanged = false;
	setColor(true, timeCounter, changeColorCounter, myCAN);

	INFO_MSG("Starting lighting loop");
	exitProg = false;
	while (!exitProg) {
		// check for new color input
		colorChanged = false;
		if (!commandQueue->empty()) {
			commandVector = boost::static_pointer_cast<rst::generic::Value >(commandQueue->pop());
			colorChanged = getCommand(commandVector, recInformer);
		}
		if (colorChanged) {
			changeColorCounter = 0;
		}
		if (timeCounter >= periodTimeSet/10) {
			timeCounter = 0;
			if (changeColorCounter >= colorsSet.size()-1) {
				changeColorCounter = 0;
			} else {
				changeColorCounter++;
			}
		} else {
			timeCounter++;
		}
		// set colors
		setColor(colorChanged, timeCounter, changeColorCounter, myCAN);

		// sleep 10 ms
		usleep(10000);
	}

	for (uint led=0; led<8; led++) {
		myCAN.setLightColor(led, LightModel::initColors[led]);
	}

	// normal exit procedure
	INFO_MSG("Exit");
	return EXIT_SUCCESS;
}


bool sameColors(amiro::Color c1, amiro::Color c2) {
	return c1.getRed() == c2.getRed() && c1.getGreen() == c2.getGreen() && c1.getBlue() == c2.getBlue();
}

bool getCommand(boost::shared_ptr<rst::generic::Value> commandValueArray, rsb::Informer<std::string>::Ptr recInformer) {
	DEBUG_MSG("New command received");
  if (commandValueArray->type()!=rst::generic::Value::ARRAY){
    boost::shared_ptr<std::string> StringPtr(new std::string(REC_ERROR
    ));
    recInformer->publish(StringPtr);
    WARNING_MSG("Invalid color command!");
    return false;
  }
  std::vector<int> commandVector(0,0);
  rst::generic::Value entry;
  for (int i=0;i<commandValueArray->array_size();i++){
    entry=commandValueArray->array(i);
    if (entry.type()!=rst::generic::Value::INT){
      boost::shared_ptr<std::string> StringPtr(new std::string(REC_ERROR
      ));
      recInformer->publish(StringPtr);
      WARNING_MSG("Invalid color command!");
      return false;
    }
    commandVector.push_back(entry.int_());
  }

	// check if vector has correct length
	if (commandVector.size() >= 2) {
		int commandSize = commandVector.size();
		bool error = false;
		bool colorChanged = false;
		int commandType = commandVector.at(0);
		bool commandTypeInit = (commandType == LightModel::LightType::SINGLE_INIT) || (commandType == LightModel::LightType::CHANGE_INIT);
		bool lightTypeChange = LightModel::lightTypeIsChange(commandType);
		std::vector<amiro::Color> colors;
		int periodTime = commandVector.at(commandSize-1);
		if (periodTime < 0) periodTime = 0;
		if ((LightModel::lightTypeIsKnown(commandType))
		 || (lightTypeChange && (commandSize-2) % 3 == 0)
		 || (!lightTypeChange && (commandSize == 5 || commandSize == 26))
		 || (commandTypeInit)
		) {
			// check if everything negative
			bool allNegative = true;
			for (int c=0; c<commandSize; c++) {
				if (commandVector.at(c) >= 0) {
					allNegative = false;
					break;
				}
			}
			if (allNegative) {
				exitProg = true;
				return false;
			}

			// load colors
			int colorCount = (commandSize-2)/3;
			if (commandTypeInit) {
				for (uint led=0; led<8; led++) {
					amiro::Color color((int)(LightModel::initColors[led].getRed()), (int)(LightModel::initColors[led].getGreen()), (int)(LightModel::initColors[led].getBlue()));
					colors.push_back(color);
				}
			} else if (colorCount == 0) {
				WARNING_MSG("Missing colors! Please set colors for lighting types, which do not use initial colors!");

				boost::shared_ptr<std::string> StringPtr(new std::string(REC_ERROR));
				recInformer->publish(StringPtr);

				return false;
			} else if (!lightTypeChange && colorCount == 1) {
				amiro::Color color(commandVector.at(1), commandVector.at(2), commandVector.at(3));
				for (uint led=0; led<8; led++) {
					colors.push_back(color);
				}
			} else {
				for (int led=0; led<colorCount; led++) {
					amiro::Color color(commandVector.at(led*3+1), commandVector.at(led*3+2), commandVector.at(led*3+3));
					colors.push_back(color);
				}
			}

			// correct period time to 100ms steps and to lighting type specific minimal time
			if (periodTime % 100 != 0) {
				periodTime += (100 - periodTime % 100);
			}
			if (periodTime < LightModel::LightTypeMinPeriodTime[commandType]) {
				periodTime = LightModel::LightTypeMinPeriodTime[commandType];
			}

			// check if the color(s), lighting type or period time have been changed
			colorChanged = (singleColor && lightTypeChange) || (!singleColor && !lightTypeChange) || commandType != commandTypeSet || periodTime != periodTimeSet;
			if (!colorChanged && colors.size() == colorsSet.size()) {
				for (uint led=0; led<colors.size(); led++) {
					if (colors[led].getRed() != colorsSet[led].getRed() || colors[led].getGreen() != colorsSet[led].getGreen() || colors[led].getBlue() != colorsSet[led].getBlue()) {
						colorChanged = true;
						break;
					}
				}
			} else if (colors.size() != colorsSet.size()) {
				colorChanged = true;
			}
		}

		// only set everything new, if there is a change
		if (colorChanged) {
			init = false;
			shine = false;
			blinking = false;
			blinkWarning = false;
			crossed = false;
			circleLeft = false;
			circleRight = false;

			switch (commandType) {
				case LightModel::LightType::SINGLE_INIT: init = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_SHINE: shine = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_BLINK: blinking = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_WARNING: blinkWarning = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_CROSSED: crossed = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_CIRCLELEFT: circleLeft = true; singleColor = true; break;
				case LightModel::LightType::SINGLE_CIRCLERIGHT: circleRight = true; singleColor = true; break;
				case LightModel::LightType::CHANGE_INIT: init = true; singleColor = false; break;
				case LightModel::LightType::CHANGE_SHINE: shine = true; singleColor = false; break;
				case LightModel::LightType::CHANGE_BLINK: blinking = true; singleColor = false; break;
				default: WARNING_MSG("Lighting type '" << commandType << "' is unknown!"); error = true; break;
			}
			if (error) {
				// command was unknown, switch back everything
				switch (commandTypeSet) {
					case LightModel::LightType::SINGLE_INIT: init = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_SHINE: shine = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_BLINK: blinking = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_WARNING: blinkWarning = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_CROSSED: crossed = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_CIRCLELEFT: circleLeft = true; singleColor = true; break;
					case LightModel::LightType::SINGLE_CIRCLERIGHT: circleRight = true; singleColor = true; break;
					case LightModel::LightType::CHANGE_INIT: init = true; singleColor = false; break;
					case LightModel::LightType::CHANGE_SHINE: shine = true; singleColor = false; break;
					case LightModel::LightType::CHANGE_BLINK: blinking = true; singleColor = false; break;
				}

				boost::shared_ptr<std::string> StringPtr(new std::string(REC_ERROR));
				recInformer->publish(StringPtr);

				return false;
			} else {
				// save lighting type, colors and period time
				commandTypeSet = commandType;
				colorsSet.clear();
				for (uint led=0; led<colors.size(); led++) {
					colorsSet.push_back(colors[led]);
				}
				periodTimeSet = periodTime;
				// correct period time to 100ms steps and to lighting type specific minimal time
				if (periodTimeSet % 100 != 0) {
					periodTimeSet += (100 - periodTimeSet % 100);
				}
				if (periodTimeSet < LightModel::LightTypeMinPeriodTime[commandTypeSet]) {
					periodTimeSet = LightModel::LightTypeMinPeriodTime[commandTypeSet];
				}

				// print change notice
				switch (commandTypeSet) {
					case LightModel::LightType::SINGLE_INIT: INFO_MSG("Color changed: Type " << LightModel::LightTypeName[commandTypeSet] << ", Colors are initial"); break;
					case LightModel::LightType::SINGLE_SHINE: INFO_MSG("Color changed: Type " << LightModel::LightTypeName[commandTypeSet] << ", Colors are set"); break;
					case LightModel::LightType::CHANGE_INIT: INFO_MSG("Color changed: Type " << LightModel::LightTypeName[commandTypeSet] << ", Colors are initial, period time has " << periodTimeSet << " ms per Color"); break;
					default:
						if (singleColor) {
							INFO_MSG("Color changed: Type " << LightModel::LightTypeName[commandTypeSet] << ", Colors are set, period time has " << periodTimeSet << " ms");
						} else {
							INFO_MSG("Color changed: Type " << LightModel::LightTypeName[commandTypeSet] << ", Colors are set, period time has " << periodTimeSet << " ms per Color");
						}
						break;
				}

				boost::shared_ptr<std::string> StringPtr(new std::string(REC_OK));
				recInformer->publish(StringPtr);

				DEBUG_MSG(" => Lights have been changed.");

				return true;
			}
		}

		boost::shared_ptr<std::string> StringPtr(new std::string(REC_OK));
		recInformer->publish(StringPtr);

		DEBUG_MSG(" => Lights already set.");

		return false;
	}

	boost::shared_ptr<std::string> StringPtr(new std::string(REC_ERROR));
	recInformer->publish(StringPtr);

	WARNING_MSG("Invalid color command!");

	return false;
}


void setColor(bool colorChanged, int counter10ms, int colorCounter, ControllerAreaNetwork &myCAN) {
	// prepare comparison values
	int steps10ms = periodTimeSet/10;
	int steps10msHalf = steps10ms/2;
	bool firstHalf = counter10ms < steps10msHalf;
	int roundStepCounter = -1;
	if (periodTimeSet > 0) roundStepCounter = counter10ms*10/(periodTimeSet/8);

	// only set lights for specific reasons
	if ((colorChanged)
	 || (!singleColor && counter10ms == 0)
	 || ((blinking || blinkWarning || crossed) && counter10ms % steps10msHalf == 0)
	 || (circleLeft || circleRight)
	) {
		for (int led=0; led<8; led++) {
			int ledNext = led+1;
			if (ledNext >= 8) ledNext -= 8;

			if ((init)
			 || (shine)
		    	 || (blinking && firstHalf)
		    	 || (blinkWarning && firstHalf && led >= 4)
		    	 || (blinkWarning && !firstHalf && led < 4)
			 || (crossed && firstHalf && led % 2 == 0)
			 || (crossed && !firstHalf && led % 2 != 0)
		    	 || (circleLeft && (7-led == roundStepCounter || 7-ledNext == roundStepCounter))
		    	 || (circleRight && (led == roundStepCounter || ledNext == roundStepCounter))
			) {
				// this LED has to be turned on (color is set)
				if (singleColor && !sameColors(curColors[led], colorsSet[led])) {
					myCAN.setLightColor(led, colorsSet[led]);
					curColors[led].setRedGreenBlue((int)(colorsSet[led].getRed()), (int)(colorsSet[led].getGreen()), (int)(colorsSet[led].getBlue()));
				} else if (!singleColor && !sameColors(curColors[led], colorsSet[colorCounter])) {
					myCAN.setLightColor(led, colorsSet[colorCounter]);
					curColors[led].setRedGreenBlue((int)(colorsSet[colorCounter].getRed()), (int)(colorsSet[colorCounter].getGreen()), (int)(colorsSet[colorCounter].getBlue()));
				}
			} else if (!sameColors(curColors[led], amiro::Color(0, 0, 0))) {
				// this LED has to be turned off (color is black)
				myCAN.setLightColor(led, amiro::Color(0, 0, 0));
				curColors[led].setRedGreenBlue(0, 0, 0);
			}
		}
	}
}
