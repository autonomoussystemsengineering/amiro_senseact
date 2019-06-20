//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : The robot drives forward to the current edge, turns until the
//               edge is behind and drives to the next edge. Afterwards it 
//               drives a little bit backwards to get distance from the edge.
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
#include <rsb/Version.h>
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

#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;
using namespace std;

#include <Types.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <Constants.h>
#include <Color.h>
#include <sensorModels/VCNL4020Models.h>
#include <actModels/lightModel.h>

#include <extspread/extspread.hpp>

using namespace amiro;

using namespace rsb;
using namespace rsb::patterns;


// margins
#define OBSTACLE_MARGIN 100
#define OBSTACLE_MARGIN_SIDE 7500
#define GROUND_MARGIN 0.06
#define GROUND_MARGIN_DANGER 0.05
#define EDGE_DIFF 0.004

// velocities
#define VEL_FORWARD 8
#define VEL_FORWARD_SLOW 4
#define VEL_TURNING 40
#define VEL_TURNING_SLOW 20


// scopenames for rsb
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
std::string proxSensorInscopeGround = "/rir_prox/ground";
std::string commandInscopePart1 = "/tobiamirotable";
std::string commandInscopePart2 = "/state";
std::string answerOutscopePart1 = "/amirotable";
std::string answerOutscopePart2 = "tobi/state";
std::string lightOutscope = "/amiro/lights";

// rsb commands
std::string cmdTransport = "drive";
std::string ansReady = "finish";
std::string cmdansRec = "rec";

// velocities
float forwardSpeed = 0.06; // m/s
float correctSpeed = 0.02; // m/s
float turnSpeed = 20.0 * M_PI/180.0; // rad/s

// amiro constants
unsigned int amiroID = 0;

// behavior values
float irDetectionDist = 0.05; // m
float edgeMaxDist = 0.055; // m
float edgeMinDist = 0.01; // m
float edgeDistVariance = 0.005; // m
float tableEdgeDist = 0.05; // m

std::string spreadhost = "localhost";
std::string spreadport = "4823";

int stateCount = 7;
enum States {	waitForCommand,
		bufferForStart,
		driveForOrientation,
		correctForOrientation,
		turnForOrientation,
		driveToPosition,
		correctEdgeDist
};

std::string stateNames[] = {	"Waiting for drive command.",
				"Start driving.",
				"Driving to edge for orientation.",
				"Correct distance to edge for orientation.",
				"Turning towards other table edge.",
				"Drive to position.",
				"Correct distance to edge."
};


// Initialize command flags
bool transportCommand = false;
bool restartCommand = false;
bool transportFinished = false;
bool restartFinished = false;

bool commandsViaIR = true;


void sendMotorCmd(float speed, float angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(speed*1000000.0), (int)(angle*1000000.0));
}

void printIRDist(boost::shared_ptr<std::vector<int>> sensorValues) {
	for (int i=0; i<ringproximity::SENSOR_COUNT; i++) {
		float dist = VCNL4020Models::obstacleModel(0, sensorValues->at(i));
		DEBUG_MSG(" -> Sensor " << i << ": " << dist << " (" << sensorValues->at(i) << ")");
	}
}

bool gotIRCommand(boost::shared_ptr<std::vector<int>> sensorValues) {
	float distBefore = VCNL4020Models::obstacleModel(0, sensorValues->at(ringproximity::SENSOR_COUNT-1));
	for (int i=0; i<ringproximity::SENSOR_COUNT; i++) {
		float distCur = VCNL4020Models::obstacleModel(0, sensorValues->at(i));
		if (distCur < irDetectionDist && distBefore < irDetectionDist) {
//			printIRDist(sensorValues);
			return true;
		}
		distBefore = distCur;
	}
	return false;
}

bool edgeIsInfront(boost::shared_ptr<std::vector<int>> sensorValues, bool drivingToEdge=true) {
	for (int i=0; i<4; i++) {
		float dist = VCNL4020Models::edgeModel(sensorValues->at(i+2));
		if (dist < edgeMaxDist) {
			return true;
		}
	}
	return false;
}

bool enoughDistToEdge(boost::shared_ptr<std::vector<int>> sensorValues) {
	float minDist = 10.0;
	for (int i=0; i<4; i++) {
		float dist = VCNL4020Models::edgeModel(sensorValues->at(i+2));
		minDist = min(dist, minDist);
	}
	return minDist > edgeMinDist;
}

bool edgeIsDirectBehind(boost::shared_ptr<std::vector<int>> sensorValues) {
	return VCNL4020Models::edgeModel(sensorValues->at(0)) < edgeMaxDist && VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1)) < edgeMaxDist;
}

void turnToEdge(ControllerAreaNetwork &myCAN, boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround) {
	boost::shared_ptr<std::vector<int>> sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
	float distS = VCNL4020Models::edgeModel(sensorValues->at(0));
	float distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
	if (distS > distB) {
		float c = distB;
		distB = distS;
		distS = c;
	}
	while (!(edgeIsDirectBehind(sensorValues) && distB-distS <= edgeDistVariance)) {
		sendMotorCmd(0.0, turnSpeed, myCAN);
		while (!(edgeIsDirectBehind(sensorValues) && distB-distS <= edgeDistVariance)) {
			usleep(100000);
			sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
			distS = VCNL4020Models::edgeModel(sensorValues->at(0));
			distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
			if (distS > distB) {
				float c = distB;
				distB = distS;
				distS = c;
			}
		}
		sendMotorCmd(0.0, 0.0, myCAN);
		usleep(300000);
		sensorValues = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
		distS = VCNL4020Models::edgeModel(sensorValues->at(0));
		distB = VCNL4020Models::edgeModel(sensorValues->at(ringproximity::SENSOR_COUNT-1));
		if (distS > distB) {
			float c = distB;
			distB = distS;
			distS = c;
		}
	}
}

void setLights(int lightType, Color color, int period, rsb::Informer< std::vector<int> >::Ptr informer) {
	std::vector<int> lightCommand = setLight2Vec(lightType, color, period);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightCommand.begin(),lightCommand.end()));
	informer->publish(commandVector);
}

void checkCommands(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue, rsb::Informer<std::string>::Ptr informerCmd, boost::shared_ptr<std::vector<int>> sensorValues) {
	transportCommand = false;
	restartCommand = false;
	if (!cmdQueue->empty()) {
		std::string command(*cmdQueue->pop());
		DEBUG_MSG("New command is '" << command << "'");
		if (command == cmdTransport) {
			DEBUG_MSG("Transport command received.");
			transportCommand = true;
		}
		if (transportFinished && command == ansReady+cmdansRec) {
			DEBUG_MSG("Finished transport has been recognized.");
			transportFinished = false;
		}
		if (transportCommand || restartCommand) {
			boost::shared_ptr<std::string> stringPublisher(new std::string(command + cmdansRec));
			informerCmd->publish(stringPublisher);
		}
	}
	if (commandsViaIR) {
		if (gotIRCommand(sensorValues)) {
			DEBUG_MSG("IR command received.");
			transportCommand = true;
			restartCommand = true;
			boost::shared_ptr<std::string> stringPublisher(new std::string(cmdansRec));
			informerCmd->publish(stringPublisher);
		}
	}
}

void sendCommands(rsb::Informer<std::string>::Ptr informerCmd) {
	if (transportFinished) {
		boost::shared_ptr<std::string> stringPublisher(new std::string(ansReady));
		informerCmd->publish(stringPublisher);
	}
	if (restartFinished) {
		boost::shared_ptr<std::string> stringPublisher(new std::string(ansReady));
		informerCmd->publish(stringPublisher);
	}
}

int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	float turnSpeedS = 20.0;
	unsigned int bufferSec = 5;

	std::string commandInscope, answerOutscope;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
		("id", po::value<unsigned int>(&amiroID), "AMiRo ID (default: 0).")
		("proxObstacleInscope,o", po::value<std::string>(&proxSensorInscopeObstacle), "Inscope for receiving proximity sensor values for obstacle model.")
		("proxGroundInscope,g", po::value<std::string>(&proxSensorInscopeGround), "Inscope for receiving proximity sensor values for edge model.")
		("lightOutscope,l", po::value<std::string>(&lightOutscope), "Outscope for light commands.")
		("forwardSpeed,f", po::value<float>(&forwardSpeed), "Forward speed in m/s (default: 0.06).")
		("turnSpeed,t", po::value<float>(&turnSpeedS), "Angular speed in degree/s (default: 20.0).")
		("irDetectionDist,i", po::value<float>(&irDetectionDist), "Maximal distance for command detection by the proximity sensors in m (default: 0.05).")
		("edgeMaxDist,d", po::value<float>(&edgeMaxDist), "Distance for edge detection in m (default: 0.055).")
		("edgeDistVariance,v", po::value<float>(&edgeDistVariance), "Maximal variance between the proximity sensors for edge orientation in m (default: 0.005).")
		("tableEdgeDistance,e", po::value<float>(&tableEdgeDist), "Distance between robot and table edge for grasping and setting objects onto the robot in m (default: 0.05).")
		("bufferStart,b", po::value<unsigned int>(&bufferSec), "Buffer between command recognition and start in seconds (default: 5).")
		("commandsRSBOnly,r", "Flag, if the commands shall only given via RSB")
		("host", po::value<std::string>(&spreadhost), "Host of external spread (default: localhost).")
		("port", po::value<std::string>(&spreadport), "Port of external spread (default: 4823).");

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

	if (bufferSec < 1) {
		bufferSec = 1;
	}

	commandsViaIR = !vm.count("commandsRSBOnly");

	commandInscope = commandInscopePart1 + std::to_string(amiroID) + commandInscopePart2;
	answerOutscope = answerOutscopePart1 + std::to_string(amiroID) + answerOutscopePart2;

	turnSpeed = turnSpeedS * M_PI/180.0;

	INFO_MSG("Initialize RSB:");
	INFO_MSG(" - Proximity Sensor Inscopes:");
	INFO_MSG("    -> Obstacle Model:     " << proxSensorInscopeObstacle);
	INFO_MSG("    -> Edge Model:         " << proxSensorInscopeGround);
	INFO_MSG(" - Command Communication:");
	INFO_MSG("    -> Incomming commands: " << commandInscope);
	INFO_MSG("    -> Sending answers:    " << answerOutscope);
	INFO_MSG(" - Lights:");
	INFO_MSG("    -> Light Command:      " << lightOutscope);
	INFO_MSG("");

	// Get the RSB factory
	rsb::Factory& factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, spreadhost, spreadport);

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Listener ----------------------

	// prepare RSB listener for the IR data (obstacles)
	rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueObstacle)));

	// prepare RSB listener for the IR data (edges)
	rsb::ListenerPtr proxListenerGround = factory.createListener(proxSensorInscopeGround);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListenerGround->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueGround)));

	// prepare RSB listener for commands
	rsb::ListenerPtr cmdListener = factory.createListener(commandInscope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	cmdListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(cmdQueue)));

	// ------------ Informer ----------------------

	// prepare RSB informer for answers
	rsb::Informer<std::string>::Ptr informerAnswer = factory.createInformer<std::string> (answerOutscope, extspreadconfig);

	// prepare RSB informer for lights
	rsb::Informer< std::vector<int> >::Ptr informerLights = factory.createInformer< std::vector<int> > (lightOutscope);



	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	// Initialize state
	States state = States::waitForCommand;

	// start loop
	int period = 100; // ms
	bool stateSwitched = true;
	int problemCounter = 0;
	int bufferCounter = 0;
	int bufferMax = bufferSec*1000 / period; // ms
	int lightTypes[] = {LightModel::LightType::SINGLE_CIRCLERIGHT, LightModel::LightType::SINGLE_WARNING};
	int lightPeriods[] = {0, 800};
	bool blinkWarning = true;

	// Get initial sensor values
	while (proxQueueObstacle->empty() || proxQueueGround->empty()) {
		usleep(100000);
	}
	boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
	boost::shared_ptr<std::vector<int>> sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
	
	while (true) {
		// Check for problems
		if (problemCounter > 5) {
			ERROR_MSG("No incomming data for 500 ms!");
			break;
		}

		// load sensor data
		if (proxQueueObstacle->empty() || proxQueueGround->empty()) {
			problemCounter++;
		} else {
			problemCounter = 0;
		}
		if (!proxQueueObstacle->empty()) sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
		if (!proxQueueGround->empty()) sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());

		// Check for incomming commands
		checkCommands(cmdQueue, informerAnswer, sensorValuesObstacle);

		// Resend messages
		sendCommands(informerAnswer);

		// Print state
		if (stateSwitched) INFO_MSG("State: " << stateNames[state]);

		// Check state
		States newState = state;
		switch (state) {
		case States::waitForCommand:
			if (stateSwitched) {
				blinkWarning = !blinkWarning;
				setLights(LightModel::LightType::SINGLE_SHINE, Color(0,255,0), 0, informerLights);
			}
			if (transportCommand) {
				bufferCounter = 0;
				newState = States::bufferForStart;
			}
			break;

		case States::bufferForStart:
			if (stateSwitched) {
				bufferCounter = 0;
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			} else if (bufferCounter < bufferMax) {
				bufferCounter++;
			} else {
				newState = States::driveForOrientation;
			}
			break;

		case States::driveForOrientation:
			if (stateSwitched) {
				sendMotorCmd(forwardSpeed, 0.0, myCAN);
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			}
			if (edgeIsInfront(sensorValuesGround)) {
				sendMotorCmd(0.0, 0.0, myCAN);
				usleep(200000);
				while (proxQueueGround->empty()) {
					usleep(10000);
				}
				sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
				if (edgeIsInfront(sensorValuesGround)) {
					newState = States::correctForOrientation;
				} else {
					sendMotorCmd(forwardSpeed, 0.0, myCAN);
				}
			}
			break;

		case States::correctForOrientation:
			if (stateSwitched) {
				sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
				sendMotorCmd(-correctSpeed, 0.0, myCAN);
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			}
			if (enoughDistToEdge(sensorValuesGround)) {
				sendMotorCmd(0.0, 0.0, myCAN);
				usleep(200000);
				while (proxQueueGround->empty()) {
					usleep(10000);
				}
				sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
				if (enoughDistToEdge(sensorValuesGround)) {
					newState = States::turnForOrientation;
				} else {
					sendMotorCmd(-correctSpeed, 0.0, myCAN);
				}
			}
			break;

		case States::turnForOrientation:
			if (stateSwitched) {				
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			}
			turnToEdge(myCAN, proxQueueGround);
			newState = States::driveToPosition;
			break;

		case States::driveToPosition:
			if (stateSwitched) {
				sendMotorCmd(forwardSpeed, 0.0, myCAN);
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			}
			if (edgeIsInfront(sensorValuesGround)) {
				sendMotorCmd(0.0, 0.0, myCAN);
				usleep(200000);
				while (proxQueueGround->empty()) {
					usleep(10000);
				}
				sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());
				if (edgeIsInfront(sensorValuesGround)) {
					newState = States::correctEdgeDist;
				} else {
					sendMotorCmd(forwardSpeed, 0.0, myCAN);
				}
			}
			break;

		case States::correctEdgeDist:
			if (stateSwitched) {
				setLights(lightTypes[blinkWarning], Color(255,255,0), lightPeriods[blinkWarning], informerLights);
			}
			sendMotorCmd(-forwardSpeed/2.0, 0.0, myCAN);
			usleep((int)(tableEdgeDist/(forwardSpeed/2.0)*1000000.0));
			sendMotorCmd(0.0, 0.0, myCAN);
			newState = States::waitForCommand;
			restartFinished = false;
			transportFinished = true;
			break;

		default:
			ERROR_MSG("Unknown state!");
			return EXIT_FAILURE;

		}

		stateSwitched = newState != state;
		state = newState;

		usleep(period*1000);
	}

	return EXIT_SUCCESS;
}
