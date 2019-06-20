//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography including brakes.
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
#include <ControllerAreaNetwork.h>
#include <actModels/lightModel.h>

#include "choreo.h"
#include <twb/TwbTracking.h>
#include <Types.h>

// unit calculations
#define TO_MICRO 1000000.0
#define MICRO_TO 0.000001

// xml constants
std::string XMLMainName = "choreo";
std::string XMLStep = "choreoStep";
std::string XMLInclude = "choreoinclude";
std::string XMLBrake = "brake";
std::string XMLPositionX = "posx";
std::string XMLPositionY = "posy";
std::string XMLPositionT = "postheta";
std::string XMLDirectly = "direct";
std::string XMLLightAll = "la";
std::string XMLLightID = "l"; // + ID in [0,7]
std::string XMLLightSeperator = ",";
std::string XMLIncludeName = "choreopart";

// command constants
std::string COMMAND_INIT = "init";
std::string COMMAND_START = "start";
std::string COMMAND_STOP = "stop";
std::string COMMAND_BREAK = "break";
std::string COMMAND_CORR = "correct";

// command enum
enum CommandId {cmd_none, cmd_init, cmd_start, cmd_stop, cmd_break, cmd_corr};

// initialize CAN
ControllerAreaNetwork myCAN;

// constants
float driveSpeed = 0.1; // m/s
float turnSpeed = M_PI/6.0; // rad/s
float turnMax = M_PI/3.0; // rad/s
int timebuffer = (int)(0.125 * TO_MICRO); // us
std::string amiroName = "amiro0";
int markerID = 0;
float meterPerPixel = 1.0; // m/pixel
float trackingVariance = 0.02; // m
float trackingAngleVar = 4.0 * M_PI/180.0; // rad
float drivingMaxAngle = 10.0 * M_PI/180.0; // rad

// Flags
bool printChoreo = false;
bool useOdo = false;
bool useTwb = false;
bool startPosition = false;


// scopenames for rsb
std::string lightOutscope = "/amiro/lights";
std::string choreoInscope = "/twbchoreo/choreo";
std::string commandInscope = "/twbchoreo/command";
std::string amiroScope = "/twbchoreo/amiros";
std::string amiroLogoutScope = "/twbchoreo/logout";
std::string choreoCorrectionInscope = "/twbchoreo/choreocorrection";
std::string goalScope = "/twbchoreo/goal";

// global positions
position_t odoPos;
position_t oldPos;

// global variables
int amiroID = 0;
bool doChoreo = false;
bool pauseChoreo = false;
bool correctChoreo = false;
bool fakeCorrection = false;
int stepNumber = 0;
boost::shared_ptr<twbTracking::proto::PoseList> poses_include;

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
/* Examples: angle1, angle2, direction1*diff1 -> direction2*diff2
 * 150°, 210°, 60 -> 60
 * 210°, 150°, -60 -> -60
 * 30°, 330°, 300 -> -60
 * 330°, 30°, -300 -> 60
 */

//	float diff = fmod(angle1-angle2+M_PI, 2.0*M_PI);
//	if (diff > M_PI) {
//		diff = -1.0 * (diff-M_PI);
//	}
//	return diff;
/* Examples: angle1, angle2, diff -> result
 * 150°, 210°, 120° -> 120°
 * 210°, 150°, 240° -> -120°
 * 30°, 330°, -120° -> -120°
 * 330°, 30°, 120° -> 120°
 */

//	float diff = fmod(angle1-angle2, 2.0*M_PI);
//	if (diff > M_PI) {
//		diff = -1.0 * (diff-2.0*M_PI);
//	}
//	if (diff < -M_PI) {
//		diff = -1.0 * (diff+2.0*M_PI);
//	}
//	return diff;
/* Examples: angle1, angle2, diff -> result
 * 150°, 210°, -60° -> -60° / 300° -> -60°
 * 210°, 150°, 60° -> 60°
 * 30°, 330°, -300° -> -60° / 60° -> 60° !!!
 * 330°, 30°, 300° -> 60°
 */
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

void calcSpeeds(position_t curPos, float relDistx, float relDisty, float &v, float &w, float &time, float &dist, float &angle) {
	float actAngle = curPos[2];
	while (actAngle >= 2.0*M_PI) actAngle -= 2.0*M_PI;
	while (actAngle < 0) actAngle += 2.0*M_PI;

	float distDir = sqrt(relDistx*relDistx + relDisty*relDisty);
	float angleDir = atan2(relDisty, relDistx) - actAngle;
	while (angleDir >= 2.0*M_PI) angleDir -= 2.0*M_PI;
	while (angleDir < 0) angleDir += 2.0*M_PI;
	if (angleDir > M_PI) angleDir -= 2*M_PI;

	float angleBasis = abs(M_PI/2.0 - abs(angleDir));
	float angleTop = M_PI - 2.0*angleBasis;

	float moveRadius = distDir * sin(angleBasis) / sin(angleTop);
	float moveAngle = 2.0*angleDir;
	float moveDist = 2.0*moveRadius*M_PI * abs(moveAngle)/(2.0*M_PI);
	if (moveDist == 0.0) {
		moveDist = distDir;
	}

	time = moveDist/driveSpeed; // s
	v = driveSpeed; // m/s
	w = moveAngle/time; // rad/s
	dist = moveDist; // m
	angle = moveAngle; // rad
}


void trackingSpeeds(position_t curPos, float relDistx, float relDisty, bool directly, float &v, float &w) {
	float actAngle = curPos[2];

	float distDir = sqrt(relDistx*relDistx + relDisty*relDisty);
	float angleDist = atan2(relDisty, relDistx);
	float angleDir = angleDiff(actAngle, angleDist);

	if (angleDir == 0.0) {
		v = driveSpeed; // m/s
		w = 0.0; // rad/s

	} else if (fabs(angleDir) > drivingMaxAngle && directly || fabs(angleDir) >= M_PI/2.0 && !directly) {
		v = 0.0; // m/s
		if (angleDir < 0) {
			w = -turnSpeed; // rad/s
		} else {
			w = turnSpeed; // rad/s
		}

	} else {
		float angleBasis = fabs(M_PI/2.0 - fabs(angleDir));
		float moveDist, moveAngle;
		if (angleBasis <= 0.0) {
			moveAngle = M_PI;
			moveDist = distDir*M_PI/2.0;

		} else {
			float angleTop = M_PI - 2.0*angleBasis;
			float moveRadius = distDir * sin(angleBasis) / sin(angleTop);
			moveAngle = 2.0*angleDir;
			moveDist = 2.0*moveRadius*M_PI * abs(moveAngle)/(2.0*M_PI);
		}
		if (directly) {
			moveAngle *= 2.0;
		} else {
			moveAngle *= 1.5;
		}

		float time = moveDist/driveSpeed; // s
		v = driveSpeed; // m/s
		if (abs(relDistx) > trackingVariance && abs(relDisty) > trackingVariance) {
			w = moveAngle/time; // rad/s
		} else {
			w = 0.0; // rad/s
		}
		if (w > turnMax) {
			w = turnSpeed;
			v = 0.0;
		}
		if (w < -turnMax) {
			w = -turnSpeed;
			v = 0.0;
		}
	}
}

void saveOldPos() {
	oldPos[0] = odoPos[0];
	oldPos[1] = odoPos[1];
	oldPos[2] = odoPos[2];
}

void correctPosition() {
	// TODO some orientation correction?
}



// load a (sub)choreography from a file
Choreo loadSubChoreo(std::string choreoName, int subChoreoNum, std::vector<std::string> parents, float &xPos, float &yPos, float &tPos) {
	bool loadingCorrect = true;
	bool parentFound = false;
	parents.push_back(choreoName);

	// create spaces for "tabs"
	string spacesP = "";
	for (int i=0; i<subChoreoNum; i++) {
		spacesP += "  ";
	}
	string spacesC = spacesP + "  ";

	if (printChoreo) {
		string frontPart = "";
		if (subChoreoNum > 0) {
			frontPart += spacesP + "-> choreo include: ";
		}
		DEBUG_MSG(frontPart << "Load choreo '" << choreoName << "'");
	}
	Choreo choreo;
	using boost::property_tree::ptree;
	ptree pt;

	try {

		read_xml(choreoName, pt);
		BOOST_FOREACH( ptree::value_type const&tree, pt.get_child(XMLMainName)) {
			if (tree.first == XMLStep) {
				ChoreoStep choreoStep;
				choreoStep.braking = tree.second.get<int>(XMLBrake);
				choreoStep.directMovement = tree.second.get<int>(XMLDirectly) > 0;
				xPos += (float)(tree.second.get<int>(XMLPositionX))*MICRO_TO;
				yPos += (float)(tree.second.get<int>(XMLPositionY))*MICRO_TO;
				tPos = normAngle(tPos + (float)(tree.second.get<int>(XMLPositionT))*MICRO_TO * M_PI/180.0);
				position_t position;
				position[0] = xPos;
				position[1] = yPos;
				position[2] = tPos;
				choreoStep.position = position;
				light_t lights;
				std::string lightinput;
				try {
					lightinput = tree.second.get<std::string>(XMLLightAll);
					std::vector<std::string> splitstring;
					splitstring.clear();
					boost::split(splitstring, lightinput, boost::is_any_of(XMLLightSeperator));
					for (int i = 0; i < 8; i++) {
						lights[i][0] = boost::lexical_cast<int>(splitstring[2]);
						lights[i][1] = boost::lexical_cast<int>(splitstring[1]);
						lights[i][2] = boost::lexical_cast<int>(splitstring[0]);
					}
				} catch (...) {
					for (int i = 0; i < 8; i++) {
						std::vector<std::string> splitstring;
						std::string field = XMLLightID + boost::lexical_cast<std::string>(i);
						lightinput = tree.second.get<std::string>(field);
						splitstring.clear();
						boost::split(splitstring, lightinput, boost::is_any_of(XMLLightSeperator));
						lights[i][0] = boost::lexical_cast<int>(splitstring[2]);
						lights[i][1] = boost::lexical_cast<int>(splitstring[1]);
						lights[i][2] = boost::lexical_cast<int>(splitstring[0]);
					}
				}
				choreoStep.lights = lights;
				choreo.push_back(choreoStep);
				if (printChoreo) {
					DEBUG_MSG(spacesC << "-> choreo step: " << position[0] << "/" << position[1] << " [m], Ø: " << (position[2]*180.0/M_PI) << "°");
				}
			} else if (tree.first == XMLInclude) {
				std::string newfile = tree.second.get<std::string>(XMLIncludeName);
				// check for parent name
				parentFound = false;
				for (std::string parentName : parents) {
					if (newfile == parentName) {
						if (printChoreo) {
							ERROR_MSG(spacesC << "-> choreo include:");
							ERROR_MSG(spacesC << "   The choreo '" << newfile << "' is a parent choreo! Loops are not allowed!");
						}
						parentFound = true;
						break;
					}
				}
				// break if parent choreo shall be called and remove choreo elements
				if (parentFound) {
					choreo.clear();
					break;
				}
				// otherwise go on
				Choreo choreoInclude = loadSubChoreo(newfile, subChoreoNum+1, parents, xPos, yPos, tPos);
				if (choreoInclude.empty()) {
					loadingCorrect = false;
					choreo.clear();
					break;
				}
				choreo.insert(choreo.end(), choreoInclude.begin(), choreoInclude.end());
			} else if (printChoreo) {
				WARNING_MSG(spacesC << " -> Unknown childs!");
			}
		}
	} catch (...) {
		if (printChoreo) {
			ERROR_MSG(spacesP << "Problem in reading xml file => Clearing Choreo!");
		}
		choreo.clear();
	}
	if (choreo.empty() && loadingCorrect && !parentFound && printChoreo) {
		ERROR_MSG(spacesP << "Choreo is empty!");
	}	
	return choreo;
}

// loading choreography procedure
Choreo loadChoreo(std::string choreoName, float startX = 0.0, float startY = 0.0, float startTheta = 0.0) {
	float xPos = startX, yPos = startY, tPos = startTheta;
	std::vector<std::string> parents;
	Choreo choreo;
	ChoreoStep choreoStep;
	choreoStep.braking = 1;
	choreoStep.directMovement = true;
	position_t position;
	position[0] = xPos;
	position[1] = yPos;
	position[2] = tPos;
	choreoStep.position = position;
	light_t lights;
	for (int i = 0; i < 8; i++) {
		lights[i][0] = 255;
		lights[i][1] = 255;
		lights[i][2] = 0;
	}
	choreoStep.lights = lights;
	choreo.push_back(choreoStep);
	Choreo realChoreo = loadSubChoreo(choreoName, 0, parents, xPos, yPos, tPos);
	choreo.insert(choreo.end(), realChoreo.begin(), realChoreo.end());
	return choreo;
}


void setSpeeds(float v, float w, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed((int)(v*TO_MICRO), (int)(w*TO_MICRO));
}

void setLights(rsb::Informer< std::vector<int> >::Ptr informer, int lightType, amiro::Color color, int periodTime) {
	std::vector<int> lightVector = LightModel::setLight2Vec(lightType, color, periodTime);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightVector.begin(),lightVector.end()));
	informer->publish(commandVector);
}

void setLightsVec(rsb::Informer< std::vector<int> >::Ptr informer, int lightType, light_t colors, int periodTime) {
	std::vector<amiro::Color> colorVec;
	for (int l=0; l<8; l++) {
		amiro::Color color(colors[l][0], colors[l][1], colors[l][2]);
		colorVec.push_back(color);
	}
	std::vector<int> lightVector = LightModel::setLights2Vec(lightType, colorVec, periodTime);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightVector.begin(),lightVector.end()));
	informer->publish(commandVector);
}

int failureProc(rsb::Informer< std::vector<int> >::Ptr lightInformer, rsb::Informer<std::string>::Ptr amiroLogoutInformer) {
	setSpeeds(0, 0, myCAN);
	setLights(lightInformer, LightModel::LightType::SINGLE_WARNING, amiro::Color(255,0,0), 600);
	boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
	amiroLogoutInformer->publish(StringPtr);
	ERROR_MSG("The marker " << markerID << " couldn't be detected for " << twbTrackingProcess::TRACKING_TIMEOUT << " milliseconds!");
	return EXIT_FAILURE;
}

CommandId checkCommands(boost::shared_ptr<rsc::threading::SynchronizedQueue< EventPtr > > commandQueue, uint64_t &receiveTime) {
	CommandId result = cmd_none;
	if (!commandQueue->empty()) {
		EventPtr event = commandQueue->pop(0);
		std::string command = *static_pointer_cast<std::string>(event->getData());
		receiveTime = event->getMetaData().getReceiveTime();
		bool initializeCommand = command == COMMAND_INIT;
		bool stopCommand = command == COMMAND_STOP;
		bool startCommand = command == COMMAND_START;
		bool breakCommand = command == COMMAND_BREAK;
		bool correctCommand = command == COMMAND_CORR;
		if (stopCommand) {
			result = cmd_stop;
		} else if (breakCommand) {
			result = cmd_break;
		} else if (startCommand) {
			result = cmd_start;
		} else if (initializeCommand) {
			result = cmd_init;
		} else if (correctCommand) {
			result = cmd_corr;
		}
	}
	return result;
}

CommandId checkCommands(boost::shared_ptr<rsc::threading::SynchronizedQueue< EventPtr > > commandQueue) {
	uint64_t time;
	return checkCommands(commandQueue, time);
}

void choreoSleep_ms(int sleep_ms, boost::shared_ptr<rsc::threading::SynchronizedQueue< EventPtr > > commandQueue, boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::PoseList>>>choreoCorrectionQueue) {
	usleep(sleep_ms*1000);
	CommandId cmdId = checkCommands(commandQueue);
	if (doChoreo && cmdId == cmd_stop) doChoreo = false;
	if (pauseChoreo && cmdId == cmd_start) pauseChoreo = false;
	if (!pauseChoreo && cmdId == cmd_break) pauseChoreo = true;
	if (!correctChoreo && cmdId == cmd_corr) {
		correctChoreo = true;
		fakeCorrection = true;
		pauseChoreo = true;
	}
	if (!choreoCorrectionQueue->empty()) {
		poses_include = boost::static_pointer_cast<twbTracking::proto::PoseList>(choreoCorrectionQueue->pop());
		if (poses_include->id() == amiroID) {
			correctChoreo = true;
			pauseChoreo = true;
		}
	}
}

Choreo insertPoseListIntoChoreo(Choreo choreo, int curStepNo, boost::shared_ptr<twbTracking::proto::PoseList> poses) {
	INFO_MSG("");
	INFO_MSG("Correcting Choreo:")
	// transform new positions to choreo steps
	Choreo includedChoreo;
	for (int i=0; i<poses->pose_size(); i++) {
		ChoreoStep choreoStep;
		choreoStep.braking = 0;
		choreoStep.directMovement = true;
		position_t position;
		position[0] = poses->pose(i).translation().x();
		position[1] = poses->pose(i).translation().y();
		position[2] = poses->pose(i).rotation().z();
		choreoStep.position = position;
		light_t lights;
		for (int i = 0; i < 8; i++) {
			lights[i][0] = 255;
			lights[i][1] = 0;
			lights[i][2] = 0;
		}
		choreoStep.lights = lights;
		includedChoreo.push_back(choreoStep);
	}

	// print choreo before combination
	INFO_MSG(" -> Steps before:");
	for (int i=0; i<choreo.size(); i++) {
		INFO_MSG("     " << (i+1) << ". " << choreo[i].position[0] << "/" << choreo[i].position[1]);
	}

	// combine choreos
	std::size_t skippedSteps = (std::size_t)(curStepNo);
	Choreo choreo1(choreo.begin(), choreo.begin()+skippedSteps);
	Choreo choreo2(choreo.begin()+skippedSteps, choreo.end());
	choreo.clear();
	choreo.insert(choreo.end(), choreo1.begin(), choreo1.end());
	choreo.insert(choreo.end(), includedChoreo.begin(), includedChoreo.end());
	choreo.insert(choreo.end(), choreo2.begin(), choreo2.end());

	// print choreo after combination
	INFO_MSG(" -> Steps now:");
	for (int i=0; i<choreo.size(); i++) {
		INFO_MSG("     " << (i+1) << ". " << choreo[i].position[0] << "/" << choreo[i].position[1]);
	}

	return choreo;
}

int main(int argc, char **argv) {

	// delay to start the choreo after the rsb-event was created in ms
	int choreoDelay = 2000;
	int stepDelay = 1000;
	int idDelay = 1000;

	// default choreo name
	std::string choreoName = "testChoreo.xml";

	float startX = 0.0, startY = 0.0, startTheta = 0.0;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("verbose,v", "Print values that are published via CAN.")
			("amiroID,a", po::value<int>(&amiroID), "ID of the AMiRo, which has to be unique! Flag must be set!")
			("choreoIn", po::value<std::string>(&choreoInscope), "Choreography inscope.")
			("commandsIn", po::value<std::string>(&commandInscope), "Commands inscope.")
			("choreoCorrectionIn", po::value<std::string>(&choreoCorrectionInscope), "Choreo Correction inscope.")
			("amiroScope", po::value<std::string>(&amiroScope), "AMiRo Scope.")
			("logoutScope", po::value<std::string>(&amiroLogoutScope), "AMiRo Logout Scope.")
			("goalOut", po::value<std::string>(&goalScope), "Goal outscope.")
			("lightsOut", po::value<std::string>(&lightOutscope), "Light outscope.")
			("choreoname,c", po::value<std::string>(&choreoName), "Initial Choreography name.")
			("printChoreo,p", "Prints the loaded steps of the choreo.")
			("startX", po::value<float>(&startX), "Optional start position on x-axis in meters.")
			("startY", po::value<float>(&startY), "Optional start position on y-axis in meters.")
			("startTheta", po::value<float>(&startTheta), "Optional start position angle in degrees.")
			("choreoDelay", po::value<int>(&choreoDelay), "Delay between receiving the start command via RSB and starting the choreography in ms (default 2000 ms).")
			("stepDelay", po::value<int>(&stepDelay), "Delay of the brake between two steps of the choreography in ms (default 1000 ms).")
			("idDelay", po::value<int>(&idDelay), "Delay between the AMiRo starts in ms (default: 1000 ms).")
			("useOdo,o", "Flag if for navigation just the odometry shall be used.")
			("useTwb,t", "Flag if for navigation the telework bench shall be used (tracking navigation).")
			("markerId,m", po::value<int>(&markerID), "ID of the marker for robot detection (has to be set if tracking navigation is activated).")
			("mmp", po::value<float>(&meterPerPixel), "Meter per pixel of the robot detection (has to be set if tracking navigation is activated).");

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

	startPosition = vm.count("startX") && vm.count("startY") && vm.count("startTheta");
	if ((vm.count("startX") || vm.count("startY") || vm.count("startTheta")) && !startPosition) {
		std::cout << "If defining the optional start position, please give x- and y-coordinates and the angle.\n\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}
	startTheta *= M_PI/180.0;

	// check for given navigation type
	if (!vm.count("useOdo") && !vm.count("useTwb")) {
		std::cout << "Please set the navigation type:\n -> Odometry only ('useOdo')\n -> TWB detection ('useTwb')\n\nPlease check the options.\n\n" << options << "\n";
		exit(1);
	}
	if (vm.count("useTwb")) {
		if (!vm.count("markerId") && !vm.count("mmp")) {
			std::cout << "The navigation by TWB is actived. Please set the marker ID and the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		} else if (!vm.count("markerId")) {
			std::cout << "The navigation by TWB is actived. Please set the marker ID!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		} else if (!vm.count("mmp")) {
			std::cout << "The navigation by TWB is actived. Please set the meter per pixel factor!\nPlease check the options.\n\n" << options << "\n";
			exit(1);
		}
		useTwb = true;
	} else {
		useOdo = true;
	}


	// Set AMiRo name
	amiroName = "amiro" + boost::lexical_cast<std::string>(amiroID);

	// Define delays
	int delay = choreoDelay + idDelay*amiroID;
	int delayS = stepDelay + idDelay*amiroID;

	printChoreo = vm.count("printChoreo");


	INFO_MSG("Start TWB choreo algorithm with name '" << amiroName << "'.");

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

	// prepare RSB listener for choreos
	rsb::ListenerPtr choreoListener = factory.createListener(choreoInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> choreoQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	choreoListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(choreoQueue)));

	// prepare RSB listener for choreo corrections
	rsb::ListenerPtr choreoCorrectionListener = factory.createListener(choreoCorrectionInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::PoseList>>>choreoCorrectionQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::PoseList>>(1));
	choreoCorrectionListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::PoseList>(choreoCorrectionQueue)));

	// prepare RSB listener for commands
	rsb::ListenerPtr commandListener = factory.createListener(commandInscope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> commandQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(commandQueue)));

	// prepare rsb listener for tracking data
	rsb::ListenerPtr trackingListener = factory.createListener(twbTrackingProcess::getTrackingScope(), twbTrackingProcess::getTrackingRSBConfig());
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>>trackingQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>(1));
	trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));

	// prepare RSB listener for amiros
	rsb::ListenerPtr amiroListener = factory.createListener(amiroScope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> amiroQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(50));
	amiroListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(amiroQueue)));

	// prepare RSB listener for amiro logouts
	rsb::ListenerPtr amiroLogoutListener = factory.createListener(amiroLogoutScope, tmpPartConf);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> amiroLogoutQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(50));
	amiroLogoutListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(amiroLogoutQueue)));

	// ------------ Informer -----------------

	// prepare RSB informer for amiros
	rsb::Informer<std::string>::Ptr amiroInformer = factory.createInformer<std::string> (amiroScope, tmpPartConf);

	// prepare RSB informer for amiro logouts
	rsb::Informer<std::string>::Ptr amiroLogoutInformer = factory.createInformer<std::string> (amiroLogoutScope, tmpPartConf);

	// prepare RSB informer for goal position publication
	rsb::Informer<twbTracking::proto::Object>::Ptr goalInformer = factory.createInformer<twbTracking::proto::Object> (goalScope, tmpPartConf);

	// prepare RSB informer for setting lights
	rsb::Informer< std::vector<int> >::Ptr lightInformer = factory.createInformer< std::vector<int> > (lightOutscope);



	// Initialize the robot communication
	std::vector<std::string> amiros;
	bool initialized = false;
	bool exitProg = false;
	bool ledsLeft = true;
	int waitCounter = 0;
	INFO_MSG("");
	INFO_MSG("");
	INFO_MSG("Initializing:");
	// set lights
	setLights(lightInformer, LightModel::LightType::SINGLE_WARNING, amiro::Color(255,255,0), 600);
	setSpeeds(0.08, 0, myCAN);
	usleep(1500000);
	setSpeeds(-0.08, 0, myCAN);
	usleep(1500000);
	setSpeeds(0, 0, myCAN);

	while (!initialized) {
		// check amiro queue
		if (!amiroQueue->empty()) {
			EventPtr event = amiroQueue->pop(0);
			std::string inputName = *static_pointer_cast<std::string>(event->getData());
			if (amiroName != inputName) {
				bool isIn = false;
				for (std::string name : amiros) {
					if (name == inputName) {
						isIn = true;
						break;
					}
				}
				if (!isIn) {
					amiros.push_back(inputName);
					INFO_MSG(" -> Recognized AMiRo: '" << inputName << "'");
				}
			}
		}

		// check command queue
		initialized = checkCommands(commandQueue) == cmd_init;

		// inform other amiros
		if (waitCounter >= 5) {
			waitCounter = 0;
			boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
			amiroInformer->publish(StringPtr);
		} else {
			waitCounter++;
		}

		// wait 100 ms
		usleep(100000);
	}

	bool amiroCheck[amiros.size()];
	for (int i=0; i<amiros.size(); i++) {
		amiroCheck[i] = false;
	}

	// initialize the robots attributes
	light_t lights;
	system_clock::time_point *startSystemTime = new system_clock::time_point(system_clock::now());
	srand(time(NULL));

	bool quitProg = false;
	while (!quitProg) {
		// delete all old messages
		while (!commandQueue->empty()) {
			commandQueue->pop(0);
		}

		// set lights
		setLights(lightInformer, LightModel::LightType::SINGLE_SHINE, amiro::Color(255,255,255), 0);

		// wait for start command
		bool startHarvest = false;
		INFO_MSG("");
		INFO_MSG("Waiting for commands ...");
		while (!startHarvest) {
			// check choreo queue
			if (!choreoQueue->empty()) {
				EventPtr event = choreoQueue->pop(0);
				choreoName = *static_pointer_cast<std::string>(event->getData());
				INFO_MSG(" -> Choreo name has been changed to '" << choreoName << "'");
			}

			// check command queue
			uint64_t receiveTime;
			CommandId commandId = checkCommands(commandQueue, receiveTime);
			if (commandId == cmd_start) {
				startHarvest = true;
				delete startSystemTime;
				startSystemTime = new system_clock::time_point(microseconds(receiveTime) + milliseconds(delay));
			} else if (commandId == cmd_stop) {
				exitProg = true;
				INFO_MSG("");
				INFO_MSG("Exit TWB Choreo Tool ...");
				break;
			}
			
			// wait for 100 ms
			usleep(100000);
		}
		if (exitProg) {
			quitProg = true;
			break;
		}

		// load choreo
		Choreo choreo;
		if (startPosition) {
			choreo = loadChoreo(choreoName, startX, startY, startTheta);
		} else if (useTwb) {
			saveOldPos();
			odoPos = toOwnPos(twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID));
			if (odoPos[2] < 0) return failureProc(lightInformer, amiroLogoutInformer);
			correctPosition();
			choreo = loadChoreo(choreoName, odoPos[0], odoPos[1]);
		} else {
			// reset odometry
			odoPos[0] = 0;
			odoPos[1] = 0;
			odoPos[2] = 0;
			types::position pos_t;
			pos_t.x = (int)(odoPos[0]*TO_MICRO);
			pos_t.y = (int)(odoPos[1]*TO_MICRO);
			pos_t.f_z = (int)(odoPos[2]*TO_MICRO);
			myCAN.setOdometry(pos_t);
			choreo = loadChoreo(choreoName);
		}
		INFO_MSG("");
		string frontPart = " => Start choreo '" + choreoName + "': ";
		if (choreo.empty()) {
			ERROR_MSG(frontPart << "ERROR in Choreo!");
			continue;
		} else {
			INFO_MSG(frontPart << choreo.size() << " steps");
		}

		// wait for choreo to begin
		setLights(lightInformer, LightModel::LightType::SINGLE_WARNING, amiro::Color(255,255,0), 600);
		if (vm.count("verbose")) DEBUG_MSG("Waiting ...");
		boost::this_thread::sleep_until(*startSystemTime);
		if (vm.count("verbose")) DEBUG_MSG("Start choreo ...");

		// Clear amiro queue
		while (!amiroQueue->empty()) {
			amiroQueue->pop(0);
		}

		// perform the choreo
		doChoreo = true;
		pauseChoreo = false;
		correctChoreo = false;
		stepNumber = 0;
		while (doChoreo && stepNumber < choreo.size()) {
			if (correctChoreo) {
				pauseChoreo = false;
				correctChoreo = false;
				boost::shared_ptr<twbTracking::proto::PoseList> poses(new twbTracking::proto::PoseList);
				if (fakeCorrection) {
					fakeCorrection = false;
					twbTracking::proto::Pose *pose1 = poses->add_pose();
					twbTracking::proto::Pose *pose2 = poses->add_pose();
					twbTracking::proto::Pose *pose3 = poses->add_pose();
					twbTracking::proto::Translation *trans1 = pose1->mutable_translation();
					twbTracking::proto::Translation *trans2 = pose2->mutable_translation();
					twbTracking::proto::Translation *trans3 = pose3->mutable_translation();
					twbTracking::proto::Rotation *rot1 = pose1->mutable_rotation();
					twbTracking::proto::Rotation *rot2 = pose2->mutable_rotation();
					twbTracking::proto::Rotation *rot3 = pose3->mutable_rotation();
					trans1->set_x(odoPos[0]);
					trans1->set_y(odoPos[1] - 0.3);
					trans1->set_z(0.0);
					rot1->set_x(0.0);
					rot1->set_y(0.0);
					rot1->set_z(0.0);
					trans2->set_x(odoPos[0]);
					trans2->set_y(odoPos[1] + 0.3);
					trans2->set_z(0.0);
					rot2->set_x(0.0);
					rot2->set_y(0.0);
					rot2->set_z(180.0);
					trans3->set_x(odoPos[0]);
					trans3->set_y(odoPos[1]);
					trans3->set_z(0.0);
					rot3->set_x(0.0);
					rot3->set_y(0.0);
					rot3->set_z(odoPos[3]);
				} else {
//					poses = boost::static_pointer_cast<twbTracking::proto::PoseList>(choreoCorrectionQueue->pop());
					poses = poses_include;
				}
				choreo = insertPoseListIntoChoreo(choreo, stepNumber, poses);
			}
				

			if (pauseChoreo) {
				choreoSleep_ms(100, commandQueue, choreoCorrectionQueue);
				continue;
			}

			// load choreo step
			ChoreoStep cs = choreo[stepNumber];

			// get current position
			if (useTwb) {
				saveOldPos();
				odoPos = toOwnPos(twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID));
				if (odoPos[2] < 0) return failureProc(lightInformer, amiroLogoutInformer);
				correctPosition();
			}

			// send goal position via RSB
			boost::shared_ptr<twbTracking::proto::Object> goalObject(new twbTracking::proto::Object);
			goalObject->set_id(amiroID);
			goalObject->set_type(twbTracking::proto::ArMarker);
			goalObject->set_unit(twbTracking::proto::Meter);
			twbTracking::proto::Pose *objPos = goalObject->mutable_position();
			twbTracking::proto::Translation *objTrans = objPos->mutable_translation();
			objTrans->set_x(cs.position[0]);
			objTrans->set_y(cs.position[1]);
			objTrans->set_z(0.0);
			twbTracking::proto::Rotation *objRot = objPos->mutable_rotation();
			objRot->set_x(0.0);
			objRot->set_y(0.0);
			objRot->set_z(cs.position[2]);
			goalInformer->publish(goalObject);	

			// get relative distances and goal angle
			float moveDirX = cs.position[0] - odoPos[0];
			float moveDirY = cs.position[1] - odoPos[1];
			float goalAngle = cs.position[2];

			// set lights
			setLightsVec(lightInformer, LightModel::LightType::SINGLE_SHINE, cs.lights, 0);

			// set motors
			if (useOdo) {
				// calculate speeds
				float v, w, time, dist, angle;
				calcSpeeds(odoPos, moveDirX, moveDirY, v, w, time, dist, angle);

				// print action messages
				if (vm.count("verbose")) {
					DEBUG_MSG("------------------------------------------------------------");
					// print the CAN values to output
					DEBUG_MSG("own position: " << odoPos[0] << "/" << odoPos[1] << ", " << odoPos[2]);
					DEBUG_MSG("goal position: " << cs.position[0] << "/" << cs.position[1]);
					DEBUG_MSG("v: " << v << " m/s");
					DEBUG_MSG("w: " << w << " rad/s");
					DEBUG_MSG("time: " << time << " s");
				}

				// set speed
				if ((int)(time*TO_MICRO) - timebuffer/2 > 0) {
					setSpeeds(v, w, myCAN);
					usleep((int)(time*TO_MICRO) - timebuffer/2);
				}

				// set position
				types::position pos_t;
				pos_t.x = (int)(cs.position[0] * TO_MICRO);
				pos_t.y = (int)(cs.position[1] * TO_MICRO);
				pos_t.f_z = (int)(odoPos[2]*TO_MICRO + angle*TO_MICRO);
				myCAN.setOdometry(pos_t);

			} else if (useTwb) {
				position_t goalPos;
				goalPos[0] = odoPos[0] + moveDirX;
				goalPos[1] = odoPos[1] + moveDirY;
				float angleToGoal = normAngle(atan2(moveDirY, moveDirX));

				// move to position
				while (doChoreo && !pauseChoreo && (abs(moveDirX) > trackingVariance || abs(moveDirY) > trackingVariance)) {
					// calculate speeds
					float v, w, time, dist, angle;
					trackingSpeeds(odoPos, moveDirX, moveDirY, cs.directMovement, v, w);
					setSpeeds(v, w, myCAN);

					// print action messages
					if (vm.count("verbose")) {
						DEBUG_MSG("------------------------------------------------------------");
						// print the CAN values to output
						DEBUG_MSG("own position: " << odoPos[0] << "/" << odoPos[1] << ", " << odoPos[2]);
						DEBUG_MSG("goal position: " << goalPos[0] << "/" << goalPos[1]);
						DEBUG_MSG("v: " << v << " m/s");
						DEBUG_MSG("w: " << w << " rad/s");
						DEBUG_MSG("time: " << time << " s");
					}

					// sleep 100 ms
					choreoSleep_ms(100, commandQueue, choreoCorrectionQueue);

					// get new position
					saveOldPos();
					odoPos = toOwnPos(twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID));
					if (odoPos[2] < 0) return failureProc(lightInformer, amiroLogoutInformer);
					correctPosition();
					moveDirX = goalPos[0] - odoPos[0];
					moveDirY = goalPos[1] - odoPos[1];
				}

				// set final orientation
				while (doChoreo && !pauseChoreo && cs.directMovement && fabs(angleDiff(odoPos[2], goalAngle)) > trackingAngleVar) {
					if (angleDiff(odoPos[2], goalAngle) < 0) {
						setSpeeds(0, -turnSpeed, myCAN);
					} else {
						setSpeeds(0, turnSpeed, myCAN);
					}

					// sleep for 100 ms
					choreoSleep_ms(100, commandQueue, choreoCorrectionQueue);

					// get next position
					odoPos = toOwnPos(twbTrackingProcess::getNextTrackingObject(trackingQueue, markerID));
					if (odoPos[2] < 0) return failureProc(lightInformer, amiroLogoutInformer);
				}
			}


			// step is done, so rise the step number
			if (doChoreo && !pauseChoreo) stepNumber++;
			// wait for others
			if (doChoreo && !pauseChoreo && cs.braking) {
				// stop motors
				setSpeeds(0, 0, myCAN);

				// reset amiro flags
				for (int i=0; i<amiros.size(); i++) {
					amiroCheck[i] = false;
				}

				// Check for all ready amiros
				if (vm.count("verbose")) DEBUG_MSG("List of AMiRos which are already finished:");
				while (doChoreo && !amiroQueue->empty()) {
					EventPtr event = amiroQueue->pop();
					std::string inputName = *static_pointer_cast<std::string>(event->getData());
					bool isIn = false;
					int id = -1;
					for (int i=0; i<amiros.size(); i++) {
						if (amiros[i] == inputName) {
							isIn = true;
							id = i;
							break;
						}
					}
					if (isIn && !amiroCheck[id]) {
						amiroCheck[id] = true;
						if (vm.count("verbose")) DEBUG_MSG(" -> Ready AMiRo: '" << inputName.c_str() << "'");
					}
				}
				while (doChoreo && !amiroLogoutQueue->empty()) {
					EventPtr event = amiroLogoutQueue->pop();
					std::string inputName = *static_pointer_cast<std::string>(event->getData());
					bool isIn = false;
					int id = -1;
					for (int i=0; i<amiros.size(); i++) {
						if (amiros[i] == inputName) {
							isIn = true;
							id = i;
							break;
						}
					}
					if (isIn) {
						int newIdx = 0;
						std::vector<std::string> newAmiros;
						for (int curIdx=0; curIdx<amiros.size(); curIdx++) {
							if (amiros[curIdx] != inputName) {
								newAmiros.push_back(amiros[curIdx]);
								amiroCheck[newIdx] = amiroCheck[curIdx];
								newIdx++;
							}
						}
						amiros = newAmiros;
						WARNING_MSG("AMiRo Logout: '" << inputName.c_str() << "'");
					}
				}
				choreoSleep_ms(100, commandQueue, choreoCorrectionQueue);

				// check for rest (and own)
				bool ownNameReceived = false;
				bool allOthersReceived = false;
				if (vm.count("verbose")) DEBUG_MSG("Waiting for rest:");
				int publishCounter = 5;
				// set lights
				setLights(lightInformer, LightModel::LightType::SINGLE_WARNING, amiro::Color(255,255,0), 600);
				while (doChoreo && (!ownNameReceived || !allOthersReceived)) {
					// Send own name
					boost::shared_ptr<std::string> StringPtr(new std::string(amiroName));
					amiroInformer->publish(StringPtr);

					// check for rest
					while (doChoreo && !amiroQueue->empty()) {
						EventPtr event = amiroQueue->pop();
						std::string inputName = *static_pointer_cast<std::string>(event->getData());
						if (amiroName != inputName) {
							bool isIn = false;
							int id = -1;
							for (int i=0; i<amiros.size(); i++) {
								if (amiros[i] == inputName) {
									isIn = true;
									id = i;
									break;
								}
							}
							if (isIn && !amiroCheck[id]) {
								amiroCheck[id] = true;
								if (vm.count("verbose")) DEBUG_MSG(" -> Received AMiRo: '" << inputName << "'");
								delete startSystemTime;
								startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delayS));
							}
						} else if (amiroName == inputName && !ownNameReceived) {
							ownNameReceived = true;
							if (vm.count("verbose")) DEBUG_MSG(" -> Received AMiRo: '" << inputName << "' => own name!");
							delete startSystemTime;
							startSystemTime = new system_clock::time_point(microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delayS));
						}
					}
					while (doChoreo && !amiroLogoutQueue->empty()) {
						EventPtr event = amiroLogoutQueue->pop();
						std::string inputName = *static_pointer_cast<std::string>(event->getData());
						bool isIn = false;
						int id = -1;
						for (int i=0; i<amiros.size(); i++) {
							if (amiros[i] == inputName) {
								isIn = true;
								id = i;
								break;
							}
						}
						if (isIn) {
							int newIdx = 0;
							std::vector<std::string> newAmiros;
							for (int curIdx=0; curIdx<amiros.size(); curIdx++) {
								if (amiros[curIdx] != inputName) {
									newAmiros.push_back(amiros[curIdx]);
									amiroCheck[newIdx] = amiroCheck[curIdx];
									newIdx++;
								}
							}
							amiros = newAmiros;
							WARNING_MSG("AMiRo Logout: '" << inputName.c_str() << "'");
						}
					}

					allOthersReceived = true;
					for (int i=0; i<amiros.size(); i++) {
						if (!amiroCheck[i]) {
							allOthersReceived = false;
							break;
						}
					}

					// wait for 300 ms
					choreoSleep_ms(300, commandQueue, choreoCorrectionQueue);
				}
				// Clear amiro queue
				while (!amiroQueue->empty()) {
					amiroQueue->pop();
				}

				// Set LEDs
				setLights(lightInformer, LightModel::LightType::SINGLE_SHINE, amiro::Color(255,255,0), 0);

				// to continue just wait until deadline
				if (stepNumber < choreo.size()) {
					if (vm.count("verbose")) DEBUG_MSG("All are finished. Waiting ...");
					boost::this_thread::sleep_until(*startSystemTime);
					if (vm.count("verbose")) DEBUG_MSG("Continue choreo ...");
				} else {
					if (vm.count("verbose")) DEBUG_MSG("All are finished.");
				}

				// Clear amiro queue again (there have been messages again)
				while (!amiroQueue->empty()) {
					amiroQueue->pop(0);
				}
			} else if (doChoreo && stepNumber < choreo.size()) {
				if (vm.count("verbose")) DEBUG_MSG("No brake, continue ...");
			} else if (doChoreo) {
				if (vm.count("verbose")) DEBUG_MSG("No brake.");
			}
			
			if (pauseChoreo) {
				setSpeeds(0, 0, myCAN);
				setLights(lightInformer, LightModel::LightType::SINGLE_SHINE, amiro::Color(255,0,0), 0);
			}
		}

		// stop Robot
		setSpeeds(0, 0, myCAN);

		// rest print
		if (vm.count("verbose")) {
			DEBUG_MSG("------------------------------------------------------------");
		}

		if (!doChoreo) {
			INFO_MSG("The choreo has been stopped!");
		}

		// set lights
		setLights(lightInformer, LightModel::LightType::SINGLE_SHINE, amiro::Color(255,255,255), 0);
	}

	// set lights
	setLights(lightInformer, LightModel::LightType::SINGLE_INIT, amiro::Color(0,0,0), 0);

	return EXIT_SUCCESS;
}
