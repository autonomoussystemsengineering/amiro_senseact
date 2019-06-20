//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography.
//============================================================================

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
using namespace rsb;

// types
#include <ControllerAreaNetwork.h>

#include "choreo.h"
#include <Types.h>

// initialize CAN
ControllerAreaNetwork myCAN;

// variables for homing procedure
bool doHoming = false;
types::position homingPosition;

int getForwardVel(float distance, float duration) {
	return round(distance * 10000 / duration);
}

int getAngularVel(float angle, float duration) {
	return round(angle / 180.0f * M_PI * 1000000 / duration);
}

// load a choreography from a file
Choreo loadChoreo(std::string choreoName) {
	Choreo choreo;
	using boost::property_tree::ptree;
	ptree pt;

	try {

		read_xml(choreoName, pt);
		BOOST_FOREACH( ptree::value_type const&tree, pt.get_child("choreo")) {
			if (tree.first == "choreoStep") {
				ChoreoStep choreoStep;
				float duration = tree.second.get<float>("time");
				choreoStep.v = getForwardVel(tree.second.get<float>("v"), duration);
				choreoStep.w = getAngularVel(tree.second.get<float>("w"), duration);
				choreoStep.brightness = tree.second.get<int>("brightness");
				light_t lights;
				for (int i = 0; i < 8; ++i) {
					std::vector<std::string> splitstring;
					std: string field = "l" + boost::lexical_cast<std::string>(i + 1);
					std::string l = tree.second.get<std::string>(field);
					splitstring.clear();
					boost::split(splitstring, l, boost::is_any_of(","));
					lights[i][0] = boost::lexical_cast<int>(splitstring[2]);
					lights[i][1] = boost::lexical_cast<int>(splitstring[1]);
					lights[i][2] = boost::lexical_cast<int>(splitstring[0]);
				}
				choreoStep.lights = lights;
				choreoStep.time = round(duration * 1000);
				choreo.push_back(choreoStep);
			}
			if (tree.first == "choreoinclude") {
				std::string newfile = tree.second.get<std::string>("choreopart");
				ptree pt2;
				read_xml(newfile, pt2);
				BOOST_FOREACH( ptree::value_type const&tree, pt2.get_child("choreo")) {
					if (tree.first == "choreoStep") {
						ChoreoStep choreoStep1;
						float duration1 = tree.second.get<float>("time");
						choreoStep1.v = getForwardVel(tree.second.get<float>("v"), duration1);
						choreoStep1.w = getAngularVel(tree.second.get<float>("w"), duration1);
						choreoStep1.brightness = tree.second.get<int>("brightness");
						light_t lights1;
						for (int i = 0; i < 8; ++i) {
							std::vector<std::string> splitstring;
							std::string field = "l" + boost::lexical_cast<std::string>(i + 1);
							std::string l = tree.second.get<std::string>(field);
							splitstring.clear();
							boost::split(splitstring, l, boost::is_any_of(","));
							lights1[i][0] = boost::lexical_cast<int>(splitstring[2]);
							lights1[i][1] = boost::lexical_cast<int>(splitstring[1]);
							lights1[i][2] = boost::lexical_cast<int>(splitstring[0]);
						}
						choreoStep1.lights = lights1;
						choreoStep1.time = round(duration1 * 1000);
						choreo.push_back(choreoStep1);
					}
				}
			}
		}
	} catch (...) {
	}
	return choreo;
}

void moveToTargetPosition(types::position position, int16_t timeMS) {
	// initialize velocities
	int32_t v = 0;
	int32_t w = 0;

	// send target position (if not set, then again)
	do {
		myCAN.setTargetPosition(position, timeMS);
		usleep(500000);
		myCAN.getActualSpeed(v, w);
		usleep(250000);
	} while (v == 0 && w == 0);

	// wait for finished movement
	// TODO implement better termination condition when possible by CAN interface
	do {
		usleep(250000);
		myCAN.getActualSpeed(v, w);
		if (v == 0 && w == 0) {
			sleep(1);
			myCAN.getActualSpeed(v, w);
		}
	} while (v != 0 || w != 0);
}

void homing() {
	// get odometry data
	homingPosition = myCAN.getOdometry();

	// calculate angle from robot position (without respect to orientation) to origin (in rad)
	double angleOdo = atan2((double) homingPosition.y, (double) homingPosition.x) + M_PI;
	if (angleOdo > 2 * M_PI) {
		angleOdo -= 2 * M_PI;
	}

	// calculate difference angle (in urad) to turn towards origin
	int diffAngle = (int) (angleOdo * 1e6) - homingPosition.f_z;

	// calculate angle (in urad) to turn to origin orientation after reaching origin
	int backAngle = -homingPosition.f_z - diffAngle;
	while (backAngle < 0) {
		backAngle += 2 * M_PI * 1e6;
	}
	while (backAngle > M_PI * 1e6) {
		backAngle -= 2 * M_PI * 1e6;
	}

	// calculate distance between origin and robot position (in um)
	int diffDist = (int) sqrt(
			(double) homingPosition.y * (double) homingPosition.y
					+ (double) homingPosition.x * (double) homingPosition.x);

	// reset position struct
	homingPosition.y = 0;
	/*
	 // turn towards origin
	 homingPosition.x = 0;
	 homingPosition.f_z = diffAngle;
	 moveToTargetPosition(homingPosition, 1);
	 // drive to origin
	 homingPosition.x = diffDist;
	 homingPosition.f_z = 0;
	 moveToTargetPosition(homingPosition, 1);
	 // turn towards origin orientation
	 homingPosition.x = 0;
	 homingPosition.f_z = backAngle;
	 moveToTargetPosition(homingPosition, 1);
	 */
	int speed;
	// turn towards origin
	if (diffAngle < 0) {
		speed = -1e6;
	} else {
		speed = 1e6;
	}
	myCAN.setTargetSpeed(0, speed);
	usleep(abs(int(1.01f * diffAngle)));
	myCAN.setTargetSpeed(0, 0);
	// drive to origin
	if (diffDist < 0) {
		speed = -100e3;
	} else {
		speed = 100e3;
	}
	myCAN.setTargetSpeed(speed, 0);
	usleep(abs(diffDist * 11));
	myCAN.setTargetSpeed(0, 0);
	// turn towards origin orientation
	if (backAngle < 0) {
		speed = -1e6;
	} else {
		speed = 1e6;
	}
	myCAN.setTargetSpeed(0, speed);
	usleep(abs(int(1.01f * backAngle)));
	myCAN.setTargetSpeed(0, 0);
}

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string choreoInscope = "/choreo";

	// position (0=left, 1=middle left, 2=middle, 3=middle right, 4=right)
	int pos = 2;

	// delay to start the choreo after the rsb-event was created in ms
	int delay = 2000;

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("verbose,v", "Print values that are published via CAN.")(
			"choreoIn", po::value<std::string>(&choreoInscope), "Choreography inscope.")("delay",
			po::value<int>(&delay), "Delay between creating the rsb event and starting the choreography in ms.")("pos",
			po::value<int>(&pos),
			"Position in the formation (from the front: 0=left, 1=middle left, 2=middle, 3=middle right, 4=right)")(
			"homing,s",
			"The homing will be activated. After finishing the choreography the robot will return to its start position.");

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

	// check if homing should be done
	doHoming = vm.count("homing");

	// Get the RSB factory
#if RSB_VERSION_NUMERIC<1200
	rsb::Factory& factory = rsb::Factory::getInstance();
#else
	rsb::Factory& factory = rsb::getFactory();
#endif

	// prepare RSB listener for choreos
	rsb::ListenerPtr choreoListener = factory.createListener(choreoInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<EventPtr>> choreoQueue(
			new rsc::threading::SynchronizedQueue<EventPtr>(1));
	choreoListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(choreoQueue)));

	// initialize the robots attributes
	int v = 0;
	int w = 0;
	int brightness = 0;
	light_t lights;

	while (true) {
		cout << "Waiting for songname" << endl;
		// wait for a rsb message
		EventPtr event = choreoQueue->pop(0);

		// parse the choreography name
		std::string songName = *static_pointer_cast<std::string>(event->getData());

		// get the starting time
		system_clock::time_point nextStepTime(
				microseconds(event->getMetaData().getReceiveTime()) + milliseconds(delay));

		// load choreo from file
		std::stringstream fileNameStream;
		fileNameStream << songName << pos << ".xml";
		cout << "Filename: " << fileNameStream.str() << endl;
		Choreo choreo = loadChoreo(fileNameStream.str());

		if (choreo.empty()) {
			cout << "Choreo unknown!" << endl;
			continue;
		}

		// wait for choreo to begin
		boost::this_thread::sleep_until(nextStepTime);

		// reset odometry
		if (doHoming) {
			homingPosition.x = 0;
			homingPosition.y = 0;
			homingPosition.f_z = 0;
			myCAN.setOdometry(homingPosition);
		}
		// set lights
		for (int l = 0; l < 8; ++l) {
			myCAN.setLightColor(l, amiro::Color(255, 255, 255));
		}

		// perform the choreo
		for (ChoreoStep cs : choreo) {

			// update values only if they have changed

			// set steering
			if (v != cs.v || w != cs.w) {
				v = cs.v;
				w = cs.w;
				myCAN.setTargetSpeed(v, w);
			}

			// set brightness
			if (brightness != cs.brightness) {
				brightness = cs.brightness;
				myCAN.setLightBrightness(cs.brightness);
			}

			// set lights
			for (int l = 0; l < 8; ++l) {
				if (lights[l] != cs.lights[l]) {
					lights[l] = cs.lights[l];
					myCAN.setLightColor(l, amiro::Color(lights[l][0], lights[l][1], lights[l][2]));
				}
			}

			if (vm.count("verbose")) {
				// print the CAN values to output
				cout << "v: " << cs.v << " w: " << cs.w << " brightness: " << cs.brightness << " lights: ";
				for (int l = 0; l < 8; ++l) {
					cout << "[" << cs.lights[l][0] << "," << cs.lights[l][1] << "," << cs.lights[l][2] << "]";
				}
				cout << endl;
			}

			// wait till this step is finished
			nextStepTime += milliseconds(cs.time);
			boost::this_thread::sleep_until(nextStepTime);
		}

		// stop Robot
		myCAN.setTargetSpeed(0, 0);

		// reset lights
		for (int l = 0; l < 8; ++l) {
			myCAN.setLightColor(l, amiro::Color(255, 255, 255));
		}

		sleep(1);

		// do final homing
		if (doHoming) {
			homing();
		}
	}

	return EXIT_SUCCESS;
}
