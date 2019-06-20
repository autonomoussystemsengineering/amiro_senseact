#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

#include <kbhit.hpp>

using namespace std;
using namespace muroxConverter;

// Settings
const int ANGULAR_VEL = 500000, FORWARD_VEL = 50000;

// Constants
const int MIDDLE_RIGHT = 0, RIGHT = 1, LEFT = 2, MIDDLE_LEFT = 3;
const std::string sensorNames[] = {"middle right", "right", "left", "middle left"};

// Datastructure for the CAN messages

int floorProxMinValues[4] = {0, 3100, 3300, 4900};
int floorProxMaxValues[4] = {38000, 22200, 28200, 34200};
bool lineBelow[4] = {false, false, false, false};

bool turning = true; // will lead to immediate state change to driving (!turning)
int forward_vel, angular_vel;

void updateSensors(rsb::EventPtr senseFloorProxEvent) {
	float floorProxValuesNormalized[4];
	std::vector<int> floorProxValues = *(static_pointer_cast<std::vector<int> >(senseFloorProxEvent->getData()));
	for (uint8_t sensorIdx = 0; sensorIdx < floorProxValues.size(); sensorIdx++) {
		floorProxValuesNormalized[sensorIdx] = (floorProxValues[sensorIdx] - floorProxMinValues[sensorIdx])/((float) floorProxMaxValues[sensorIdx]);
		if (lineBelow[sensorIdx]) {
			if (floorProxValuesNormalized[sensorIdx] > 0.6f) lineBelow[sensorIdx] = false;
		} else {
			if (floorProxValuesNormalized[sensorIdx] < 0.4f) lineBelow[sensorIdx] = true;
		}
//		INFO_MSG(sensorNames[sensorIdx] << ":: range: [" << floorProxMinValues[sensorIdx] << "," << floorProxMaxValues[sensorIdx] << "], absolute: " << floorProxValues[sensorIdx] << ", normalized: " << floorProxValuesNormalized[sensorIdx] * 100);
	}
}

int main(int argc, char **argv) {
  
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScope = "/motor/01";
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending motor commands to motorControl")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)");

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

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converter);
  
  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Prepare RSB informer
  rsb::Informer< std::vector<int> >::Ptr motorCmdInformer = factory.createInformer< std::vector<int> > (rsbOutScope);

  std::vector<int> motorCmd(3,0);

  boost::uint64_t stopTime = -1;
  int KB_code = 0, step_size_forward_vel = 10000, angular_vel_when_turning = 500000, duration_for_turning = 500000;

  while(KB_code != KB_ESCAPE)
  {
	  if (kbhit())
	  {
		  KB_code = getchar();
		  motorCmd[2] = 1000000000; // 10 seconds
		  INFO_MSG( "KB_code = " << KB_code )

		  switch (KB_code)
		  {
		  case KB_A:
			  // Degree per second
			  motorCmd[1] = angular_vel_when_turning;
			  stopTime = rsc::misc::currentTimeMicros() + duration_for_turning;
			  break;

		  case KB_D:
			  // Degree per second
			  motorCmd[1] = -angular_vel_when_turning;
			  stopTime = rsc::misc::currentTimeMicros() + duration_for_turning;
			  break;

		  case KB_W:
			  // Milimeter per second
			  motorCmd[0] = min(100000, motorCmd[0] + step_size_forward_vel);
			  break;

		  case KB_S:
			  // Milimeter per second
			  motorCmd[0] = max(-50000, motorCmd[0] - step_size_forward_vel);
			  break;
		  case KB_SPACE:
			  motorCmd[0] = 0;
			  motorCmd[1] = 0;
			  motorCmd[2] = 0;
			  break;
		  }
		  boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
		  motorCmdInformer->publish(motorCmdData);
	  }
	  if ((stopTime > 0 && rsc::misc::currentTimeMicros() > stopTime)) {
		  motorCmd[0] = 0;
		  motorCmd[1] = 0;
		  boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
		  motorCmdInformer->publish(motorCmdData);
		  stopTime = -1;
	  }
  }

  return EXIT_SUCCESS;
}
