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

using namespace std;
using namespace muroxConverter;

// Settings
const int ANGULAR_VEL = 250000;

// Constants
const int MIDDLE_RIGHT = 0, RIGHT = 1, LEFT = 2, MIDDLE_LEFT = 3;
const std::string sensorNames[] = {"middle right", "right", "left", "middle left"};

// Datastructure for the CAN messages

int floorProxMinValues[4] = {3200, 3100, 3300, 4900};
int floorProxMaxValues[4] = {31000, 22200, 28200, 34200};
bool lineBelow[4] = {false, false, false, false};

bool turning = true; // will lead to immediate state change to driving (!turning)
int forward_vel, angular_vel;
float floorProxValuesNormalized[4];

void updateSensors(rsb::EventPtr senseFloorProxEvent) {
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
  
  std::string outScopeBehavior = "/motor/00";
  std::string rsbInScope = "/prox/floor";
  std::string outScopeDetection = "/edge";
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&outScopeBehavior), "Scope for sending motor commands to motorControl")
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
  rsb::Factory& factory = rsb::getFactory();

  // Prepare RSB reader
  rsb::ReaderPtr floorProxSensorReader = factory.createReader(rsbInScope);

  // Prepare RSB informer
  rsb::Informer<std::vector<int> >::Ptr motorCmdInformer = factory.createInformer<std::vector<int> > (outScopeBehavior);
  rsb::Informer<bool>::Ptr edgeDetectionInformer = factory.createInformer<bool> (outScopeDetection);

  std::vector<int> motorCmd(3,0);
  int duration;
  bool sendCmd;

  while(true) {

	  sendCmd = false;
	  updateSensors(floorProxSensorReader->read()); // blocking

	  if (turning) {
		  if (!lineBelow[MIDDLE_RIGHT] || !lineBelow[MIDDLE_LEFT]) { // if no line in front anymore stop turning // OR instead
			  boost::shared_ptr<bool> lineOnRight = boost::shared_ptr<bool>(new bool(lineBelow[MIDDLE_RIGHT]));
			  //edgeDetectionInformer->publish(lineOnRight);
			  forward_vel = 0;
			  angular_vel = 0;
			  duration = 0; // stop immediately
			  sendCmd = true;
		  }
	  } else {
		  if (lineBelow[MIDDLE_RIGHT] || lineBelow[MIDDLE_LEFT]) { // if line in front start turning
			  forward_vel = 0;
			  angular_vel = ANGULAR_VEL * ((lineBelow[RIGHT] && !lineBelow[LEFT]) || (!lineBelow[RIGHT] && !lineBelow[LEFT] && floorProxValuesNormalized[MIDDLE_RIGHT] < floorProxValuesNormalized[MIDDLE_LEFT]) ? 1 : -1);
			  duration = 5000000; // 5s
			  sendCmd = true;
		  }
	  }
	  if (sendCmd) {
		  turning = !turning; // switch state: turning <-> driving
		  INFO_MSG("State change: " << (turning ? "TURNING" : "DRIVING"));

		  motorCmd[0] = forward_vel;
		  motorCmd[1] = angular_vel;
		  motorCmd[2] = duration;
		  boost::shared_ptr< std::vector<int> > motorCmdData = boost::shared_ptr<std::vector<int> >(new std::vector<int>(motorCmd.begin(),motorCmd.end()));
		  motorCmdInformer->publish(motorCmdData);
	  }

	  // Sleep for a while
	  boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

  return EXIT_SUCCESS;
}
