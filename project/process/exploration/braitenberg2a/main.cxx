
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Perform the Braitenberg 2a vehicle on the AMiRo
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <ControllerAreaNetwork.h>

using namespace std;

int main (int argc, const char **argv){
 
  INFO_MSG("Start the Braitenberg behaviour")

  // Init the CAN interface
  ControllerAreaNetwork CAN;    

  // Datastructure for the proximity values
  std::vector< uint16_t > proximity(8,0), minProximity(8,2500), diffProximity(8,0);

  // Set connections between the input (proximity) and output (steering)
  const std::vector< int32_t > connections_v = {200, 55, -650, -3200, -3200, -650, 55, 200};
  const std::vector< int32_t > connections_w = {-50, -100, -12000, -40000, 40000, 12000, 100, 50};

  int v=0; // Velocity in µm/s
  int w=0; // Angularvelocity in µrad/s
  // Process the incomming proximity values and perform the motor commands in (µm/s, µrad/s)
  for(;;) {
    // Get the proximity values
    const int fail = CAN.getProximityRingValue(proximity);

    if (fail == 0) { // Process the data if everything went fine
      // Preprocess the incomming proximity values
      for ( uint8_t sensorIdx = 0; sensorIdx < proximity.size(); ++sensorIdx) {
        // Because each proximity sensor gives the mininum value different from each other,
        // the mininum Proximity will be determined and saved.
        if ((proximity[sensorIdx] < minProximity[sensorIdx]) && (proximity[sensorIdx] != 0)) {
          minProximity[sensorIdx] = proximity[sensorIdx];
        }
        // To calculate the differential proximity value
        diffProximity[sensorIdx] = proximity[sensorIdx] - minProximity[sensorIdx];
        // Print out the unbiasd proximity values
        INFO_MSG( (int) sensorIdx << ": " << diffProximity[sensorIdx])
      }

      // Reset the steering and map the incomming proximity to the steering
      v = 200000;
      w = 0;

      for (uint8_t sensorIdx = 0; sensorIdx < proximity.size(); ++sensorIdx) {
        v += connections_v[sensorIdx] * sqrt(diffProximity[sensorIdx]);
        w += connections_w[sensorIdx] * sqrt(diffProximity[sensorIdx]);
      }

      // To avoid the standstill and backward drive for braitenberg vehicle
      if (v  < 0)
        v = 0;
      else if ((v  < 5000) && (w < 8000) && (w >= 0))
        w = 8000;
      else if ((v  < 5000) && (w < 0) && (w > -8000))
        w = -8000;

    } else { // Stop if we have an invalid reading
      v=0;
      w=0;
    }

    // Set the target Velocities
    INFO_MSG( "Target speed: v=" << static_cast<float>(v) * 1e-6 << " m/s, w=" << static_cast<float>(w) * 1e-6 << " rad/s")
    CAN.setTargetSpeed(v, w);
  }

  return EXIT_SUCCESS;
}  