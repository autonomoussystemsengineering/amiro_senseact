//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Stops the motors of the AMiRo and "resets" the lights.
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <actModels/lightModel.h>

using namespace std;

int main(int argc, char **argv) {
  // Init the CAN interface
  ControllerAreaNetwork CAN;    
  // Stop motor
  CAN.setTargetSpeed(0, 0);
  INFO_MSG("AMiRo stopped!");

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::WHITE));
  }

  sleep(1);

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, LightModel::initColors[led]);
  }
  INFO_MSG("AMiRo lights reset.");

  return EXIT_SUCCESS;
}
