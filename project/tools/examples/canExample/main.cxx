//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Example of CAN communication
//============================================================================


 #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <ControllerAreaNetwork.h>

#include <vector>
//#include "ControllerAreaNetwork.h"
#include <iostream>

#include <Types.h>  // types::position

using namespace std;
using namespace amiro;


/* omitted vital #includes and error checking */


int main(int argc, char **argv)
{

  ControllerAreaNetwork myCAN;

  types::position robotPosition;
  robotPosition.x = 0;
  robotPosition.y = 0;
  robotPosition.f_z = 0;
  myCAN.setOdometry(robotPosition);


 INFO_MSG( "Drive forward with 10 cm/s and 1 rad/s" )
 int32_t v = 100000, w = 100000;
 myCAN.setTargetSpeed(v,w);


 INFO_MSG ("Reading the odometry for 20 times:" );
 for (int idx = 0; idx < 20 ; ++idx ){
   types::position robotPosition = myCAN.getOdometry();
   INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z)
 }



 INFO_MSG ("Reading the odometry for 20 times:" );
 for (int idx = 0; idx < 20 ; ++idx ){
   types::position robotPosition = myCAN.getOdometry();
   INFO_MSG(" x: " << robotPosition.x << " y: " << robotPosition.y << " f_z: " << robotPosition.f_z)
 }

 INFO_MSG ("Drive stop" )
 v = 0, w = 0;
 myCAN.setTargetSpeed(v,w);



 return 0;

}
