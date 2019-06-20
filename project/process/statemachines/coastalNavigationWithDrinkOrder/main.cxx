//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Example of CAN communication
//============================================================================


// #define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>


// Reading inScope, outScope and freq
#include <ofFlags.hpp>

#include <ControllerAreaNetwork.h>

#include <vector>
//#include "ControllerAreaNetwork.h"
#include <iostream>

#include <Types.h>  // types::position
//#include <boost/thread.hpp>
//#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>
//#include <pthread.h>
//#include <signal.h>
#include <string>
//#include <fstream>

//#include <rsb/Informer.h>
//#include <rsb/Factory.h>
//#include <rsb/Event.h>
//#include <rsb/Handler.h>
//#include <rsb/converter/Repository.h>


using namespace std;
using namespace amiro;


/* omitted vital #includes and error checking */

enum STATES {INIT, COSTAL_NAV, NAV, CORNER_TURN_INIT, CORNER_TURN, BEER, COCKTAIL};
char *StateTypes[] ={"INIT","COSTAL_NAV", "NAV","CORNER_TURN_INIT","CORNER_TURN","BEER","COCKTAIL"};
STATES state = INIT;


ControllerAreaNetwork myCAN;
std::vector<uint16_t> proxRing(8,0);

void ringSensors() {
	   if (true) {
//				 for (int sensorIdx = 0; sensorIdx < proxRing.size(); sensorIdx++) {
//					if (proxRing[2] > 0x1000u)
//						state = INIT;
			if (proxRing[0] > 0x1000u || proxRing[1] > 0x1000u) {
				state = BEER;
				for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
					myCAN.setLightColor(ledIdx, amiro::Color::YELLOW);
				}
				myCAN.setTargetSpeed(0, 0); // STOP
				//timeout
				INFO_MSG("start sleep");
				sleep(5);
				INFO_MSG("end sleep");
				for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
					myCAN.setLightColor(ledIdx, amiro::Color::WHITE);
				}
			} else if (proxRing[6] > 0x1000u || proxRing[7] > 0x1000u) {
				state = COCKTAIL;
				for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
					myCAN.setLightColor(ledIdx, amiro::Color::DARKCYAN);
				}
				myCAN.setTargetSpeed(0, 0); // STOP
				//timeout
				INFO_MSG("start sleep");
				sleep(5);
				INFO_MSG("end sleep");
				for (int ledIdx = 0; ledIdx < 8; ++ledIdx) {
					myCAN.setLightColor(ledIdx, amiro::Color::WHITE);
				}
			} else if (proxRing[2] > 0x1000u || proxRing[3] > 0x1000u
				|| proxRing[4] > 0x1000u || proxRing[5] > 0x1000u) {
				// ToDo Collision avoidance
				state = INIT;
			}
//				 }
	   } else {
		 WARNING_MSG( "Fail" )
	   }
}

int main(int argc, char **argv)
{
	const int numSensors = 4;
	const int vNorm = 20000;
	int32_t v = 0, w = 0;  // Velocity and angular velocity
	// Mean and Variance of the floor sensors calculated by the INIT state
	 std::vector<uint32_t> proxValuesMean(numSensors,0);
	 std::vector<uint32_t> proxValuesVar(numSensors,0);
	 std::vector< std::vector<uint16_t> > proxValues(numSensors);  // Prox. values for each sensor

//	   INFO_MSG ("Reading the proximity ring sensors for 20 times:" )
//	   for (;;){
//	     std::vector<uint16_t> prox(8,0);
//	     if (myCAN.getProximityRingValue(prox) == 0) {
//	       for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++)
//	         INFO_MSG( sensorIdx << ": " << prox[sensorIdx] )
//	     } else {
//	       WARNING_MSG( "Fail" )
//	     }
//	     INFO_MSG( "-------" )
//	   }

	  int brightness = 200;
	  INFO_MSG("Set brightness to  "<< brightness );
	  myCAN.setLightBrightness(brightness);


  types::position robotPositionNull;
  robotPositionNull.x = 0;
  robotPositionNull.y = 0;
  robotPositionNull.f_z = 0;
  myCAN.setOdometry(robotPositionNull);

  while (true) {
  myCAN.getProximityRingValue(proxRing);

  switch(state) {
  case INIT: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
		myCAN.setTargetSpeed(0,0); // STOP
		  // Init phase
		   for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
			 myCAN.setLightColor(ledIdx, amiro::Color::MAGENTA);
		   }

		   // Clear values
		   for (int sensorIdx = 0; sensorIdx < numSensors; sensorIdx++) {
			   proxValues[sensorIdx].clear();
		   }

		  // Build statistics from the sensor values
		  INFO_MSG( "Record values" )
		  for ( int idx = 0; idx < 40; ++idx ){
		  // for ( ;;){
		   std::vector<uint16_t> prox(numSensors,0);
		   if (myCAN.getProximityFloorValue(prox) == 0) {
			 for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++) {
			   proxValues[sensorIdx].push_back(prox[sensorIdx]);
			   INFO_MSG( sensorIdx  << ": "<< prox[sensorIdx])
			 }
			} else {
			 WARNING_MSG( "Fail" )
			}
		   }


		 // Calc Mean
		 for (int sensorIdx = 0; sensorIdx < proxValues.size(); sensorIdx++) {
			 for (int valueIdx = 0; valueIdx < proxValues[sensorIdx].size(); valueIdx++) {
			  proxValuesMean[sensorIdx] += proxValues[sensorIdx][valueIdx];
			}
			proxValuesMean[sensorIdx] /= proxValues[sensorIdx].size();
			INFO_MSG( "Mean " << proxValuesMean[sensorIdx]  )
		 }

		 // Calc Variance
		 for (int sensorIdx = 0; sensorIdx < proxValues.size(); sensorIdx++) {
			 for (int valueIdx = 0; valueIdx < proxValues[sensorIdx].size(); valueIdx++) {
				 //INFO_MSG( "Val " << valueIdx << " " << proxValues[sensorIdx][valueIdx] )
				 //INFO_MSG( "Var " << valueIdx << " " << pow(int32_t(proxValues[sensorIdx][valueIdx]) - int32_t(proxValuesMean[sensorIdx]) ,2) )
			  proxValuesVar[sensorIdx] += pow(int32_t(proxValues[sensorIdx][valueIdx]) - int32_t(proxValuesMean[sensorIdx]),2);
			}
			//INFO_MSG( "Var " << proxValuesVar[sensorIdx] )
			proxValuesVar[sensorIdx] /= proxValues[sensorIdx].size();
			INFO_MSG( "Var " << proxValuesVar[sensorIdx] )
		 }

		 // INIT finish
		 INFO_MSG( "Drive forward with 2 cm/s and 0 rad/s" )
		 v = vNorm; w = 0;
		 myCAN.setTargetSpeed(v,w);
		 // INIT finish: Show some green light
		 for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
			 myCAN.setLightColor(ledIdx, amiro::Color::GREEN);
			}
		 state = COSTAL_NAV;
    break;
	}
  case NAV: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
  }
  case COSTAL_NAV: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
	  // Read the sensor values
		   std::vector<uint16_t> prox(4,0);
	  // types::position robotPosition = myCAN.getOdometry();
	  // robotPosition.x
		   if (myCAN.getProximityFloorValue(prox) == 0) {
			 for (int sensorIdx = 0; sensorIdx < prox.size(); sensorIdx++) {
			   INFO_MSG( prox[sensorIdx] )
			 }
			} else {
			 WARNING_MSG( "Fail" )
			}
	   // Change state, if the frontal sensor detected a coast
		   const int coastalDiff = -6000;//-2000;  // Grater for higher sensitivity
		   WARNING_MSG( (int32_t(prox[0]) - int32_t(proxValuesMean[0])) )
		   WARNING_MSG( (int32_t(prox[3]) - int32_t(proxValuesMean[3])) )
		   if ((int32_t(prox[0]) - int32_t(proxValuesMean[0]) < coastalDiff) ||
				   (int32_t(prox[1]) - int32_t(proxValuesMean[1]) < coastalDiff) ||
				   (int32_t(prox[2]) - int32_t(proxValuesMean[2]) < coastalDiff) ||
			   (int32_t(prox[3]) - int32_t(proxValuesMean[3]) < coastalDiff)) {
			 v = 0; w = 0;
			 myCAN.setTargetSpeed(v,w);
			 INFO_MSG( "STOP" )
		     state = CORNER_TURN_INIT;
		   }

			//w = ((int32_t(prox[1]) - int32_t(proxValuesMean[1])) * 1e2) + 100000;  // Was former 200000
			//INFO_MSG( "w: " << -w )
			//INFO_MSG( "w_mean: " << proxValuesMean[1] )
			myCAN.setTargetSpeed(vNorm,0);
			INFO_MSG( "-------" )

			 INFO_MSG ("Reading the proximity ring sensors and break if they see something" )
			   // std::vector<uint16_t> proxRing(8,0);
			   // if (myCAN.getProximityRingValue(proxRing) == 0) {
			   ringSensors();


			   break;
		 }
  case CORNER_TURN_INIT: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
	  // First just stop
		 v = 0; w = 0;
		 myCAN.setTargetSpeed(v,w);
		 INFO_MSG( "STOP" )
	  // Drive back one centimeter
		 myCAN.setTargetSpeed(-20000,0);
		 sleep(1);
		 // Reset the odometry
		 myCAN.setOdometry(robotPositionNull);
		 myCAN.setOdometry(robotPositionNull);
		 myCAN.setOdometry(robotPositionNull);
		 myCAN.setTargetSpeed(0,0);
	  // Turn for 90° to the left
		 state = CORNER_TURN;

	  // Init phase
	   for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
		 myCAN.setLightColor(ledIdx, amiro::Color::RED);
	   }
	   break;

  }
  case CORNER_TURN: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
		ringSensors();
	  // Get the current odometry
	  	 types::position robotPosition = myCAN.getOdometry();
	  // Set the velocities
	     const int32_t M_PI_HALF = int32_t((M_PI * 1e6) / 2.0f);
	     int32_t rotation = 2 * M_PI_HALF - robotPosition.f_z; // µrad/s
	     int32_t rotationMin = 200000; // µrad/s
		 v = 0;
		 w = rotation < rotationMin ? rotationMin : rotation;
		 w = rotation < rotationMin ? rotationMin : rotation;
		 w = 400000;
		 myCAN.setTargetSpeed(v,w);
		 INFO_MSG( "f_turn: " << rotation )
		 INFO_MSG( "max_rot: " << 		 4 * M_PI_HALF  * 10 / 360 )
	  // Stop turning if the robot is under 5° accuracy
		 if (rotation < 4 * M_PI_HALF / 360 * 5) {
			 INFO_MSG( "STOP" )
		     myCAN.setTargetSpeed(0,0);
			 // log the ring sensors
			 state = COSTAL_NAV;
			 for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
				 myCAN.setLightColor(ledIdx, amiro::Color::GREEN);
			 }
		 }
		 break;

  }
  case BEER: {
			DEBUG_MSG( "STATE: " << StateTypes[state] )
			myCAN.setTargetSpeed(0, 0); // STOP
			   if (true) {
					if (proxRing[0] > 0x1000u || proxRing[1] > 0x1000u) {
						state = INIT;
						//timeout
					}
			   } else {
				 WARNING_MSG( "Fail" )
			   }
			break;
		}
  case COCKTAIL: {
	  DEBUG_MSG( "STATE: " << StateTypes[state] )
			myCAN.setTargetSpeed(0, 0); // STOP
			INFO_MSG ("Reading the proximity ring sensors and break if they see something" )
			if (true) {
				if (proxRing[6] > 0x1000u || proxRing[7] > 0x1000u) {
					state = INIT;
					//timeout
				}
			} else {
				WARNING_MSG( "Fail" )
			}
			break;
  }
  default: {
		 v = 0; w = 0;
		 myCAN.setTargetSpeed(v,w);
		 INFO_MSG( "STOP" )
		 for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
		 myCAN.setLightColor(ledIdx, amiro::Color::YELLOW);
		}
		 return 1;
  }


  	  } // Switch case

  }  // while true
return 0;
}