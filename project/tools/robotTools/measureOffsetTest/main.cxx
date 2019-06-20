//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This is the proximity sensor logger. It can be used for
//               offset calculation for the rirReader.
//============================================================================

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

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

#define START_WAITING_TIME_MS 10000 // 10s
#define COUNT_MEASUREMENTS 500

using namespace std;

  
string configFileName = "irConfig.conf";
bool saveMean = false;
bool ignoreFloor = false;
bool ignoreRing = false;

int main(int argc, char **argv) {
  
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("calculate,c", "Calculate the mean of all measured values and writes the results in config file (default is 'irConfig.conf', change with -f [--configfile]).")
    ("configfile,f", po::value < std::string > (&configFileName), "Config file name (default is 'irConfig.conf'.")
    ("ignoreFloor", "Ignores the values of the floor proximity sensors.")
    ("ignoreRing", "Ignores the values of the ring proximity sensors.");

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

  saveMean = vm.count("calculate") || vm.count("configfile");
  ignoreFloor = vm.count("ignoreFloor");
  ignoreRing = vm.count("ignoreRing");

  if (ignoreFloor && ignoreRing) {
    WARNING_MSG("Floor and ring proximity sensors shall be ignored ... so there is nothing to do ;)");
    return 0;
  }

  // Init the CAN interface
  ControllerAreaNetwork CAN;    

  // Datastructure for the CAN messages
  std::vector< uint16_t > proximityRingValue(8,0);
  std::vector< uint16_t > proximityFloorValue(4,0);

  for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
    CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::WHITE));
  }

  for (int sensorIdx = 0; sensorIdx < 8 ; ++sensorIdx) {
    proximityRingValue[sensorIdx] = 0;
  }

  for (int sensorIdx = 0; sensorIdx < 4 ; ++sensorIdx) {
    proximityFloorValue[sensorIdx] = 0;
  }
      
  INFO_MSG("Waiting for " << START_WAITING_TIME_MS << " ms before measuring");
  uint8_t timestep;
  int ledIdx = 0;
  for(timestep=0; timestep<START_WAITING_TIME_MS/100; timestep++) {
    int ledCount = (int)(((float)timestep+1)*8.0/(((float)START_WAITING_TIME_MS)/100.0));
    if (ledCount > ledIdx) {
      CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BROWN));
      ledIdx = ledCount;
    }
    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
  }

  FILE *file;
  file = fopen("measurementsRingAndFloor.txt", "w");
  for (timestep=0; timestep<8; timestep++) {
    fprintf(file, "PRV %i\t", timestep);
  }
  for (timestep=0; timestep<4; timestep++) {
    fprintf(file, "PFV %i\t", timestep);
  }
  fprintf(file, "\n");

  int failR = 0;
  int failF = 0;
  int step = 0;
  uint8_t sensorIdx = 0;
  int measurs;

  std::vector<uint32_t> sumRingProximity(8,0);
  for (int v=0; v<8; v++) {
    sumRingProximity[v] = 0;
  }

  ledIdx = 0;
  INFO_MSG("Start measuring (" << COUNT_MEASUREMENTS << " times)");
  for(measurs=0; measurs<COUNT_MEASUREMENTS; measurs++) {
    // Read the proximity data
    if (!ignoreRing) {
      failR = CAN.getProximityRingValue(proximityRingValue);
    }
    if (!ignoreFloor) {
      failF = CAN.getProximityFloorValue(proximityFloorValue);
    }
    if (failR == 0) {
      for (sensorIdx = 0; sensorIdx < proximityRingValue.size(); sensorIdx++) {
        //INFO_MSG("PRV " << (int) sensorIdx << ": " << proximityRingValue[sensorIdx])
        fprintf(file, "%i\t", proximityRingValue[sensorIdx]);
        if (saveMean) {
          sumRingProximity[sensorIdx] += proximityRingValue[sensorIdx];
        }
      }
    } else {
      WARNING_MSG( "Fail in PRV" )
    }
    if (failF == 0) {
      for (sensorIdx = 0; sensorIdx < proximityFloorValue.size(); sensorIdx++) {
        //INFO_MSG("PFV " << (int) sensorIdx << ": " << proximityFloorValue[sensorIdx])
        fprintf(file, "%i\t", proximityFloorValue[sensorIdx]);
      }
    } else {
      WARNING_MSG( "Fail in PFV" )
    }
    if (step < (int)(((float)measurs+1)*8.0/(float)COUNT_MEASUREMENTS)) {
      step++;
      INFO_MSG(" - " << (float)step/8.0*100.0 << "% of measurements done");
    }
    int ledCount = (int)(((float)measurs+1)*8.0/(float)COUNT_MEASUREMENTS);
    if (ledCount > ledIdx) {
      CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BLUE));
      ledIdx = ledCount;
    }
    fprintf(file, "\n");
      
    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(150) );
  }

  fclose(file);
  INFO_MSG("Measuring finished");

  if (saveMean) {
    for (int v=0; v<8; v++) {
      sumRingProximity[v] /= 500;
    }
    FILE *fileMean;
    fileMean = fopen(configFileName.c_str(), "w");
    for (int v=0; v<8; v++) {
      fprintf(fileMean, "%i\t", sumRingProximity[v]);
    }
    fclose(fileMean);
  }

  return EXIT_SUCCESS;
}
