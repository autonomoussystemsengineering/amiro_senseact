//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Tool for printing the measured edge distance.
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <functional>
#include <algorithm>
#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time.hpp>

using namespace std;

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

// edge model 6cm
#define EDGEMODEL_M 6.359946153158588
#define EDGEMODEL_B 0.401918238192352

/* Offsets for AMiRo 36 */
static int GROUND_OFFSETS[] = {2437, 2463, 2483, 2496, 2457, 2443, 2508, 2352};
static int AIR_OFFSETS[] = {2213, 2316, 2341, 2329, 2331, 2290, 2335, 2152};




void splitString(const std::string &str, vector<std::string> &parts, const std::string delimiters) {
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos) {
        // Found a token, add it to the vector.
        parts.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

float edgeDist(int senValue, int senIdx) {
  return EDGEMODEL_M * ((float)senValue) / ((float)(GROUND_OFFSETS[senIdx] - AIR_OFFSETS[senIdx])) + EDGEMODEL_B;
}

void configRIR(bool loadOffsets) {
  if (loadOffsets) {
    char input[100];
    FILE *irConfig = fopen("irConfig.conf", "r");
    if (irConfig) {
      fgets(input, 100, irConfig);
      INFO_MSG("Read 'irConfig.conf':");
      vector<std::string> parts;
      splitString(std::string(input), parts, "\t");
      for (int part=0; part<parts.size(); part++) {
        GROUND_OFFSETS[part] = atoi(std::string(parts[part]).c_str());
        INFO_MSG(" " << (part+1) << ") " << GROUND_OFFSETS[part]);
      }
    } else {
      WARNING_MSG("Coudn't load 'irConfig.conf'! Now using standard offsets.");
    }
    irConfig = fopen("irEmpty.conf", "r");
    if (irConfig) {
      fgets(input, 100, irConfig);
      INFO_MSG("Read 'irEmpty.conf':");
      vector<std::string> parts;
      splitString(std::string(input), parts, "\t");
      for (int part=0; part<parts.size(); part++) {
        AIR_OFFSETS[part] = atoi(std::string(parts[part]).c_str());
        INFO_MSG(" " << (part+1) << ") " << AIR_OFFSETS[part]);
      }
    } else {
      WARNING_MSG("Coudn't load 'irEmpty.conf'! Now using standard offsets.");
    }
  }
}

int doRIR(std::vector<uint16_t> &obstacleValues, std::vector<uint16_t> &groundValues, ControllerAreaNetwork &CAN) {
  std::vector<uint16_t> proximityRingValue(8,0);
  int fail = CAN.getProximityRingValue(proximityRingValue);

  if (fail == 0) {

    // claculate offsets in proximity values
    for (int sensorIdx = 0; sensorIdx < 8; sensorIdx++) {

      // calculate obstacle values
      if (proximityRingValue[sensorIdx] <= GROUND_OFFSETS[sensorIdx]) {
        obstacleValues[sensorIdx] = 0;
      } else {
        obstacleValues[sensorIdx] = proximityRingValue[sensorIdx] - GROUND_OFFSETS[sensorIdx];
      }

      // calculate ground values
      if (proximityRingValue[sensorIdx] >= GROUND_OFFSETS[sensorIdx]) {
        groundValues[sensorIdx] = GROUND_OFFSETS[sensorIdx] - AIR_OFFSETS[sensorIdx];
      } else if (proximityRingValue[sensorIdx] < AIR_OFFSETS[sensorIdx]) {
        groundValues[sensorIdx] = 0;
      } else {
        groundValues[sensorIdx] = proximityRingValue[sensorIdx] - AIR_OFFSETS[sensorIdx];
      }

      float dist = edgeDist(groundValues[sensorIdx], sensorIdx);
      int ledIdx = sensorIdx+4;
      if (ledIdx >= 8) ledIdx -= 8;
      if (dist <= 1.0) {
        CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BLUE));
      } else if (dist <= 5.0) {
        CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::RED));
      } else {
        CAN.setLightColor(ledIdx, amiro::Color(amiro::Color::GREEN));
      }

    }

    return 0;
  } else {
    return 1;
  }
}

int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
      ("loadOffsets,l", "Loads offsets from the file 'irConfig.conf' and 'irEmpty.conf'.");

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

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  // Configure RIR-Reader
  configRIR(vm.count("loadOffsets"));

  for(int led=0; led<8; led++) {
    CAN.setLightColor(led, amiro::Color(amiro::Color::BLACK));
  }

  
  int fail = 1;
  uint8_t sensorIdx = 0;
  bool ok = true;

  std::vector<uint16_t> obstacleValues(8,0);
  std::vector<uint16_t> groundValues(8,0);

  while(ok) {
    fail = doRIR(obstacleValues, groundValues, CAN);    
    if (fail == 0) {
      for (int idx=0; idx<8; idx++) {
        float dist = edgeDist(groundValues[idx], idx);
        INFO_MSG((idx+1) << ") " << dist << " cm (" << (((float)groundValues[idx])/((float)(GROUND_OFFSETS[idx] - AIR_OFFSETS[idx]))) << " = " << groundValues[idx] << "/[" << GROUND_OFFSETS[idx] << " - " << AIR_OFFSETS[idx] << "])");
      }
    } else {
      WARNING_MSG("Fail");
      ok = false;
    }
      
    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(250) );
  }

  return EXIT_SUCCESS;
}
