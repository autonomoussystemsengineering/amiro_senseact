//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Receives command for setting the lights.
//               The structure (L:Brightness, R0: Red LED 0, G0: Green LED 0, B0: Blue LED 0):
//               R0G0B0R1G1B1R2G2B2R3G3B3R4G4B4R5G5B5R6G6B6R7G7B7L
// Edited by   : jhomburg <jhomburg@techfak.uni-bielefeld.de>
//============================================================================

// #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

//#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/generic/Value.pb.h>

#include <ControllerAreaNetwork.h>

using namespace std;

// static int g_red = 0;
// static int g_green = 0;
// static int g_blue = 0;

void setLightRing(rsb::EventPtr event, ControllerAreaNetwork &CAN) {

  // Convert message to std::vector<int>
  std::vector<int> lv(0,0);
  if (event->getType() != "rst::generic::Value"){
    return;
  }

  // Get the message
  boost::shared_ptr<rst::generic::Value > message = boost::static_pointer_cast<rst::generic::Value >(event->getData());
  if (message->type() != rst::generic::Value::ARRAY){
    return;
  }
  int size = message->array_size();
  rst::generic::Value entry;
  for (int i=0; i<size; i++){
    entry=message->array(i);
    if (entry.type()!=rst::generic::Value::INT){
      return;
    }
    lv.push_back(entry.int_());
  }
  if (lv.size()<25){
    return;
  }

  // Set the colors
   for (int ledIdx = 0; ledIdx < 8 ; ++ledIdx) {
     INFO_MSG( "Set LED " << ledIdx << " to r: " << lv.at(ledIdx * 3) << " g: " << lv.at(ledIdx * 3 + 1) << " b: " << lv.at(ledIdx * 3 + 2) )
     CAN.setLightColor(ledIdx, amiro::Color(lv.at(ledIdx * 3),lv.at(ledIdx * 3 + 1),lv.at(ledIdx * 3 + 2)));
   }

  // Set the brightness
  INFO_MSG( "Set LED brightness: " << lv.at(24) )
  CAN.setLightBrightness(lv.at(24));
}


int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;

  std::string rsbInScope = "/lights";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,o", po::value < std::string > (&rsbInScope), "Scope for receiving the light commands");

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

  // Create the CAN interface
  ControllerAreaNetwork CAN;

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for std::vector<int>
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader
  rsb::ReaderPtr reader = factory.createReader(rsbInScope);

  for(;;) {
    // Wait for the message
    setLightRing(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
}
