
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send motion vectors to the motor of AMiRo
//============================================================================

#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/misc/SignalWaiter.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/math/Vec2DFloat.pb.h>

#include <ControllerAreaNetwork.h>

using namespace std;


static bool vectorTwbIsProx = false;

void sendMotorCmd(rsb::EventPtr event, ControllerAreaNetwork &CAN) {
  //Check if event is from type value
  if (event->getType() != "rst::math::Vec2DFloat"){
    return;
  }
  // Get the message
  boost::shared_ptr<rst::math::Vec2DFloat> message = boost::static_pointer_cast<rst::math::Vec2DFloat>(event->getData());

  // Set the motor speed
  // Velocity send in µm/s
  // Angular velocity send in µrad/s
  if (vectorTwbIsProx) {
    CAN.sendTwbVector(message->x(), message->y());
  } else {
    CAN.sendProxVector(message->x(), message->y());
  }
  DEBUG_MSG( "x: " << message->x() << " y: " << message->y())
}

static std::string rsbInScope = "/vector";
static int x = 0;
static int y = 0;
static bool useStaticVelocities = false;
static bool sendStaticVelocitiesOnce = false;
static int sendStaticVelocitiesPeriod_us = 100 /*ms*/ * 1e3;

int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands")
    ("x", po::value < int > (&x), "Static x vector")
    ("y", po::value < int > (&y), "Static y vector")
    ("useStaticVelocities", po::bool_switch(&useStaticVelocities)->default_value(false), "Make use of the given vector, instead of using spread")
    ("vectorTwbIsProx", po::bool_switch(&vectorTwbIsProx)->default_value(false), "Send the vector with CAN id as proximity (and not as TWB)")
    ("sendStaticVelocitiesOnce", po::bool_switch(&sendStaticVelocitiesOnce)->default_value(false), "Send the static velocity command once")
    ("sendStaticVelocitiesPeriod", po::value < int > (&sendStaticVelocitiesPeriod_us), "Period between static velocity sending in µs");


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

  // STATIC BEHAVIOR
  rsc::misc::initSignalWaiter();
  if ( useStaticVelocities ) {
    if (sqrt(x*x+y*y) > 1.0) {
      WARNING_MSG("Given vector length greater than 1");
    }
    while (rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED) {
      if (vectorTwbIsProx) {
        CAN.sendTwbVector(x, y);
      } else {
        CAN.sendProxVector(x, y);
      }
      if (sendStaticVelocitiesOnce) {
        break;
      }
      usleep(sendStaticVelocitiesPeriod_us);
    }
    return 0;
  }


  // RSB BEHAVIOUR
  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for std::vector<int>
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::math::Vec2DFloat> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::math::Vec2DFloat>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader
  rsb::ReaderPtr reader = factory.createReader(rsbInScope);

  while (rsc::misc::lastArrivedSignal() != rsc::misc::INTERRUPT_REQUESTED && rsc::misc::lastArrivedSignal() != rsc::misc::TERMINATE_REQUESTED) {
          // Wait for the message
          sendMotorCmd(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
}
