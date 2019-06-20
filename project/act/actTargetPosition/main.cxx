// ============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send target positions relative to the current
//               AMiRos local position with respect to its
//               coordinate system
// ============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>


// RST
#include <rsb/converter/ProtocolBufferConverter.h>
// #include <types/TargetPoseEuler.pb.h>
// #include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Pose.pb.h>

#include <Eigen/Geometry>

#include <ControllerAreaNetwork.h>
#include <utils.h>


using namespace std;

void sendTargetPosition(rsb::EventPtr event, ControllerAreaNetwork &CAN) {
  // Get the message
  // boost::shared_ptr<rst::geometry::TargetPoseEuler> message = boost::static_pointer_cast<rst::geometry::TargetPoseEuler>(event->getData());
  boost::shared_ptr<rst::geometry::Pose > message = boost::static_pointer_cast<rst::geometry::Pose >(event->getData());

  const ::rst::geometry::Rotation& rotation = message->rotation();
  const ::rst::geometry::Translation& translation = message->translation();


  Eigen::Quaternion< double > q(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
  Eigen::Matrix<double,3,1> rpy;
  utils::conversion::quaternion2euler(&q, &rpy);

  types::position robotPosition;
  robotPosition.x   = static_cast<int>(translation.x() * 1e6);
  robotPosition.y   = static_cast<int>(translation.y() * 1e6);
  robotPosition.z   = static_cast<int>(translation.z() * 1e6);
  robotPosition.f_x = static_cast<int>(rpy(0) * 1e6);
  robotPosition.f_y = static_cast<int>(rpy(1) * 1e6);
  if(rotation.qz() == -1.0) { // hack because qz == -1 and qz == 1 do the same action but 1 a left rotation and -1 a left rotation.
    robotPosition.f_z = static_cast<int>(-rpy(2) * 1e6);
  } else {
    robotPosition.f_z = static_cast<int>(rpy(2) * 1e6);
  }

  INFO_MSG("x: " << robotPosition.x << "um");
  INFO_MSG("y: " << robotPosition.y << "um");
  INFO_MSG("f_z: " << robotPosition.f_z << "urad");
  // INFO_MSG("time: " << static_cast<uint16_t>(message->target_time()) << "ms");
  // Set the target position
  CAN.setTargetPosition(robotPosition, static_cast<uint16_t>(1 /*ms*/));
}

static std::string rsbInScope = "/targetPositions";

int main(int argc, const char ** argv) {
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.")
    ("inscope,i", po::value<std::string>(&rsbInScope), "Scope for receiving target positions");

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

  // Create the CAN interface
  ControllerAreaNetwork CAN;

  // Get the RSB factory
  #if RSB_VERSION_NUMERIC < 1200
  rsb::Factory& factory = rsb::Factory::getInstance();
  #else
  rsb::Factory& factory = rsb::getFactory();
  #endif

  //   // Register new converter
  // boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler> >
  //     converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler>());
  // rsb::converter::converterRepository<std::string>()->registerConverter(converter);#
  // Register new converter
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader
  rsb::ReaderPtr reader = factory.createReader(rsbInScope);

  for (;;) {
    // Wait for the message
    sendTargetPosition(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
} // main
