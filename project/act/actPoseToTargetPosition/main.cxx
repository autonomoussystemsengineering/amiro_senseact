
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Drives to target coordinate system
//============================================================================

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
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>

#include <ControllerAreaNetwork.h>

#include <Eigen/Geometry>

#include <utils.h>

using namespace std;

void sendTargetPosition(rsb::EventPtr event, ControllerAreaNetwork &CAN) {

  // Get the message
  boost::shared_ptr<rst::geometry::Pose > message = boost::static_pointer_cast<rst::geometry::Pose >(event->getData());

  const ::rst::geometry::Rotation& rotation = message->rotation();
  const ::rst::geometry::Translation& translation = message->translation();

  types::position robotPosition;
  robotPosition.x = static_cast<int>(0);
  robotPosition.y = static_cast<int>(0);
  robotPosition.z = static_cast<int>(0);
  robotPosition.f_x = static_cast<int>(0);
  robotPosition.f_y = static_cast<int>(0);
  robotPosition.f_z = static_cast<int>(0);

  // To drive into a target pose [T,R], AMiRo has to process three steps:
  // 1. Turn (angle of T in the origin) to the target pose, to face directly the coordinate system
  const double angT_rad = atan2(translation.y(), translation.x());
  // 2. Drive the straight path (|T|), to get into the center of the target system
  const double absT_meter = sqrt(pow(translation.y(),2) + pow(translation.x(),2));
  // 3. Turn to the target pose (R - angle of T in the origin)
  Eigen::Quaternion< double > q(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
  Eigen::Matrix<double,3,1> rpy;
  utils::conversion::quaternion2euler(&q, &rpy);
  const double yaw_rad = rpy(2);
  const double targetRotation_rad = yaw_rad - angT_rad;

  // 1.
  INFO_MSG("angT_rad: " << angT_rad * 180 / M_PI << " deg");
  robotPosition.f_z = static_cast<int>(angT_rad * 1e6);
  CAN.setTargetPosition(robotPosition, static_cast<uint16_t>(1 /*ms*/));
  robotPosition.f_z = static_cast<int>(0);
  usleep(1 /*seconds*/ * 1000000);
  // 2.
  INFO_MSG("x: " << absT_meter << "m");
  robotPosition.x = static_cast<int>(absT_meter * 1e6);
  CAN.setTargetPosition(robotPosition, static_cast<uint16_t>(1 /*ms*/));
  robotPosition.x = static_cast<int>(0);
  usleep(4 /*seconds*/ * 1000000);
  // 3.
  INFO_MSG("targetRotation_rad: " << targetRotation_rad * 180 / M_PI << " deg");
  robotPosition.f_z = static_cast<int>(targetRotation_rad * 1e6);
  CAN.setTargetPosition(robotPosition, static_cast<uint16_t>(1 /*ms*/));
  robotPosition.f_z = static_cast<int>(0);
  usleep(1 /*seconds*/ * 1000000);
}

static std::string rsbInScope = "/targetPose";
static std::string sSuccessScope = "/success";
static std::string sSuccessContent = "rec";

int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
        ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving target positions")
        ("outscope,o", po::value < std::string > (&sSuccessScope), "Scope for success reply")
        ("outscopeContent,c", po::value < std::string > (&sSuccessContent), "Content of success reply");

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
#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif

    // Register new converter
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
      converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader
  rsb::ReaderPtr reader = factory.createReader(rsbInScope);
  // Prepare RSB informer
  boost::shared_ptr<std::string> successContent(new std::string(sSuccessContent));
  rsb::Informer< std::string >::Ptr informerFollowing = factory.createInformer< std::string > (sSuccessScope);

  for(;;) {
    // Wait for the message
    sendTargetPosition(reader->read(), CAN);
    // Send the success
    informerFollowing->publish(successContent);
  }

  return EXIT_SUCCESS;
}
