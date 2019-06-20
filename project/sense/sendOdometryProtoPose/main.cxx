//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity ring sensor data of AMiRo
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

#include <Eigen/Geometry>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
#include <rst/geometry/Pose.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

using namespace std;

inline Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

int main(int argc, char **argv) {
  INFO_MSG("")
  // Handle program options
  namespace po = boost::program_options;
  
  std::string rsbOutScope = "/prox";
  uint32_t rsbPeriod = 0;
  bool resetOdom = false;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending odometry values")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)")
    ("resetodom,r", po::value< bool > (&resetOdom), "Reset odometry to 0 at startup");

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

  // Register new converter for std::vector<int>
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
      converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif
  rsb::Informer< rst::geometry::Pose >::Ptr informer = factory.createInformer< rst::geometry::Pose > (rsbOutScope);

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  // Reset the odometry
  if (resetOdom) {
    INFO_MSG("Resetting odometry");
    types::position o = {0,0,0,0,0,0};
    CAN.setOdometry(o);
  }

  // Datastructure for the CAN messages
  rsb::Informer<rst::geometry::Pose>::DataPtr odomData(new rst::geometry::Pose);

  Eigen::Quaterniond quat;


  types::position robotPosition;
  for(;;) {

    // Read the odometry data
    robotPosition = CAN.getOdometry();
    // Convert it to rsb data
    odomData->mutable_translation()->set_x(static_cast<double>(robotPosition.x) * 1e-6);
    odomData->mutable_translation()->set_y(static_cast<double>(robotPosition.y) * 1e-6);
    odomData->mutable_translation()->set_z(0.0f);
    quat = euler2Quaternion(0.0, 0.0, static_cast<double>(robotPosition.f_z) * 1e-6);
    odomData->mutable_rotation()->set_qx(quat.x());
    odomData->mutable_rotation()->set_qy(quat.y());
    odomData->mutable_rotation()->set_qz(quat.z());
    odomData->mutable_rotation()->set_qw(quat.w());

    informer->publish(odomData);

    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

  return EXIT_SUCCESS;
}
