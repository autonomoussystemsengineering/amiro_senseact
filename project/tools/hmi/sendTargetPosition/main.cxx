
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send target positions relative to the current
//               AMiRos local position with respect to its
//               coordinate system
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
#include <types/TargetPoseEuler.pb.h>
#include <rst/geometry/Translation.pb.h>

using namespace std;


int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string outScope = "/targetPositions";
  double x = 0, f = 0;
  unsigned short t = 1;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&outScope), "Scope for sending the motor steering commands")
    ("x", po::value < double > (&x), "Distance to drive in meter")
    ("f", po::value < double > (&f), "Turn to spin in degree")
    ("t", po::value < unsigned short > (&t), "Time for travel in milliseconds");

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

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();
  
  // Register new converter
boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler>());
rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Informer< rst::geometry::TargetPoseEuler >::Ptr informer = factory.createInformer< rst::geometry::TargetPoseEuler > (outScope);

  // Define the steering message for sending over RSB
  boost::shared_ptr< rst::geometry::TargetPoseEuler > steering(new rst::geometry::TargetPoseEuler);
  // Get the mutable objects
  rst::geometry::RotationEuler *steeringRotation = steering->mutable_target_pose()->mutable_rotation();
  rst::geometry::Translation *steeringTranslation = steering->mutable_target_pose()->mutable_translation();
  steeringRotation->set_roll(0);
  steeringRotation->set_pitch(0);
  steeringRotation->set_yaw(f * M_PI / 180.0);
  steeringTranslation->set_x(x);
  steeringTranslation->set_y(0);
  steeringTranslation->set_z(0);
  steering->set_target_time((unsigned short)t);

  INFO_MSG( " x = " << x << " m , f = " << f << " deg , t = " << t << " ms" );

  informer->publish(steering);

  return 0;

}
