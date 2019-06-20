
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <rst/geometry/Pose.pb.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

void printGpsPose(rsb::EventPtr event) {
  
  // Get the message
  boost::shared_ptr< rst::geometry::Pose > pose = boost::static_pointer_cast< rst::geometry::Pose >(event->getData());

  rst::geometry::Translation translation = pose->translation();
  rst::geometry::Rotation rotation = pose->rotation();

  const double a = rotation.qw();
  const double b = rotation.qx();
  const double c = rotation.qy();
  const double d = rotation.qz();
  
  const double f_x = atan2(2*((a*b) + (c*d)) , (pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2)));
  const double f_y = - asin(2*((b*d) + (a*c)));
  const double f_z = atan2(2*((a*d) + (b*c)) , (pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2)));
  
  INFO_MSG( "Pose start")
  cout.setf(ios::fixed,ios::floatfield);
  cout.precision(4);
  INFO_MSG( "x:  " << translation.x() << "\ty:  " << translation.y() << "\tz:  " << translation.z())
  INFO_MSG( "qw: " << rotation.qw()   << "\tqx: " << rotation.qx()   << "\tqy: " << rotation.qy() << "\tqz:" << rotation.qz())
  INFO_MSG( "fx: " << f_x             << "\tfy: " << f_y             << "\tfz: " << f_z)
  INFO_MSG( "Pose end")
  
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string inScope = "/AMiRo/gps";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&inScope), "Scope for receiving gps data");

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
  rsb::Factory& factory = rsb::Factory::getInstance();
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> > converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::ReaderPtr reader = factory.createReader(inScope);


  while( true ){
    printGpsPose(reader->read());
  }

  return 0;

}
