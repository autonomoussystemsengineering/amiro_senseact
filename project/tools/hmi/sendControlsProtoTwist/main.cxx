
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "MSG.h"

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
#include <rst/kinematics/Twist.pb.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
 
// For checking character pressed in the console
#include "kbhit.hpp"


int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string outScope = "/motor";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&outScope), "Scope for sending the motor steering commands");

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
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist> > converter(new rsb::converter::ProtocolBufferConverter<rst::kinematics::Twist>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Informer< rst::kinematics::Twist >::Ptr informer_vec = factory.createInformer< rst::kinematics::Twist > (outScope);

  // Define the steering message for sending over RSB
  boost::shared_ptr< rst::kinematics::Twist > vecSteering(new rst::kinematics::Twist);
  // Get the mutable objects
  rst::kinematics::AngularVelocities *angVel = vecSteering->mutable_angular();
  rst::kinematics::LinearVelocities  *linVel = vecSteering->mutable_linear();
  // Set initial values to all values
  angVel->set_a(0.0f); angVel->set_b(0.0f); angVel->set_c(0.0f);
  linVel->set_x(0.0f); linVel->set_y(0.0f); linVel->set_z(0.0f);

  // Init the key code
  int KB_code=0;

  while(KB_code != KB_ESCAPE )
  { 
    if (kbhit())
    {
      KB_code = getchar();
      INFO_MSG( "KB_code = " << KB_code )

          switch (KB_code)
          {
              case KB_A:
                // Rad per second
                angVel->set_c(angVel->c() + 0.1f);
                informer_vec->publish(vecSteering);
              break;
              case KB_D:
                // Rad per second
                angVel->set_c(angVel->c() - 0.1f);
                informer_vec->publish(vecSteering);
              break;
              case KB_W:
                // Meter per second
                linVel->set_x(linVel->x() + 0.01f);
                informer_vec->publish(vecSteering);
              break;
              case KB_S:
                // Meter per second
                linVel->set_x(linVel->x() - 0.01f);
                informer_vec->publish(vecSteering);
              break;
              case KB_SPACE:
                  angVel->set_c(0.0f);
                  linVel->set_x(0.0f);
                  informer_vec->publish(vecSteering);
              break;
              case KB_ENTER:
                  informer_vec->publish(vecSteering);
              break;
          }
          INFO_MSG( " Vx = " << vecSteering->linear().x() << " m/s , Wz = " << vecSteering->angular().c() << " rad/s" )
    }
    // Sleep for a millisecond
    usleep(1000);
  }

  return 0;

}
