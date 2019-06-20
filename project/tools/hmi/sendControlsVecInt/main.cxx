
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
 
// For checking character pressed in the console
#include <kbhit.hpp>


int main(int argc, const char **argv){
  INFO_MSG("Start")
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
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Prepare RSB informer
  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (outScope);

  // Calculate the new steering (two elements with 0 initialized)
  boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));
  
  // Translation in µm/s: vecSteering->at(0)
  // Rotation in µrad/s: vecSteering->at(1)
  // Sending the steering vector: informer_vec->publish(vecSteering);

   int KB_code=0;

   // Process as long as ESC is not pressed
   while(KB_code != KB_ESCAPE )
   {
     // Enter this condition if a button was pressed
     if (kbhit())
      {
            // Get the character
            KB_code = getchar();
            INFO_MSG( "KB_code = " << KB_code )

            switch (KB_code)
            {
                case KB_A:
                  // µDegree per second
                  vecSteering->at(1) = vecSteering->at(1) + 100000;
                  informer_vec->publish(vecSteering);
                break;

                case KB_D:
                  // µDegree per second
                  vecSteering->at(1) = vecSteering->at(1) - 100000;
                  informer_vec->publish(vecSteering);
                break;

                case KB_W:
                  // Micrometer per second
                  vecSteering->at(0) = vecSteering->at(0) + 10000;
                  informer_vec->publish(vecSteering);
                break;

                case KB_S:
                  // Micrometer per second
                  vecSteering->at(0) = vecSteering->at(0) - 10000;
                  informer_vec->publish(vecSteering);
                break;
                case KB_SPACE:
                  vecSteering->at(0) = 0;
                  vecSteering->at(1) = 0;
                  informer_vec->publish(vecSteering);
                break;
                case KB_ENTER:
                  informer_vec->publish(vecSteering);
                break;
            }        
            INFO_MSG( "Vx  = " << vecSteering->at(0) << " µm/s , Wz = " << vecSteering->at(1) << " µrad/s" )
      }
      // Sleep for a millisecond
      usleep(1000);
  }
  return 0;
}
