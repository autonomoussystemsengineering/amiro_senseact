//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send integer vector with arbitrary values
//============================================================================




#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>


#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecConverter/main.hpp>

using namespace std;
using namespace muroxConverter;
using namespace boost;


std::string g_sOutScope_IR = "/IR";
int g_vecAmount = 12;
int g_vecMax = 10;

int main (int argc, const char **argv){
  
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&g_sOutScope_IR), "Scope for sending the IR data")
    ("vecAmount,a", po::value < int > (&g_vecAmount), "Amount of vector entries")
    ("vecMax,v", po::value < int > (&g_vecMax), "Maximal value of randomnes (0 .. vecMax)");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(
    po::command_line_parser(argc, argv).options(options).positional(p).run(),
    vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }
  
  // afterwards, let program options handle argument errors
  po::notify(vm);


  INFO_MSG( "Outscope: " << g_sOutScope_IR)
  INFO_MSG( "Vector entries: " << g_vecAmount)
  INFO_MSG( "Maximal value: " << g_vecMax)

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecConverter<int> > converter(new vecConverter<int>());


  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Factory& factory = rsb::getFactory();
  rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (g_sOutScope_IR);

  // Share the pointer to the IR data
  boost::shared_ptr< std::vector<int> > vecData(new std::vector<int> (g_vecAmount,0));
  
  for(;;) {
    // Get random values
    for (unsigned int idx = 0; idx < vecData->size(); idx++)
      vecData->at(idx) = rand() % g_vecMax;
    // Send IR data
    informer_vec->publish(vecData);
    DEBUG_MSG( "Send vector<int> with "<< g_vecAmount << " values to the scope " << g_sOutScope_IR )
    // Sleep for 125 ms
    boost::this_thread::sleep( boost::posix_time::milliseconds(125) );
  }

  return EXIT_SUCCESS;
}
