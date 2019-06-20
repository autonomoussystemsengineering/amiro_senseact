

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


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;

int main(int argc, const char **argv){
      
      
    // Get the RSB factory
    rsb::Factory& factory = rsb::getFactory();
    

    // Prepare RSB informer
    rsb::Informer< std::string >::Ptr informer_vec = factory.createInformer< std::string > ("/scope");

    // Calculate the new steering (two elements with 0 initialized)
    boost::shared_ptr< std::string > value(new std::string("42"));

   while(true)
   {
        // Sleep for 125 ms
        boost::this_thread::sleep( boost::posix_time::milliseconds(125) );
        informer_vec->publish(value);
        std::cout << "send" << std::endl;
  }

  return 0;

}
