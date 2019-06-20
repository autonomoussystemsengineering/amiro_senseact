
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/iplImageConverter/IplImageConverter.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>


using namespace boost;
using namespace std;
using namespace rsb;
using namespace rst::converters::opencv;
using namespace rsb::converter;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string g_sInScope = "/image";

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving compressed images");

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
    
    INFO_MSG( "Scope: " << g_sInScope)


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  shared_ptr<IplImageConverter> converter(new IplImageConverter());
  converterRepository<std::string>()->registerConverter(converter);


  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create and start the listener
  rsb::ListenerPtr listener = factory.createListener(g_sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<IplImage>(imageQueue)));

  // Pop the images and show them
  while (true) {
        cv::Mat image = cv::Mat(imageQueue->pop().get(), true);
        // Exit the program if any key was pressed
        if ( cv::waitKey(1) >= 0 )
          break;

        cv::imshow("input", image);
}

  return 0;

}
