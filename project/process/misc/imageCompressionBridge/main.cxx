
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
#include <rsb/util/QueuePushHandler.h>


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

static std::string sInScope = "/image/input";
static std::string sOutScope = "/image/compressed";
static int inputFrequency = 5; // Hz
static int compressParam = 100;
static float resizeRatio = 1;

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscope,i", po::value < std::string > (&sInScope),"Scope for receiving original image.")
            ("outscope,o", po::value < std::string > (&sOutScope),"Scope for sending compressed image.")
            ("frequence,f", po::value < int > (&inputFrequency),"Frequency of sending compressed images [Hz].")
            ("compressParameters,c", po::value < int > (&compressParam),"Compress Parameter.")
            ("resizeRatio,r", po::value < float > (&resizeRatio),"Reduced pixel resolution.");

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
    
    INFO_MSG("Image scope:      " << sInScope);
    INFO_MSG("Compressed scope: " << sOutScope);
    INFO_MSG("Frequency:        " << inputFrequency);
    INFO_MSG("Compress Param.:  " << compressParam);
    INFO_MSG("Resize ration:    " << resizeRatio);


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  shared_ptr<IplImageConverter> converterIpl(new IplImageConverter());
  converterRepository<std::string>()->registerConverter(converterIpl);


  rsb::Factory &factory = rsb::getFactory();

  // Create and start the listener
  rsb::ListenerPtr listener = factory.createListener(sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<IplImage>(imageQueue)));

  // Create the informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(sOutScope));
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // calculate time for sleeping after each compression
  int sleepTime_us = 1000000/inputFrequency;

  // parameters for compression
  cv::Mat image;
  vector<uchar> buf;
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(compressParam);

  // Pop the images and compress them
  while (true) {
	// load image
        cv::cvarrToMat(&(*imageQueue->pop())).copyTo(image);
	// compress image
        imencode(".jpeg", image, buf, compression_params);

	// send compressed image
	boost::shared_ptr<std::string> frame(new std::string(buf.begin(), buf.end()));
	informer->publish(frame);

	// sleep for time specified by given frequence
	// TODO better: sleep_until
	usleep(sleepTime_us);
  }

  return 0;

}
