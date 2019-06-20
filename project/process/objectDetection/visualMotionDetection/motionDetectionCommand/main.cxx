//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This is the interface program for the communication and
//               visualization of the motion detection. It receives and shows
//               the camera frames and outputs a mark on the console if a
//               motion has been detected. With the keys "enter", "back space"
//               and "esc" the commands "start", "stop" and "quit" can be
//               sent.
//============================================================================

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


// protocol defines
std::string COMMAND_QUIT = "QUIT";
std::string COMMAND_START = "START";
std::string COMMAND_STOP = "STOP";

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::util;

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string g_sImageScope = "/motionDetection/image";
static std::string g_sInScope = "/motionDetection/detected";
static std::string g_sOutScope = "/motionDetection/command";

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscope,s", po::value < std::string > (&g_sInScope),"Scope for receiving motion detection signal.")
            ("imagescope,i", po::value < std::string > (&g_sImageScope),"Scope for receiving compressed images.")
            ("outscope,c", po::value < std::string > (&g_sOutScope),"Scope for sending commands.");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }
    
    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    INFO_MSG( "Output scope: " << g_sOutScope);
    INFO_MSG( "Input scope: " << g_sInScope);
    INFO_MSG( "Image scope: " << g_sImageScope);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    Factory &factory = getFactory(); //Factory::getInstance();

    // Create the command informer
    Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

    // Create and start the listener for pictures
    rsb::ListenerPtr imageListener = factory.createListener(g_sImageScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

    imageListener->addHandler(HandlerPtr(new QueuePushHandler<std::string>(imageQueue)));

    // Create and start the listener for detections
    rsb::ListenerPtr detectListener = factory.createListener(g_sInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > detectQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

    detectListener->addHandler(HandlerPtr(new QueuePushHandler<std::string>(detectQueue)));

    // Pop the images and show them
    while (true) {

        // Get the image as string
        std::string imageJpg = *imageQueue->pop().get();
        // Copy to a vector
        std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
        // Decode the image
        cv::Mat image = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
        cv::imshow(g_sInScope, image);

        // Get detection
        if (!detectQueue->empty()) {
            std::string detection = *detectQueue->pop().get();
            // check for detection
            INFO_MSG("Motion detected!");
        }

        char key = cv::waitKey(10);
        if (char(key) >= 0) {
//            printf("Key = %i (%c)\n", char(key), char(key));
            std::string command;
            switch (char(key)) {
                case 27:
                    command = COMMAND_QUIT;
                    break;
                case 8:
                    command = COMMAND_STOP;
                    break;
                case 13:
                    command = COMMAND_START;
                    break;
                default:
                    command = "";
                    break;
            }
            if (command != "") {
                shared_ptr<std::string> StringPtr(new std::string(command));
                informer->publish(StringPtr);
            }
            if (char(key) == 27) {
                break;
            }
        }
    }

    return 0;

}
