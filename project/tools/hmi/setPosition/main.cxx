
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

#include <converter/vecFloatConverter/main.hpp>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace muroxConverter;

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <kbhit.hpp> // for keyboard codes

// Program options
static std::string g_sInScope = "/image";
static std::string g_sOutScope = "/setPosition";

// Flip parameters
bool flipHorizontal, flipVertical;

// Position
static int positionX = 0;
static int positionY = 0;
static double positionTheta = 0;

bool showLine = false;
static int mousePositionX = 0;
static int mousePositionY = 0;

// Orignal received image
cv::Mat origImage;

static boost::shared_ptr< std::vector<float> > vecPosition(new std::vector<float> (3,0));
rsb::Informer< std::vector<float> >::Ptr informer;
static void onMouse(int event, int x, int y, int, void*) {

    if (event == cv::EVENT_LBUTTONDOWN) {
        positionX = x;
        positionY = y;
        showLine = true;
    } else if (event == cv::EVENT_LBUTTONUP) {

        // Undo flip
        if (flipHorizontal) {
            y = (origImage.size().height - 1) - y;
            positionY = (origImage.size().height - 1) - positionY;
        }

        if (flipVertical) {
            x = (origImage.size().width - 1) - x;
            positionX = (origImage.size().width - 1) - positionX;
        }

        positionTheta = atan2((double)y - positionY, (double)x - positionX);
        INFO_MSG("Setting positon to " << positionX << " " << positionY << " " << positionTheta);
        vecPosition->at(0) = positionX;
        vecPosition->at(1) = positionY;
        vecPosition->at(2) = positionTheta;
        informer->publish(vecPosition);
        showLine = false;
    }

    if (event == cv::EVENT_MOUSEMOVE) {
        mousePositionX = x;
        mousePositionY = y;
    }
}

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving compressed images")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for publishing new position")
            ("flipHorizontal", "Flip incoming image horizontally")
            ("flipVertical", "Flip incoming image vertically");

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

    INFO_MSG( "Scope: " << g_sInScope);

    // Evaluate flipping arguments
    int flipCode;
    bool flipping = false;
    flipHorizontal = false;
    flipVertical = false;
    if (vm.count("flipHorizontal")) {
        INFO_MSG("Flipping image horizontally");
        flipping = true;
        flipHorizontal = true;
        flipCode = 0; // flip around x
    }

    if (vm.count("flipVertical")) {
        INFO_MSG("Flipping image vertically");
        flipVertical = true;
        if (flipHorizontal) {
            flipCode = -1; // flip around both axes
        } else {
            flipping = true;
            flipCode = 1; // flip around y
        }
    }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::getFactory();

  // Create and start the listener
  rsb::ListenerPtr listener = factory.createListener(g_sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(imageQueue)));

  // Create publisher for new positions
  boost::shared_ptr<vecFloatConverter> converterVecFloat(new vecFloatConverter());
  converterRepository<std::string>()->registerConverter(converterVecFloat);
  informer = factory.createInformer< std::vector<float> >(g_sOutScope);

  // Setup up window with mouse handler
  std::string winName = g_sInScope;
  cv::namedWindow(winName);
  cv::setMouseCallback(winName, onMouse);

  // Exit the program if escape was pressed
  while (cv::waitKey(30) != KB_ESCAPE) {
    // Get the image as string

    if (!imageQueue->empty()) {
        INFO_MSG("Received new image");
        std::string imageJpg = *(imageQueue->pop());
        // Copy to a vector
        std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
        // Decode the image
        origImage = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
    }

    // Check if we already have a valid image
    if (origImage.size().area() > 0) {
        // Get the last original image (without drawn lines)
        cv::Mat image = origImage.clone();

        // Flip the image according to program options
        if (flipping) {
            cv::flip(image, image, flipCode);
        }

        // Draw line
        if (showLine) {
            cv::line(image, cv::Point(positionX, positionY), cv::Point(mousePositionX, mousePositionY), cv::Scalar(0,0,255));
        }

        cv::imshow(winName, image);
    }
  }

  return 0;

}
