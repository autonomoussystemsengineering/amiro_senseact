#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <converter/matConverter/matConverter.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

// For program options
#include <boost/program_options.hpp>

// RSC
#include <rsc/misc/SignalWaiter.h>
// Protoconverter
#include <rsb/converter/ProtocolBufferConverter.h>
// Prototypes
#include <rst/vision/Image.pb.h>
#ifndef RST_013_USED
# include <rst/vision/EncodedImage.pb.h>
#endif

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

using namespace boost;
using namespace std;
// using namespace cv;
using namespace rsb;
// using namespace muroxConverter; // The namespace for the own converters
using namespace rsb::converter;

static std::string g_sOutScope = "/image";
static int g_iDevice        = 0;
static int imageCompression = 0;
static int compressionValue = -1;

int main(int argc, char ** argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.")
    ("outscope,o", po::value<std::string>(&g_sOutScope), "Scope for sending images via RSB.")
    ("device,d", po::value<int>(&g_iDevice), "Number of device /dev/video#.")
    ("flip,f", "Flag to flip the output image around x and y axis")
    ("compression,c", po::value<int>(&compressionValue)->default_value(compressionValue), "Enable image compression with value betweeen 0-100.");

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

  INFO_MSG("Scope: " << g_sOutScope)
  INFO_MSG("Device: " << g_iDevice)
  INFO_MSG("compression " << compressionValue)

  if(compressionValue >= 0)
    imageCompression = 1;

  // //////////////////////////////////////////////////////////////////////////////////////////////////
  //	Register our converter within the collection of converters for
  //	the string wire-type (which is used for arrays of octets in C++).
  //	shared_ptr<MatConverter> converter(new MatConverter());
  //	converterRepository<std::string>()->registerConverter(converter);

  // Register new converter for std::vector<rst::vision::Image>


  rsb::Factory &factory = rsb::getFactory();
  Informer<rst::vision::Image>::Ptr informer;
  #ifdef RST_013_USED
  Informer<std::string>::Ptr informerCompressed;
  #else
  Informer<rst::vision::EncodedImage>::Ptr informerCompressed;
  #endif
  // Create the informer
  if (imageCompression) {
    #ifdef RST_013_USED
    // boost::shared_ptr< rsb::converter::ProtocolBufferConverter<std::string> >
    //  converter(new rsb::converter::ProtocolBufferConverter<std::string>());
    //  rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informerCompressed = factory.createInformer<std::string>(Scope(g_sOutScope));
    #else
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informerCompressed = factory.createInformer<rst::vision::EncodedImage>(Scope(g_sOutScope));
    #endif
  } else {
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::Image> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::vision::Image>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informer = factory.createInformer<rst::vision::Image>(Scope(g_sOutScope));
  }
  //	Informer<cv::Mat>::Ptr informer = getFactory().createInformer<cv::Mat> (Scope(g_sOutScope));
  // //////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // int ex = static_cast<int>(cam.get(CV_CAP_PROP_FOURCC));
  // cv::Size S = cv::Size((int) cam.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
  //              (int) cam.get(CV_CAP_PROP_FRAME_HEIGHT));
  // ////Transform from int to char via Bitwise operators
  // char EXT[] = {(char)(ex & 0XFF),(char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24),0};
  // INFO_MSG("Input frame resolution: Width=" << S.width << "  Height=" << S.height)
  // INFO_MSG("Get video frame with following FOURCC code: " << EXT )

  cam.set(CV_CAP_PROP_CONVERT_RGB, static_cast<double>(1));
  // cam.set(CV_CAP_PROP_FORMAT,CV_8UC3);

  // Open the device /dev/video<g_iDevice>
  cam.open(g_iDevice);
  // Allocate a frame object to store the picture
  //	shared_ptr<cv::Mat> frame(new cv::Mat);
  cv::Mat frame;

  // stuff for compression
  std::vector<unsigned char> buff;
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = compressionValue; // default(95) 0-100

  // Process the cam forever
  while (true) {
    // Save the actual picture to the frame object
    cam >> frame;
    if(vm.count("flip"))
      cv::flip(frame, frame, -1);
    #ifndef NDEBUG
    cv::imshow("frame", frame);
    cv::waitKey(1);
    #endif
    if (imageCompression) {
      cv::imencode(".jpg", frame, buff, param);
      #ifdef RST_013_USED
      boost::shared_ptr<std::string> frameJpg(new std::string(buff.begin(), buff.end()));
      informerCompressed->publish(frameJpg);
      INFO_MSG("Compressed Image published")
      #else
      boost::shared_ptr<rst::vision::EncodedImage> encodedImage(new rst::vision::EncodedImage());
      encodedImage->set_encoding(rst::vision::EncodedImage::JPG);
      encodedImage->set_data(std::string(reinterpret_cast<const char *>(&buff[0]), buff.size()));
      informerCompressed->publish(encodedImage);
      INFO_MSG("Encoded Image published")
      #endif
    } else {
      boost::shared_ptr<rst::vision::Image> image(new rst::vision::Image());
      image->set_width(frame.size().width);
      image->set_height(frame.size().height);
      image->set_data_order(rst::vision::Image::DATA_INTERLEAVED);
      image->set_data((void*)frame.data, frame.size().width * frame.size().width * 3); // conversion from uchar* tp char*

      // image->set_data(reinterpret_cast<char *>(frame.data)); // conversion from uchar* tp char*
      // Send the data.
      informer->publish(image);
      INFO_MSG("Image published")
    }
  }

  // Free the cam
  cam.release();

  return 0;
} // main
