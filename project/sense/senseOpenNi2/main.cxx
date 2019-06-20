#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <converter/iplImageConverter/IplImageConverter.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// For program options
#include <boost/program_options.hpp>

// RST
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

#include <stdio.h>
#include <OpenNI.h>


#define SAMPLE_READ_WAIT_TIMEOUT 2000 // 2000ms

using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;

static std::string g_sOutScope = "/depthImage";

int main(int argc, char ** argv) {
  namespace po = boost::program_options;

  static int imageCompression = 0;
  static int compressionValue = -1;
  static bool justPrint       = false;
  int depthMode = -1;

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.")
    ("outscope,o", po::value<std::string>(&g_sOutScope), "Scope for sending images.")
    ("compression,c", po::value<int>(&compressionValue)->default_value(compressionValue), "Enable image compression with value betweeen 0-100.")
    ("depthMode,d", po::value<int>(&depthMode), "Mode of Depth Image.")
    ("printInfo,p", "Flag if just the camera infos shall be printed. The tool closes afterwards.");

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
  justPrint = vm.count("printInfo");

  INFO_MSG("Scope: " << g_sOutScope)
  INFO_MSG("compression " << compressionValue)
  if (depthMode >= 0) INFO_MSG("Mode of Depth Image will be set to " << depthMode);


  if (compressionValue >= 0)
    imageCompression = 1;

  // //////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::getFactory();

  // Create the informer

  Informer<rst::vision::Image>::Ptr informer;
  #ifdef RST_013_USED
  Informer<std::string>::Ptr informerCompressed;
  #else
  Informer<rst::vision::EncodedImage>::Ptr informerCompressed;
  #endif

  if (imageCompression) {
    #ifdef RST_013_USED
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

  // stuff for compression
  std::vector<unsigned char> buff;
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = compressionValue; // default(95) 0-100

  // //////////////////////////////////////////////////////////////////////////////////////////////////

  Status rc = OpenNI::initialize();
  if (rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }

  Device device;
  rc = device.open(ANY_DEVICE);
  if (rc != STATUS_OK) {
    printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    return 2;
  }

  VideoFrameRef depthImage;
  VideoStream depthStream;

  // get depth stream
  const SensorInfo * depthinfo = device.getSensorInfo(SENSOR_DEPTH);
  if (depthinfo != NULL) {
    rc = depthStream.create(device, SENSOR_DEPTH);
    if (rc != STATUS_OK) {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
      return 3;
    }
    const openni::Array<VideoMode>& modes = depthinfo->getSupportedVideoModes();
    if (justPrint) {
      INFO_MSG("Modes of depth stream:");
      for (int i = 0; i < modes.getSize(); i++) {
        INFO_MSG(
          " " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat()
              << " format");
      }
      return 0;
    }
    if (depthMode < modes.getSize() && depthMode >= 0) {
      rc = depthStream.setVideoMode(modes[depthMode]);
      if (rc != STATUS_OK) {
        ERROR_MSG("Could not set mode of depth stream: " << OpenNI::getExtendedError());
        return EXIT_FAILURE;
      }
    }
  }

  rc = depthStream.start();
  if (rc != STATUS_OK) {
    printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    return 4;
  }

  double minVal, maxVal;
  float alpha, beta;

  while (true) {
    int changedStreamDummy;
    VideoStream * depthStreamTemp = &depthStream;
    rc = OpenNI::waitForAnyStream(&depthStreamTemp, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != STATUS_OK) {
      printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
      continue;
    }

    rc = depthStream.readFrame(&depthImage);
    if (rc != STATUS_OK) {
      printf("Read failed!\n%s\n", OpenNI::getExtendedError());
      continue;
    }

    if (depthImage.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && depthImage.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM) {
      printf("Unexpected frame format\n");
      continue;
    }

    const openni::DepthPixel * depthBuffer = (const openni::DepthPixel *) depthImage.getData();
    cv::Mat depthMat(depthImage.getHeight(), depthImage.getWidth(), CV_16UC1);
    memcpy(depthMat.data, depthBuffer, depthImage.getHeight() * depthImage.getWidth() * sizeof(uint16_t));
    //

    cv::minMaxLoc(depthMat, &minVal, &maxVal);
    alpha = 255.0 / maxVal;
    beta  = 255.0 - maxVal * alpha;
    depthMat.convertTo(depthMat, CV_8UC1, alpha, beta);
    cv::imshow("depth", depthMat);
    cv::waitKey(1);

    if (imageCompression) {
      cv::imencode(".png", depthMat, buff, param);
      #ifdef RST_013_USED
      boost::shared_ptr<std::string> framePng(new std::string(buff.begin(), buff.end()));
      informerCompressed->publish(framePng);
      INFO_MSG("Compressed Image published")
      #else
      boost::shared_ptr<rst::vision::EncodedImage> encodedImage(new rst::vision::EncodedImage());
      encodedImage->set_encoding(rst::vision::EncodedImage::JPG);
      encodedImage->set_data(std::string(reinterpret_cast<const char *>(&buff[0]), buff.size()));
      informerCompressed->publish(encodedImage);
      INFO_MSG("Encoded Image published")
      #endif
    } else {
      boost::shared_ptr<rst::vision::Image> rstVisionImage(new rst::vision::Image());
      rstVisionImage->set_width(depthMat.size().width);
      rstVisionImage->set_height(depthMat.size().height);
      rstVisionImage->set_data((void *) depthMat.data, depthMat.size().height * depthMat.size().width);
      rstVisionImage->set_channels(1);
      rstVisionImage->set_color_mode(rst::vision::Image::COLOR_GRAYSCALE);
      rstVisionImage->set_depth(rst::vision::Image::DEPTH_8U);
      // Send the data.
      informer->publish(rstVisionImage);
      INFO_MSG("Image published")
    }
  }

  depthStream.stop();
  depthStream.destroy();
  device.close();
  OpenNI::shutdown();

  return 0;
} // main
