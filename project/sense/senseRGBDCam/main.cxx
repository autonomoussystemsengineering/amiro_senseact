// ============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Reading images from depth cameras.
// ============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef RST_013_USED
# include <opencv2/nonfree/nonfree.hpp>
#else
# include <opencv2/xfeatures2d.hpp>
#endif
// #include <converter/iplImageConverter/IplImageConverter.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

#include <rsb/Informer.h>
#include <rsb/Version.h>
#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>
#include <rsc/misc/SignalWaiter.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#include <ctime>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>


#define SAMPLE_READ_WAIT_TIMEOUT 2000 // 2000ms

using namespace std;
using namespace boost::chrono;
using namespace cv;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;


// scope names
std::string rgbScope   = "/image/rgb";
std::string depthScope = "/image/depth";
static unsigned int g_uiQuality = 100;
bool sendImage  = false;
bool justPrint  = false;
bool debugImage = false;
bool showIR     = false;

int main(int argc, char ** argv) {
  namespace po = boost::program_options;
  int rgbMode         = -1;
  int depthMode       = -1;
  int irMode          = -1;
  unsigned int period = 10; // ms

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.")
    ("RGBOutscope,r", po::value<std::string>(&rgbScope), "Scope for sending RGB image.")
    ("DepthOutscope,d", po::value<std::string>(&depthScope), "Scope for sending depth image.")
    ("printTime,t", "Prints the time")
    ("rgbMode", po::value<int>(&rgbMode), "Mode of RGB Image.")
    ("depthMode", po::value<int>(&depthMode), "Mode of Depth Image.")
    ("irMode", po::value<int>(&irMode), "Mode of IR Image.")
    ("period", po::value<unsigned int>(&period), "Period between two image fetches in ms (default: 10).")
    ("compression,c", po::value<unsigned int>(&g_uiQuality), "Compression value [0,100]")
    ("showIR", "Flag if the IR image shall be shown instead of the RGB Image.")
    ("sendImage,s", "Flag if the image shall be converted to OpenCV and send via RSB.")
    ("printInfo,p", "Flag if just the camera infos shall be printed. The tool closes afterwards.")
    ("debugImage,i", "Flag if over the RGB scope the RGB image and depth image shall be send in one image.");

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

  sendImage  = vm.count("sendImage");
  justPrint  = vm.count("printInfo");
  debugImage = vm.count("debugImage");
  showIR     = vm.count("showIR");

  if (!showIR && rgbMode >= 0) INFO_MSG("Mode of RGB Image will be set to " << rgbMode);
  if (showIR && irMode >= 0) INFO_MSG("Mode of IR Image will be set to " << irMode);
  if (depthMode >= 0) INFO_MSG("Mode of Depth Image will be set to " << depthMode);

  std::string rgbName = "RGB";
  if (showIR) {
    rgbName = "IR";
    rgbMode = irMode;
  }

  // Initialize clock
  system_clock::time_point systime1, systime2;
  milliseconds mstime;

  if (vm.count("printTime")) systime1 = system_clock::now();
  if (vm.count("printTime")) DEBUG_MSG("Starting RSB Initialization");


  // +++++ RSB Initialization +++++

  rsb::Factory &factory = rsb::getFactory();

  INFO_MSG("RSB Scopes:");
  INFO_MSG(" -> sending images:     " << rgbScope);
  INFO_MSG(" -> sending laser data: " << depthScope);
  INFO_MSG("");

  // +++ Informer +++
  // create informer for RGB images
  rsb::Informer<std::string>::Ptr RGBInformer = factory.createInformer<std::string>(rgbScope);
  // create informer for depth images
  rsb::Informer<std::string>::Ptr DepthInformer = factory.createInformer<std::string>(depthScope);


  if (vm.count("printTime")) systime2 = system_clock::now();
  if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2 - systime1);
  if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");


  // +++++ OpenNI Initialization +++++

  if (vm.count("printTime")) systime1 = system_clock::now();
  if (vm.count("printTime")) DEBUG_MSG("Starting OpenNI Initialization");
  VideoFrameRef rgbImage, depthImage;
  VideoStream rgbStream, depthStream;
  Device device;

  cv::Mat rgbMat, depthMat;

  // Compress data
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(g_uiQuality);
  vector<uchar> bufRgb, bufDepth, bufIR;

  Status respON = OpenNI::initialize();
  if (respON != STATUS_OK) {
    ERROR_MSG("Initialize failed: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  DEBUG_MSG("Open device");
  // open device and start stream
  respON = device.open(ANY_DEVICE);
  if (respON != STATUS_OK) {
    ERROR_MSG("Could not open device: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  // get rgb stream
  const SensorInfo * rgbinfo;
  if (showIR) {
    rgbinfo = device.getSensorInfo(SENSOR_IR);
  } else {
    rgbinfo = device.getSensorInfo(SENSOR_COLOR);
  }
  if (rgbinfo != NULL) {
    if (showIR) {
      respON = rgbStream.create(device, SENSOR_IR);
    } else {
      respON = rgbStream.create(device, SENSOR_COLOR);
    }
    if (respON != STATUS_OK) {
      ERROR_MSG("Could not create " << rgbName << " stream: " << OpenNI::getExtendedError());
      return EXIT_FAILURE;
    }
    const openni::Array<VideoMode>& modes = rgbinfo->getSupportedVideoModes();
    if (justPrint) {
      INFO_MSG("Modes of " << rgbName << " stream:");
      for (int i = 0; i < modes.getSize(); i++) {
        INFO_MSG(
          " " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat()
              << " format");
      }
    }
    if (rgbMode < modes.getSize() && rgbMode >= 0) {
      respON = rgbStream.setVideoMode(modes[rgbMode]);
      if (respON != STATUS_OK) {
        ERROR_MSG("Could not set mode of " << rgbName << " stream: " << OpenNI::getExtendedError());
        return EXIT_FAILURE;
      }
    }
  }

  // get depth stream
  const SensorInfo * depthinfo = device.getSensorInfo(SENSOR_DEPTH);
  if (depthinfo != NULL) {
    respON = depthStream.create(device, SENSOR_DEPTH);
    if (respON != STATUS_OK) {
      ERROR_MSG("Could not create depth stream: " << OpenNI::getExtendedError());
      return EXIT_FAILURE;
    }
    const openni::Array<VideoMode>& modes = depthinfo->getSupportedVideoModes();
    if (justPrint) {
      INFO_MSG("Modes of depth stream:");
      for (int i = 0; i < modes.getSize(); i++) {
        INFO_MSG(
          " " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat()
              << " format");
      }
    }
    if (depthMode < modes.getSize() && depthMode >= 0) {
      respON = depthStream.setVideoMode(modes[depthMode]);
      if (respON != STATUS_OK) {
        ERROR_MSG("Could not set mode of depth stream: " << OpenNI::getExtendedError());
        return EXIT_FAILURE;
      }
    }
  }

  // start streams
  respON = rgbStream.start();
  if (respON != STATUS_OK) {
    ERROR_MSG("Could not start the " << rgbName << " stream: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }
  respON = depthStream.start();
  if (respON != STATUS_OK) {
    ERROR_MSG("Could not start the depth stream: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  VideoStream * rgbStreamTemp = &rgbStream;
  int changedRgbStreamDummy;
  respON = OpenNI::waitForAnyStream(&rgbStreamTemp, 1, &changedRgbStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
  if (respON != STATUS_OK) {
    ERROR_MSG("Wait for " << rgbName << " stream failed (timeout is " << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }
  VideoStream * depthStreamTemp = &depthStream;
  int changedDepthStreamDummy;
  respON = OpenNI::waitForAnyStream(&depthStreamTemp, 1, &changedDepthStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
  if (respON != STATUS_OK) {
    ERROR_MSG("Wait for depth stream failed (timeout is " << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  // read first image
  respON = rgbStream.readFrame(&rgbImage);
  if (respON != STATUS_OK) {
    ERROR_MSG("Read " << rgbName << " image failed: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  // read first scan
  respON = depthStream.readFrame(&depthImage);
  if (respON != STATUS_OK) {
    ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
    return EXIT_FAILURE;
  }

  if (justPrint) {
    int imageHeight = rgbImage.getHeight();
    int imageWidth  = rgbImage.getWidth();
    int scanHeight  = depthImage.getHeight();
    int scanWidth   = depthImage.getWidth();
    INFO_MSG(rgbName << " Image size " << imageWidth << "x" << imageHeight << " with laser image " << scanWidth << "x" << scanHeight);
  }

  double minVal, maxVal;
  float alpha, beta;
  while (!justPrint) {
    if (vm.count("printTime")) DEBUG_MSG("-----------------------------------------------")
      systime1 = system_clock::now();
    if (vm.count("printTime")) DEBUG_MSG("Reading camera frames");
    respON = rgbStream.readFrame(&rgbImage);
    if (respON != STATUS_OK) {
      ERROR_MSG("Read " << rgbName << " image failed: " << OpenNI::getExtendedError());
      return EXIT_FAILURE;
    }
    respON = depthStream.readFrame(&depthImage);
    if (respON != STATUS_OK) {
      ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
      return EXIT_FAILURE;
    }
    if (vm.count("printTime")) systime2 = system_clock::now();
    if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2 - systime1);
    if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

    if (!sendImage) {
      int rgbHeight = rgbImage.getHeight();
      int rgbWidth  = rgbImage.getWidth();
      int dHeight   = depthImage.getHeight();
      int dWidth    = depthImage.getWidth();
      INFO_MSG(" => " << rgbName << " Image loaded: " << rgbWidth << "/" << rgbHeight << ", " << dWidth << "/" << dHeight);
    } else {
      // Convert to cv::Mat
      if (vm.count("printTime")) systime1 = system_clock::now();
      if (vm.count("printTime")) DEBUG_MSG("Conversion: Transform OpenNI to OpenCV");

      const openni::RGB888Pixel * rgbBuffer = (const openni::RGB888Pixel *) rgbImage.getData();
      rgbMat.create(rgbImage.getHeight(), rgbImage.getWidth(), CV_8UC3);
      memcpy(rgbMat.data, rgbBuffer, 3 * rgbImage.getHeight() * rgbImage.getWidth() * sizeof(uint8_t));

      const openni::DepthPixel * depthBuffer = (const openni::DepthPixel *) depthImage.getData();
      Mat depthdata(depthImage.getHeight(), depthImage.getWidth(), CV_16UC1);
      memcpy(depthdata.data, depthBuffer, depthImage.getHeight() * depthImage.getWidth() * sizeof(uint16_t));

      if (vm.count("printTime")) systime2 = system_clock::now();
      if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2 - systime1);
      if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

      if (vm.count("printTime")) systime1 = system_clock::now();
      if (vm.count("printTime")) DEBUG_MSG("Conversion: Convert to correctly flipped " << rgbName << " images");

      cvtColor(rgbMat, rgbMat, CV_BGR2RGB);
      flip(rgbMat, rgbMat, 1);

      Mat depthDataImage1C;
      minMaxLoc(depthdata, &minVal, &maxVal);
      alpha = 255.0 / maxVal;
      beta  = 255.0 - maxVal * alpha;
      depthdata.convertTo(depthDataImage1C, CV_8UC1, alpha, beta);
      cvtColor(depthDataImage1C, depthMat, CV_GRAY2RGB);
      flip(depthMat, depthMat, 1);

      if (vm.count("printTime")) systime2 = system_clock::now();
      if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2 - systime1);
      if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

      // Compress image
      if (vm.count("printTime")) systime1 = system_clock::now();
      if (vm.count("printTime")) DEBUG_MSG("Compressing and sending images");
      if (debugImage) {
        Mat publicMat(max(rgbMat.rows, depthMat.rows), rgbMat.cols + depthMat.cols, CV_8UC3);
        Mat part[3];
        part[0] = Mat(publicMat, Rect(0, 0, rgbMat.cols, rgbMat.rows));
        part[1] = Mat(publicMat, Rect(rgbMat.cols, 0, depthMat.cols, depthMat.rows));
        rgbMat.copyTo(part[0]);
        depthMat.copyTo(part[1]);
        imencode(".jpg", publicMat, bufRgb, compression_params);
        ::boost::shared_ptr<std::string> rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
        RGBInformer->publish(rgbFrameJpg);
      } else {
        imencode(".jpg", rgbMat, bufRgb, compression_params);
        imencode(".jpg", depthMat, bufDepth, compression_params);
        ::boost::shared_ptr<std::string> rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
        RGBInformer->publish(rgbFrameJpg);
        ::boost::shared_ptr<std::string> depthFrameJpg(new std::string(bufDepth.begin(), bufDepth.end()));
        DepthInformer->publish(depthFrameJpg);
      }
    }
    systime2 = system_clock::now();
    mstime   = duration_cast<milliseconds>(systime2 - systime1);
    if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

    usleep(period * 1000);
  }

  rgbStream.stop();
  rgbStream.destroy();
  depthStream.stop();
  depthStream.destroy();

  device.close();
  OpenNI::shutdown();

  return EXIT_SUCCESS;
} // main
