//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This program detects the next object in front of the robot
//               based on the proximity sensor values. The object marking is
//               based on seeded region growing.
//============================================================================


#include <iostream>
#include <fstream>

#include <time.h>
#include <dirent.h>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// Video 4 Linux
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <fcntl.h>

// protocol defines
std::string COMMAND_START = "START";
std::string COMMAND_STOP = "STOP";
std::string COMMAND_QUIT = "QUIT";


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;
using namespace rsb::converter;
using namespace cv;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

#include <Constants.h>
#include <sensorModels/VCNL4020Models.h>
using namespace amiro;

static std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
static std::string g_sImageScope = "/frontObject/image";
static std::string g_sInScope = "/frontObject/command";
static std::string g_sDevice = "/dev/video6";
static unsigned int g_uiSendNthFrame = 3;
static unsigned int g_uiQuality = 85;

bool sendingPic = false;
bool debugging = false;

float colorDifference = 50.0;

float camSideDist = 0.12; // m
float camGroundDist = 0.115; // m

float actionRange = 0.15; // m


Mat markInOrigin(Mat origin, Mat mark, int red, int green, int blue) {
    Mat originMark(origin.rows, origin.cols, CV_8UC3);
    origin.copyTo(originMark);
    for (int x=0; x < origin.cols; x++) {
        for (int y=0; y < origin.rows; y++) {
            if (mark.at<uint8_t>(y,x) != 0) {
                originMark.at<Vec3b>(y,x) = Vec3b(blue, green, red);
            }
        }
    }
    return originMark;
}

float colorDist(Vec3b color, Vec3b refColor) {
    int blue = color.val[0] - refColor.val[0];
    int green = color.val[1] - refColor.val[1];
    int red = color.val[2] - refColor.val[2];
    return sqrt(blue*blue + green*green + red*red);
}


int main (int argc, char * const argv[]) {
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("imagescope,i", po::value<std::string> (&g_sImageScope),"Scope for sending images.")
            ("proxscope,p", po::value<std::string> (&proxSensorInscopeObstacle),"Scope for receiving obstacle proximity sensor values.")
            ("commandscope,c", po::value<std::string> (&g_sInScope),"Scope for receiving commands.")
            ("device,d", po::value<std::string> (&g_sDevice),"Location of camera device (default: /dev/video6).")
            ("sendNthFrame,n", po::value<unsigned int> (&g_uiSendNthFrame),"Send only every n'th frame for visualization for performance (3).")
            ("quality,q", po::value<unsigned int> (&g_uiQuality),"Quality of JPEG compression [0 .. 100].")
            ("colorThreshold,t", po::value<float> (&colorDifference),"Color threshold for region growing (default: 50).")
            ("continuousSending,s", "Sends the camera frames continued.")
            ("debug", "Activates debugging which includes generated pictures and additional console information.");

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

    sendingPic = vm.count("continuousSending");
    debugging = vm.count("debug");
    
    INFO_MSG("Picture scope: " << g_sImageScope);
    INFO_MSG("Command scope: " << g_sInScope);
    INFO_MSG("Device: " << g_sDevice);
    INFO_MSG("sendNthFrame: " << g_uiSendNthFrame);
    INFO_MSG("JPEG Quality: " << g_uiQuality);

    float camAngleX = acos(1 - camSideDist*camSideDist / (2 * (camGroundDist*camGroundDist + camSideDist*camSideDist/4)));
    float camAngleOffset = (camAngleX - ringproximity::SENSOR_DIST_ANGULAR) / 2.0;
    float camSensorFactor = camAngleX/(ringproximity::SENSOR_DIST_ANGULAR);

    // Compress image parameter
    vector<uchar> buf;
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(g_uiQuality);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    rsb::Factory &factory = rsb::getFactory();

    // Register new converter for std::vector<int>
    boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

    // Create the informer
    Informer<std::string>::Ptr imageInformer = getFactory().createInformer<std::string> (Scope(g_sImageScope));

    // Create and start the command listener
    rsb::ListenerPtr listener = factory.createListener(g_sInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

    listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));


    // prepare RSB listener for the IR data
    rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int> > > >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int> > >(1));
    proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int> >(proxQueueObstacle)));

    ////////////////////////////////////////////////////////////////////////////////////////////////////

//-------------Handle camera as native V4L2 device - Start -
        struct v4l2_format fmt;
        int rc, fd = -1;
        unsigned int i, length;
        const char *dev_name = g_sDevice.c_str();
        char out_name[256];
        FILE *fout;

        fd = v4l2_open(dev_name, O_RDWR, 0);
        if (fd < 0) {
                fprintf(stderr, "Cannot open device %s\n", dev_name);
                exit(EXIT_FAILURE);
        }
        
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640;  // Maximum width
        fmt.fmt.pix.height      = 480;  // Maximum hight
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;//V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        rc = v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt);
        printf("Resolution: %d x %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
        printf("format: %d \n", fmt.fmt.pix.pixelformat);
        printf("field: %d \n", fmt.fmt.pix.field);
        if (rc == -1) {
                fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (fmt.fmt.pix.sizeimage < (fmt.fmt.pix.width * fmt.fmt.pix.height)) {
                fprintf(stderr, "Error: Driver is sending image at %dx%d with size of %d\n",
                        fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
                exit(EXIT_FAILURE);
        }


  // Allocate a frame object to store the picture
   cv::Mat next_frame(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC3);
   cv::Mat frameTmp(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC2);
  

//-------------Handle camera as native V4L2 device - End -

    unsigned int frameIdx = 1;
    Mat objectFrame, bluredImage;
    int objectPosition = 0;
    bool objectFound = false;

    while ( true/* TODO in rsb0.12 rsc::misc::lastArrivedSignal() == rsc::misc::Signal::NO_SIGNAL*/) {
        // Save the current frame to the frame object
        length = v4l2_read(fd, (void*) frameTmp.data , fmt.fmt.pix.sizeimage);
        if (length == -1) {
          ERROR_MSG(" " <<  errno << " " <<  strerror(errno));
          exit(EXIT_FAILURE);
        }
        // Show only every "g_uiSendNthFrame" image
        if (frameIdx >= g_uiSendNthFrame) {
          frameIdx = 1;
        } else {
          ++frameIdx;
          continue;
        }
        // Convert frame from YUY2 to RGB
        cvtColor(frameTmp,next_frame,CV_YUV2BGR_YUY2);
    
        next_frame.copyTo(objectFrame);
        next_frame.copyTo(bluredImage);
        Mat objectMark = Mat::zeros(bluredImage.rows, bluredImage.cols, CV_8UC1);
        blur(bluredImage, bluredImage, Size(5,5));

        if (objectFound && !proxQueueObstacle->empty()) {
            // Read the proximity data
            boost::shared_ptr<std::vector<int> > sensorValuesObstacle = boost::static_pointer_cast<std::vector<int> >(proxQueueObstacle->pop());
            float leftDist = VCNL4020Models::obstacleModel(0, sensorValuesObstacle->at(3)) + 0.05;
            float rightDist = VCNL4020Models::obstacleModel(0, sensorValuesObstacle->at(4)) + 0.05;
//                std::cout << "dists: " << leftDist << " - " << rightDist << " (" << sensorValuesObstacle->at(3) << " - " << sensorValuesObstacle->at(4) << ")" << std::endl;

            float lx = leftDist*cos(5.0 * ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
            float ly = leftDist*sin(5.0 * ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
            float rx = rightDist*cos(3.0 * ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
            float ry = rightDist*sin(3.0 * ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
            float tx = rx-lx;
            float ty = ry-ly;
            float newAngle = -atan(ty/tx) + camAngleX/2.0;
            if (newAngle < 0) newAngle = 0;
            if (newAngle > camAngleX) newAngle = camAngleX;
//                std::cout << "angle: " << (newAngle*180/M_PI) << std::endl;

            objectPosition = newAngle/camAngleX * objectFrame.cols;
            if (objectPosition < 0) objectPosition = 0;
            if (objectPosition >= objectFrame.cols) objectPosition = objectFrame.cols-1;
//                std::cout << "Pos in picture: " << objectPosition << " (of " << objectFrame.cols << ")" << std::endl;

            // mark focus
            if (debugging) {
                int rectHalfSize = 5;
                int xStart = objectPosition-rectHalfSize;
                int xEnd = objectPosition+rectHalfSize;
                if (xStart < 0) xStart = 0;
                if (xEnd >= objectFrame.cols) xEnd = objectFrame.cols-1;
                for (int x=xStart; x<=xEnd; x++) {
                    objectFrame.at<Vec3b>(objectFrame.rows/2-rectHalfSize, x) = Vec3b(0, 0, 255);
                    objectFrame.at<Vec3b>(objectFrame.rows/2+rectHalfSize, x) = Vec3b(0, 0, 255);
                }
                for (int y= objectFrame.rows/2-rectHalfSize; y <= objectFrame.rows/2+rectHalfSize; y++) {
                    objectFrame.at<Vec3b>(y, xStart) = Vec3b(0, 0, 255);
                    objectFrame.at<Vec3b>(y, xEnd) = Vec3b(0, 0, 255);
                }
            }

            // mark object
            Mat visited = Mat::zeros(bluredImage.rows, bluredImage.cols, CV_8UC1);
            vector<int> xQueue;
            vector<int> yQueue;
            xQueue.push_back(objectPosition);
            yQueue.push_back(objectFrame.rows/2);
            Vec3b meanColor = bluredImage.at<Vec3b>(yQueue.at(0), xQueue.at(0));
            visited.at<uint8_t>(yQueue.at(0), xQueue.at(0)) = 1;
            int pixCounter = 0;
            int minX = xQueue.at(0);
            int maxX = xQueue.at(0);
            int minY = yQueue.at(0);
            int maxY = yQueue.at(0);
            while (pixCounter < xQueue.size()) {
                int xPixel = xQueue.at(pixCounter);
                int yPixel = yQueue.at(pixCounter);
                pixCounter++;
                Vec3b refColor = bluredImage.at<Vec3b>(yPixel, xPixel);

                if (colorDist(meanColor, refColor) < colorDifference) {
                    objectMark.at<uint8_t>(yPixel, xPixel) = 255;
                    if (xPixel < minX) {
                        minX = xPixel;
                    } else if (xPixel > maxX) {
                        maxX = xPixel;
                    }
                    if (yPixel < minY) {
                        minY = yPixel;
                    } else if (yPixel > maxY) {
                        maxY = yPixel;
                    }

                    int xStart = xPixel-1;
                    int xEnd = xPixel+1;
                    int yStart = yPixel-1;
                    int yEnd = yPixel+1;
                    if (xStart < 0) xStart = 0;
                    if (xEnd >= bluredImage.cols) xEnd = bluredImage.cols-1;
                    if (yStart < 0) yStart = 0;
                    if (yEnd >= bluredImage.rows) yEnd = bluredImage.rows-1;
                    for (int xi=xStart; xi<=xEnd; xi++) {
                        for (int yi=yStart; yi<=yEnd; yi++) {
                            if (visited.at<uint8_t>(yi,xi) == 0) {
                                visited.at<uint8_t>(yi, xi) = 1;
                                xQueue.push_back(xi);
                                yQueue.push_back(yi);
                            }
                        }
                    }
                }
            }

            for (int x=minX; x<=maxX; x++) {
                objectFrame.at<Vec3b>(minY, x) = Vec3b(0, 255, 255);
                objectFrame.at<Vec3b>(maxY, x) = Vec3b(0, 255, 255);
            }
            for (int y=minY; y<=maxY; y++) {
                objectFrame.at<Vec3b>(y, minX) = Vec3b(0, 255, 255);
                objectFrame.at<Vec3b>(y, maxX) = Vec3b(0, 255, 255);
            }
        }

        // Send the image
        if (objectFound || sendingPic) {
            Mat flippedPic;
            if (debugging) {
                Mat objectFrameS(objectFrame.rows/2, objectFrame.cols/2, CV_8UC3);
                Mat debugImage(objectFrameS.rows*2, objectFrameS.cols*2, CV_8UC3);
                resize(objectFrame, objectFrameS, Size(objectFrameS.cols, objectFrameS.rows));
                objectFrameS.copyTo(debugImage(Rect(0, objectFrameS.rows, objectFrameS.cols, objectFrameS.rows)));

                Mat bluredImageS(objectFrameS.rows, objectFrameS.cols, CV_8UC3);
                resize(bluredImage, bluredImageS, Size(objectFrameS.cols, objectFrameS.rows));
                bluredImageS.copyTo(debugImage(Rect(objectFrameS.cols, objectFrameS.rows, bluredImageS.cols, bluredImageS.rows)));

                Mat objectMark3ch(objectMark.rows, objectMark.cols, CV_8UC3);
                Mat objectMark3chS(objectFrameS.rows, objectFrameS.cols, CV_8UC3);
                cvtColor(objectMark, objectMark3ch, CV_GRAY2RGB);
                resize(objectMark3ch, objectMark3chS, Size(objectFrameS.cols, objectFrameS.rows));
                objectMark3chS.copyTo(debugImage(Rect(0, 0, objectMark3chS.cols, objectMark3chS.rows)));

                Mat originMark = markInOrigin(next_frame, objectMark, 255, 255, 0);
                Mat originMarkS(objectFrameS.rows, objectFrameS.cols, CV_8UC3);
                resize(originMark, originMarkS, Size(objectFrameS.cols, objectFrameS.rows));
                originMarkS.copyTo(debugImage(Rect(objectFrameS.cols, 0, originMarkS.cols, originMarkS.rows)));

                flip(debugImage, flippedPic, 0);
            } else {
                flip(objectFrame, flippedPic, 0);
            }
            imencode(".jpg", flippedPic, buf, compression_params);
            boost::shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
            imageInformer->publish(frameJpg);
        }

        // Get command
        if (!commandQueue->empty()) {
            std::string command = *commandQueue->pop().get();
            // check for quit command
            if (command == COMMAND_QUIT) {
                INFO_MSG("Quit application.");
                break;
            // check for start command
            } else if (command == COMMAND_START) {
                objectFound = true;
//                    INFO_MSG("Start object detection.");
            // check for stop site command
            } else if (command == COMMAND_STOP) {
                objectFound = false;
//                    INFO_MSG("Stop object detection.");
            // otherwise it is an unknown command
            } else {
                INFO_MSG("Unknown command.");
            }
        }
    }
    // Free everything
    v4l2_close(fd);
    return 0/* TODO in rsb0.12 rsc::misc::lastArrivedSignal()*/;   
}

