//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This is the program for motion detection. It sends a motion
//               detection signal over RSB and can receive commands. If
//               activated it sends the camera frames (including motion
//               markers).
//============================================================================


#include <iostream>
#include <fstream>

#include <time.h>
#include <dirent.h>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

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


// protocol defines
std::string COMMAND_START = "START";
std::string COMMAND_STOP = "STOP";
std::string COMMAND_QUIT = "QUIT";

#define maxObjects 10
#define maxObjectSites 8


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
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

static std::string g_sImageScope = "/motionDetection/image";
static std::string g_sOutScope = "/motionDetection/detected";
static std::string g_sInScope = "/motionDetection/command";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;

const string EXT = ".jpg";
std::string directory = "/home/root/motionDetectionPics/";

bool sendingPic = false;
bool debugging = false;
bool trackMotion = true;
bool saveMotionPic = false;

// Format of directory
const string DIR_FORMAT = "%d%h%Y"; // 1Jan1970
const string FILE_FORMAT = DIR_FORMAT + "/" + "%d%h%Y_%H%M%S"; // 1Jan1970/1Jan1970_12153
const string CROPPED_FILE_FORMAT = DIR_FORMAT + "/cropped/" + "%d%h%Y_%H%M%S"; // 1Jan1970/cropped/1Jan1970_121539



// Check if the directory exists, if not create it
// This function will create a new directory if the image is the first
// image taken for a specific day
inline bool directoryExistsOrCreate(const char* pzPath)
{
    DIR *pDir;
    // directory doesn't exists -> create it
    if (pzPath == NULL || (pDir = opendir (pzPath)) == NULL) {
        if (mkdir(pzPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
            return false;
        }

    // if directory exists we opened it and we
    // have to close the directory again.
    } else if (pDir != NULL) {
        (void) closedir (pDir);
    }
    return true;
}

// When motion is detected we write the image to disk
//    - Check if the directory exists where the image will be stored.
//    - Build the directory and image names.
int incr = 0;
inline bool saveImg(Mat image, const string DIRECTORY, const string EXTENSION, const char * DIR_FORMAT, const char * FILE_FORMAT)
{
    stringstream ss;
    time_t seconds;
    struct tm * timeinfo;
    char TIME[80];
    time (&seconds);
    // Get the current time
    timeinfo = localtime (&seconds);
    
    // Create name for the date directory
    strftime (TIME,80,DIR_FORMAT,timeinfo);
    ss.str("");
    ss << DIRECTORY << TIME;
    if (!directoryExistsOrCreate(ss.str().c_str())) {
        return false;
    }
    ss << "/cropped";
    if (!directoryExistsOrCreate(ss.str().c_str())) {
        return false;
    }

    // Create name for the image
    strftime (TIME,80,FILE_FORMAT,timeinfo);
    ss.str("");
    if(incr < 100) incr++; // quick fix for when delay < 1s && > 10ms, (when delay <= 10ms, images are overwritten)
    else incr = 0;
    ss << DIRECTORY << TIME << static_cast<int>(incr) << EXTENSION;
    return imwrite(ss.str().c_str(), image);
}

// Check if there is motion in the result matrix
// count the number of changes and return.
inline int detectMotion(const Mat & motion, Mat & result, Mat & result_cropped, Mat & debugImage,
                 int x_start, int x_stop, int y_start, int y_stop,
                 int max_deviation,
                 Scalar & color)
{
    // calculate the standard deviation
    Scalar mean, stddev;
    meanStdDev(motion, mean, stddev);
    // if not to much changes then the motion is real (neglect agressive snow, temporary sunlight)
    if(stddev[0] < max_deviation)
    {
        int number_of_changes = 0;
        int min_x = motion.cols, max_x = 0;
        int min_y = motion.rows, max_y = 0;
        // loop over image and detect changes
        for(int j = y_start; j < y_stop; j+=2){ // height
            for(int i = x_start; i < x_stop; i+=2){ // width
                // check if at pixel (j,i) intensity is equal to 255
                // this means that the pixel is different in the sequence
                // of images (prev_frame, current_frame, next_frame)
                if(static_cast<int>(motion.at<uchar>(j,i)) == 255)
                {
                    number_of_changes++;
                    if(min_x>i) min_x = i;
                    if(max_x<i) max_x = i;
                    if(min_y>j) min_y = j;
                    if(max_y<j) max_y = j;
                }
            }
        }
        if(number_of_changes){
            //check if not out of bounds
            if(min_x-10 > 0) {
                min_x -= 10;
            } else {
                min_x = 0;
            }
            if(min_y-10 > 0) {
                min_y -= 10;
            } else {
                min_y = 0;
            }
            if(max_x+10 < result.cols-1) {
                max_x += 10;
            } else {
                max_x = result.cols-1;
            }
            if(max_y+10 < result.rows-1) {
                max_y += 10;
            } else {
                max_y = result.rows-1;
            }
            // draw rectangle round the changed pixel
            Point x(min_x,min_y);
            Point y(max_x,max_y);
            Rect rect(x,y);
            Mat cropped = result(rect);
            cropped.copyTo(result_cropped);
            rectangle(result,rect,color,1);

            result.copyTo(debugImage(Rect(0, 0, result.cols, result.rows)));
            Mat motion3ch(motion.rows, motion.cols, CV_8UC3);
            cvtColor(motion, motion3ch, CV_GRAY2RGB);
            motion3ch.copyTo(debugImage(Rect(result.cols, 0, motion3ch.cols, motion3ch.rows)));
        }
        return number_of_changes;
    }
    return 0;
}

int main (int argc, char * const argv[])
{
    // If more than 'there_is_motion' pixels are changed, we say there is motion
    // and store an image on disk
    int there_is_motion = 5;
    
    // Maximum deviation of the image, the higher the value, the more motion is allowed
    int max_deviation = 100;
  
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("imagescope,i", po::value < std::string > (&g_sImageScope),"Scope for sending images.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending recognition of motion detection.")
            ("commandscope,c", po::value < std::string > (&g_sInScope),"Scope for receiving commands.")
            ("directory", po::value < std::string > (&directory),"Directory where the images can be stored.")
            ("device,d", po::value < int > (&g_iDevice),"Number of video device (/dev/video<number>).")
            ("quality,q", po::value < unsigned int > (&g_uiQuality),"Quality of JPEG compression [0 .. 100].")
            ("motionsize,m", po::value < int > (&there_is_motion),"Minimum number of pixels which have been changed for motion detectin.")
            ("motiondeviation,r", po::value < int > (&max_deviation),"Maximum deviation of the image (the higher the value, the more motion is allowed).")
            ("waitForStart,w", "Does not start with motion detection immediately. It waits for start command.")
            ("sending,s", "Sends the taken snapshot over RSB.")
            ("saveMotion", "Sends the taken snapshot over RSB.")
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

    sendingPic = vm.count("sending");
    debugging = vm.count("debug");
    saveMotionPic = vm.count("saveMotion");
    trackMotion = vm.count("waitForStart") == 0;
    
    INFO_MSG("Output scope: " << g_sOutScope);
    INFO_MSG("Command scope: " << g_sInScope);
    if (sendingPic) {
        INFO_MSG("Picture scope: " << g_sImageScope);
    }
    INFO_MSG("Device: " << g_iDevice);
    INFO_MSG("JPEG Quality: " << g_uiQuality);

    if (!directoryExistsOrCreate(directory.c_str())) {
        ERROR_MSG("The directory " << directory << " could not be created!");
        return -1;
    }

    // Compress image parameter
    vector<uchar> buf;
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(g_uiQuality);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    rsb::Factory &factory = rsb::getFactory();

    // Create the informer
    Informer<std::string>::Ptr imageInformer = getFactory().createInformer<std::string> (Scope(g_sImageScope));
    Informer<std::string>::Ptr detectedInformer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

    // Create and start the command listener
    rsb::ListenerPtr listener = factory.createListener(g_sInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

    listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));
    ////////////////////////////////////////////////////////////////////////////////////////////////////



    // Creating the cam object
    cv::VideoCapture camera;
    // Open the device /dev/video<g_iDevice>
    if (camera.open(g_iDevice)) {
    
        // Take images and convert them to gray
        Mat result, result_cropped;
        Mat prev_frame, current_frame, next_frame;
        camera >> prev_frame;
        prev_frame.copyTo(result);
        camera >> current_frame;
        camera >> next_frame;
        cvtColor(current_frame, current_frame, CV_RGB2GRAY);
        cvtColor(prev_frame, prev_frame, CV_RGB2GRAY);
        cvtColor(next_frame, next_frame, CV_RGB2GRAY);
    
        // d1 and d2 for calculating the differences
        // result, the result of and operation, calculated on d1 and d2
        // number_of_changes, the amount of changes in the result matrix.
        // color, the color for drawing the rectangle when something has changed.
        Mat d1, d2, motion;
        int number_of_changes, number_of_sequence = 0;
        Scalar mean_, color(0,255,255); // yellow
    
        // Detect motion in window
        int x_start = 0, x_stop = current_frame.cols-1;
        int y_start = 0, y_stop = current_frame.rows-1;
    
        // Erode kernel
        Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));
    
        // All settings have been set, now go in endless loop and
        // take as many pictures you want..
        while (true) {
            // Take a new image
            current_frame.copyTo(prev_frame);
            next_frame.copyTo(current_frame);
            camera >> next_frame;
            next_frame.copyTo(result);
            cvtColor(next_frame, next_frame, CV_RGB2GRAY);

            // Calc differences between the images and do AND-operation
            // threshold image, low differences are ignored (ex. contrast change due to sunlight)
            absdiff(prev_frame, next_frame, d1);
            absdiff(next_frame, current_frame, d2);
            bitwise_and(d1, d2, motion);
            threshold(motion, motion, 35, 255, CV_THRESH_BINARY);
            erode(motion, motion, kernel_ero);

            Mat debugImage(result.rows, result.cols*2, CV_8UC3);
            result.copyTo(debugImage(Rect(0, 0, result.cols, result.rows)));
            if (trackMotion) {
                number_of_changes = detectMotion(motion, result, result_cropped, debugImage, x_start, x_stop, y_start, y_stop, max_deviation, color);
            }

            // Send the image
            if (sendingPic) {
                Mat flippedPic;
                if (debugging) {
                    flip(debugImage, flippedPic, 0);
//                    imencode(".jpg", debugImage, buf, compression_params);
                } else {
                    flip(result, flippedPic, 0);
//                    imencode(".jpg", result, buf, compression_params);
                }
                imencode(".jpg", flippedPic, buf, compression_params);
                shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
                imageInformer->publish(frameJpg);
            }

            if (trackMotion) {
                // If a lot of changes happened, we assume something changed.
                if (number_of_changes>=there_is_motion) {

                    if(number_of_sequence>0) {
                        if (!saveImg(result,directory,EXT,DIR_FORMAT.c_str(),FILE_FORMAT.c_str())) {
                            WARNING_MSG("Could not save image in directory " << directory << ".");
                        }
                        if (!saveImg(result_cropped,directory,EXT,DIR_FORMAT.c_str(),CROPPED_FILE_FORMAT.c_str())) {
                            WARNING_MSG("Could not save image in directory " << directory << " in subdirectory for cropped images.");
                        }
                        // Send detection signal
                        shared_ptr<std::string> StringPtr(new std::string("MotionDetected"));
                        detectedInformer->publish(StringPtr);
                    }
                    number_of_sequence++;
                } else {
                    number_of_sequence = 0;
                }
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
                    trackMotion = true;
                    INFO_MSG("Start motion detection.");
                // check for stop site command
                } else if (command == COMMAND_STOP) {
                    trackMotion = false;
                    INFO_MSG("Stop motion detection.");
                // otherwise it is an unknown command
                } else {
                    INFO_MSG("Unknown command.");
                }
            }
        }
    } else {
        ERROR_MSG("Could not open camera at /dev/video" << g_iDevice);
    }
    return 0;    
}

