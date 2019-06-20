//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Test project for edge detection.
//============================================================================


#include <iostream>
#include <fstream>

#include <time.h>
#include <dirent.h>
#include <sstream>
#include <fstream>
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

// For checking character pressed in the console
//#include <kbhit.hpp>

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

// view information
char* winnameSource = "Source";
char* winnameEdge = "Edges";
char* winnameLabel = "Labels";

// source name
string sourcePath = "pics/detect0.jpg";

Mat src, src_gray;
Mat dst, detected_edges, labeled;

int edgeThresh = 1;
int lowThreshold = 90;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;


// functions
vector<string> splitString(string text, char delimiter);

void CannyThreshold(int, void*) {
    // Reduce noise with a kernel 3x3
    blur(src_gray, detected_edges, Size(3,3));

    // Canny detector
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

    // Using Canny's output as a mask, we display our result
    dst = Scalar::all(0);

    src.copyTo(dst, detected_edges);
    imshow(winnameEdge, detected_edges);
}

void tableDetection() {
    vector<int> xPoses;
    vector<int> yPoses;

    labeled = Mat::zeros(src.rows, src.cols, CV_8UC3);
    Mat visited = Mat::zeros(src.rows, src.cols, CV_8UC1);

    int tcBlue = 210;
    int tcGreen = 230;
    int tcRed = 233;
/*    for (int x=0; x < src.cols; x++) {
        Vec3b color = src.at<Vec3b>(src.rows-1,x);
        tcBlue += color.val[0];
        tcGreen += color.val[1];
        tcRed += color.val[2];
    }
    tcBlue /= src.cols;
    tcGreen /= src.cols;
    tcRed /= src.cols;*/
    
    Vec3b tableColor = Vec3b(tcBlue, tcGreen, tcRed);
    std::cout << "Table Color: " << tcBlue << "/" << tcGreen << "/" << tcRed << std::endl;

    xPoses.push_back(0);
    yPoses.push_back(src.rows-1);
    visited.at<char>(src.rows-1,0) = 255;
    xPoses.push_back(src.cols-1);
    yPoses.push_back(src.rows-1);
    visited.at<char>(src.rows-1,src.cols-1) = 255;

    int posCounter = 0;
    int meanDiff = 40;
    while (posCounter < xPoses.size()) {
        int x = xPoses.at(posCounter);
        int y = yPoses.at(posCounter);
        posCounter++;

        Vec3b color = src.at<Vec3b>(y,x);
        int blue = color.val[0];
        int green = color.val[1];
        int red = color.val[2];
        bool blueOK = blue >= tableColor.val[0]-meanDiff && blue <= tableColor.val[0]+meanDiff;
        bool greenOK = green >= tableColor.val[1]-meanDiff && green <= tableColor.val[1]+meanDiff;
        bool redOK = red >= tableColor.val[2]-meanDiff && red <= tableColor.val[2]+meanDiff;

        if (blueOK, greenOK, redOK) {
            labeled.at<Vec3b>(y,x) = Vec3b(0,255,255);

            int xstart = x-1; int xend = x+1; int ystart = y-1; int yend = y+1;
            if (xstart < 0) xstart = x;
            if (ystart < 0) ystart = y;
            if (xend >= src.cols) xend = src.cols-1;
            if (yend >= src.rows) yend = src.rows-1;

            for (int xi=xstart; xi<=xend; xi++) {
                for (int yi=ystart; yi<=yend; yi++) {
                    if (visited.at<char>(yi,xi) == 0) {
                        visited.at<char>(yi,xi) = 255;
                        xPoses.push_back(xi);
                        yPoses.push_back(yi);
                    }
                }
            }
        }
    }

    imshow(winnameLabel, labeled);
}

// main function
int main (int argc, char * const argv[]) {
  
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("source,s", po::value<std::string>(&sourcePath),"Path to the source image.");

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


    // create windows
    cvNamedWindow(winnameSource,CV_WINDOW_NORMAL);
    cvNamedWindow(winnameEdge,CV_WINDOW_NORMAL);
    cvNamedWindow(winnameLabel,CV_WINDOW_NORMAL);

    // Load source
    src = imread(sourcePath);

    if(!src.data) {
        return -1;
    }

    // Show source
    imshow(winnameSource, src);

    tableDetection();

    // Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());

    // Convert the image to grayscale
    cvtColor(src, src_gray, CV_BGR2GRAY);

    // Create a Trackbar for user to enter threshold
    createTrackbar("Min Threshold:", winnameEdge, &lowThreshold, max_lowThreshold, CannyThreshold);

    // Show the image
    CannyThreshold(0, 0);

    // Wait until user exit program by pressing a key
    waitKey(0);

    cvDestroyAllWindows();
    return 0;

}


vector<string> splitString(string text, char delimiter) {
    vector<string> parts;
    std::stringstream textstream(text);
    string part;
    while (getline(textstream, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

