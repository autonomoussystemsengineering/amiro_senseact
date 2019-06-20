//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Follows the guide based on the height of the guiding object.
//============================================================================

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
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

#include <rsb/Informer.h>
#include <rsb/Version.h>
#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>
#include <rsc/misc/SignalWaiter.h>

// For program options
#include <boost/program_options.hpp>

#include <ControllerAreaNetwork.h>
#include <actModels/lightModel.h>
#include <extspread/extspread.hpp>

#include <converter/vecIntConverter/main.hpp>
using namespace muroxConverter;

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#include <ctime>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>


#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace std;
using namespace boost::chrono;
using namespace cv;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;

// Init the CAN interface
ControllerAreaNetwork myCAN;

// scope names
std::string rgbScope = "/images/rgb";
std::string lightScope = "/amiro/lights";
std::string inScope = "/following";
std::string commandInit = "init";
std::string commandStop = "stop";

std::string port = "4823";
std::string host = "localhost";

static unsigned int g_uiQuality = 100;
bool sendImage = false;
bool justPrint = false;
bool debugImage = false;
bool followNow = false;
unsigned int sectorCount = 9;
uint16_t invalidRange = 350; // mm
uint16_t maxRange = 2000; // mm

float maxVelocityX = 0.5;
float minVelocityX = 0.1;
float maxAngVelDeg = 45.0;
float minAngVelDeg = 20.0;
float maxVelocityW = maxAngVelDeg * M_PI/180.0;
float minVelocityW = minAngVelDeg * M_PI/180.0;

vector<amiro::Color> curColors(8);

void motorAction(float speed, float turn) {
	myCAN.setTargetSpeed((int)(speed*1000000.0), (int)(turn*1000000.0));
}

void setLights(int lightType, vector<amiro::Color> colors, int period, rsb::Informer< std::vector<int> >::Ptr informer) {
	std::vector<int> lightCommand = setLights2Vec(lightType, colors, period);
	boost::shared_ptr<std::vector<int>> commandVector = boost::shared_ptr<std::vector<int> >(new std::vector<int>(lightCommand.begin(),lightCommand.end()));
	informer->publish(commandVector);
}

void calculateSectors(Mat depthMat, vector< vector<int> > &sectorResults, vector< vector<int> > sectors) {
	for (unsigned int s=0; s<sectors.size(); s++) {
		float pixelCount = float((sectors[s][1]-sectors[s][0]) * (sectors[s][3]-sectors[s][2]));

		int invalidCount = 0;
		int objectCount = 0;
		unsigned int valueSum = 0;
		for (int px=sectors[s][0]; px<sectors[s][1]; px++) {
			for (int py=sectors[s][2]; py<sectors[s][3]; py++) {
				if (depthMat.at<uint16_t>(py, px) < invalidRange) {
					invalidCount++;
				} else if (depthMat.at<uint16_t>(py, px) < maxRange) {
					objectCount++;
					valueSum += (unsigned int)(depthMat.at<uint16_t>(py, px));
				}
			}
		}
		if (float(objectCount)/pixelCount > 0.5) {
			sectorResults[s/sectorCount][s % sectorCount] = int(float(valueSum)/float(objectCount));
		} else {
			sectorResults[s/sectorCount][s % sectorCount] = 0;
		}
	}
}

void sectorFocus2Motor(vector< vector<int> > values, rsb::Informer< std::vector<int> >::Ptr informer) {
	int focusHeight = -1;
	bool heightFound = false;
	int leftSide, rightSide;
	for (int ys=0; ys<values.size(); ys++) {
		leftSide = -1;
		rightSide = -1;
		for (int xs=0; xs<values[ys].size(); xs++) {
			if (values[ys][xs] > 0) {
				focusHeight = ys;
				heightFound = true;
				if (leftSide < 0) {
					leftSide = xs;
					rightSide = xs;
				} else {
					rightSide = xs;
				}
			}
		}
		if (heightFound) break;
	}
	if (heightFound) {
		int center = values.size()/2;
		int centerWidth = int(float(center)/3.0);
		
		// Initialize velocity
		float velX = 0.0;
		float velW = 0.0;
		
		// Calculate forward velocity
		if (focusHeight > center+centerWidth) {
			float factor = float(focusHeight-(center+centerWidth)) / float(center-centerWidth);
			float velDiff = maxVelocityX-minVelocityX;
			velX = velDiff*factor + minVelocityX;
		} else if (focusHeight < center-centerWidth) {
			float factor = float(center-centerWidth-focusHeight)/float(center-centerWidth);
			float velDiff = maxVelocityX-minVelocityX;
			velX = -(velDiff*factor + minVelocityX);
		}
		
		// Calculate angular velocity
		float rightDist = float(rightSide - (center + centerWidth));
		float leftDist = float((center - centerWidth) - leftSide);
		if (leftDist > rightDist && leftDist > 0.0) {
			float distDiff = leftDist;
			if (rightDist > 0) distDiff -= rightDist;
			float factor = distDiff / float(center-centerWidth);
			float velDiff = maxVelocityW-minVelocityW;
			velW = velDiff*factor + minVelocityW;
		} else if (rightDist > leftDist && rightDist > 0.0) {
			float distDiff = rightDist;
			if (leftDist > 0) distDiff -= leftDist;
			float factor = distDiff / float(center-centerWidth);
			float velDiff = maxVelocityW-minVelocityW;
			velW = -(velDiff*factor + minVelocityW);
		}
		
		// set colors
		if (velX > 0) {
			curColors[7].setRedGreenBlue(255, 128, 0);
			curColors[0].setRedGreenBlue(255, 128, 0);
			curColors[3].setRedGreenBlue(255, 255, 0);
			curColors[4].setRedGreenBlue(255, 255, 0);
		} else if (velX < 0) {
			curColors[3].setRedGreenBlue(255, 128, 0);
			curColors[4].setRedGreenBlue(255, 128, 0);
			curColors[7].setRedGreenBlue(255, 255, 0);
			curColors[0].setRedGreenBlue(255, 255, 0);
		} else {
			curColors[3].setRedGreenBlue(255, 255, 0);
			curColors[4].setRedGreenBlue(255, 255, 0);
			curColors[7].setRedGreenBlue(255, 255, 0);
			curColors[0].setRedGreenBlue(255, 255, 0);
		}
		if (velW > 0) {
			curColors[5].setRedGreenBlue(255, 128, 0);
			curColors[6].setRedGreenBlue(255, 128, 0);
			curColors[1].setRedGreenBlue(255, 255, 0);
			curColors[2].setRedGreenBlue(255, 255, 0);
		} else if (velW < 0) {
			curColors[1].setRedGreenBlue(255, 128, 0);
			curColors[2].setRedGreenBlue(255, 128, 0);
			curColors[5].setRedGreenBlue(255, 255, 0);
			curColors[6].setRedGreenBlue(255, 255, 0);
		} else {
			curColors[1].setRedGreenBlue(255, 255, 0);
			curColors[2].setRedGreenBlue(255, 255, 0);
			curColors[5].setRedGreenBlue(255, 255, 0);
			curColors[6].setRedGreenBlue(255, 255, 0);
		}
		
		// set motor velocity
		motorAction(velX, velW);
	} else {
		motorAction(0.0, 0.0);
		for (int c=0; c<8; c++) {
			curColors[c].setRedGreenBlue(255, 255, 0);
		}
	}
	setLights(LightModel::LightType::SINGLE_SHINE, curColors, 0, informer);
}

int main(int argc, char **argv) {

	namespace po = boost::program_options;
	int rgbMode = 0;
	int depthMode = 0;
	unsigned int period = 10; // ms
	
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("commandScope", po::value < std::string > (&inScope),"Scope for receiving commands (default: /following).")
			("rbgScope", po::value < std::string > (&rgbScope),"Scope for sending debug image (default: /images/rgb).")
			("lightScope", po::value < std::string > (&lightScope),"Scope for sending light commands (default: /amiro/lights).")
			("host", po::value < std::string > (&host),"Host name of external spread (default: localhost).")
			("port", po::value < std::string > (&port),"Port of external spread (default: 4823).")
			("printTime,t","Prints the time of single parts.")
			("rgbMode", po::value<int>(&rgbMode), "Mode of RGB Image (default: 0).")
			("depthMode", po::value<int>(&depthMode), "Mode of Depth Image (default: 0).")
			("period", po::value<unsigned int>(&period), "Period between two image fetches in ms (default: 10).")
			("compression,c", po::value<unsigned int>(&g_uiQuality), "Compression value [0,100] (default: 100)")
			("sectorCount", po::value<unsigned int>(&sectorCount), "Count of sectors per side (default: 9).")
			("invalidRange,a", po::value<uint16_t>(&invalidRange), "Range until it is invalid in mm (default 350).")
			("maxRange,b", po::value<uint16_t>(&maxRange), "Maximal range in mm (default: 2000).")
			("maxVelX,c", po::value<float>(&maxVelocityX), "Maximal forward velocity in m/s (default: 0.5).")
			("minVelX,d", po::value<float>(&minVelocityX), "Minimal forward velocity in m/s (default: 0.1).")
			("maxVelW,e", po::value<float>(&maxAngVelDeg), "Maximal angular velocity in degrees/s (default: 45.0).")
			("minVelW,f", po::value<float>(&minAngVelDeg), "Minimal angular velocity in degrees/s (default: 20.0).")
			("sendImage,s", "Flag if the image shall be converted to OpenCV and send via RSB.")
			("startImmediatley,i", "Flag if the following procedure shall start immediately.")
			("printInfo,p", "Flag if just the camera information shall be printed. The tool closes afterwards.");

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
	
	sendImage = vm.count("sendImage");
	justPrint = vm.count("printInfo");
	followNow = vm.count("startImmediatley");
	
	
	if (rgbMode >= 0) INFO_MSG("Mode of RGB Image will be set to " << rgbMode);
	if (depthMode >= 0) INFO_MSG("Mode of Depth Image will be set to " << depthMode);
	
	if (sectorCount %2 == 0) sectorCount += 1;
	INFO_MSG("Sector count: " << sectorCount << " (center: " << (sectorCount/2+1) << ")");
	
	maxVelocityW = maxAngVelDeg * M_PI/180.0;
	minVelocityW = minAngVelDeg * M_PI/180.0;
	
	// Initialize clock
	system_clock::time_point systime1, systime2;
	milliseconds mstime;

	if (vm.count("printTime")) systime1 = system_clock::now();
	if (vm.count("printTime")) DEBUG_MSG("Starting RSB Initialization");

	// +++++ RSB Initialization +++++
	rsb::Factory &factory = rsb::getFactory();

	// Generate the programatik Spreadconfig for extern communication
	rsb::ParticipantConfig extspreadconfig = getextspreadconfig(factory, host, port);
	
	INFO_MSG("RSB Scopes:");
	INFO_MSG(" -> sending images:     " << rgbScope);
	INFO_MSG("");

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);
	
	// ------------ Listener ------------------------

	// prepare listener for input commands
	rsb::ListenerPtr commandListener = factory.createListener(inScope, extspreadconfig);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	commandListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));
	
	// ------------ Informer ------------------------
	
	// create informer for RGB images
	rsb::Informer<std::string>::Ptr RGBInformer = factory.createInformer<std::string> (rgbScope);
	
	// create informer for light commad
	rsb::Informer< vector<int> >::Ptr LightInformer = factory.createInformer< vector<int> > (lightScope);
	
	

	if (vm.count("printTime")) systime2 = system_clock::now();
	if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
	if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

	if (vm.count("printTime")) systime1 = system_clock::now();
	if (vm.count("printTime")) DEBUG_MSG("Starting OpenNI Initialization");
	// +++++ OpenNI Initialization +++++
	VideoFrameRef rgbImage, depthImage;
	VideoStream rgbStream, depthStream;
	Device device;
	
	Mat rgbMat, depthMat;

	// Compress data
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(g_uiQuality);
	vector<uchar> bufRgb, bufDepth;

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
	const SensorInfo* rgbinfo = device.getSensorInfo(SENSOR_COLOR);
	if (rgbinfo != NULL) {
		respON = rgbStream.create(device, SENSOR_COLOR);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not create RGB stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		const openni::Array<VideoMode>& modes = rgbinfo->getSupportedVideoModes();
		if (justPrint) {
			INFO_MSG("Modes of RGB stream:");
			for (int i=0; i<modes.getSize(); i++) {
				INFO_MSG(" " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat() << " format");
			}
		}
		if (rgbMode < modes.getSize() && rgbMode >= 0) {
			respON = rgbStream.setVideoMode(modes[rgbMode]);
			if (respON != STATUS_OK) {
				ERROR_MSG("Could not set mode of RGB stream: " << OpenNI::getExtendedError());
				return EXIT_FAILURE;
			}
		}
	}
	
	// get depth stream
	const SensorInfo* depthinfo = device.getSensorInfo(SENSOR_DEPTH);
	if (depthinfo != NULL) {
		respON = depthStream.create(device, SENSOR_DEPTH);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not create depth stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		const openni::Array<VideoMode>& modes = depthinfo->getSupportedVideoModes();
		if (justPrint) {
			INFO_MSG("Modes of depth stream:");
			for (int i=0; i<modes.getSize(); i++) {
				INFO_MSG(" " << i << ": " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << ", " << modes[i].getFps() << " fps, " << modes[i].getPixelFormat() << " format");
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
		ERROR_MSG("Could not start the RGB stream: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	respON = depthStream.start();
	if (respON != STATUS_OK) {
		ERROR_MSG("Could not start the depth stream: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	VideoStream* rgbStreamTemp = &rgbStream;
	int changedRgbStreamDummy;
	respON = OpenNI::waitForAnyStream(&rgbStreamTemp, 1, &changedRgbStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	if (respON != STATUS_OK) {
		ERROR_MSG("Wait for RGB stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	VideoStream* depthStreamTemp = &depthStream;
	int changedDepthStreamDummy;
	respON = OpenNI::waitForAnyStream(&depthStreamTemp, 1, &changedDepthStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	if (respON != STATUS_OK) {
		ERROR_MSG("Wait for depth stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}
	
	// read first image
	respON = rgbStream.readFrame(&rgbImage);
	if (respON != STATUS_OK) {
		ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
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
		int imageWidth = rgbImage.getWidth();
		int scanHeight = depthImage.getHeight();
		int scanWidth = depthImage.getWidth();
		INFO_MSG("Image size " << imageWidth << "x" << imageHeight << " with laser image " << scanWidth << "x" << scanHeight);
	}
	
	// define sectors
	float sectorWidth = float(depthImage.getWidth())/float(sectorCount);
	float sectorHeight = float(depthImage.getHeight())/float(sectorCount);
	vector< vector<int> > sectors;
	for (unsigned int sy=0; sy<sectorCount; sy++) {
		for (unsigned int sx=0; sx<sectorCount; sx++) {
			vector<int> sectorEdges(4);
			sectorEdges[0] = int(float(sx)*sectorWidth);
			sectorEdges[1] = int(float(sx+1)*sectorWidth);
			sectorEdges[2] = int(float(sy)*sectorHeight);
			sectorEdges[3] = int(float(sy+1)*sectorHeight);
			sectors.push_back(sectorEdges);
		}
	}
	Mat sectorialImage(depthImage.getHeight(), depthImage.getWidth(), CV_8UC3);
	Mat sectorMat[sectors.size()];
	for (unsigned int s=0; s<sectors.size(); s++) {
		sectorMat[s] = Mat(sectorialImage, Rect(sectors[s][0], sectors[s][2], sectors[s][1]-sectors[s][0], sectors[s][3]-sectors[s][2]));
	}
	vector< vector<int> > sectorResults;
	for (int s=0; s<sectorCount; s++) {
		vector<int> sectorLine(sectorCount);
		for (int l=0; l<sectorCount; l++) {
			sectorLine[l] = 0;
		}
		sectorResults.push_back(sectorLine);
	}
	
	for (int c=0; c<8; c++) {
		curColors[c] = amiro::Color(255, 255, 0);
	}
	setLights(LightModel::LightType::SINGLE_SHINE, curColors, 0, LightInformer);

	double minVal, maxVal;
	float alpha, beta;
	while (!justPrint) {
		
		if (!commandQueue->empty()) {
			std::string command (*commandQueue->pop());
			if (command.compare(commandInit) == 0) {
				followNow = true;
				INFO_MSG("Starting following.");
			} else if (command.compare(commandStop) == 0) {
				followNow = false;
				INFO_MSG("Stopping following.");
				motorAction(0.0,0.0);
			}
		}
		
		if (followNow) {
			if (vm.count("printTime")) DEBUG_MSG("-----------------------------------------------")
			systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Reading camera frames");
			respON = rgbStream.readFrame(&rgbImage);
			if (respON != STATUS_OK) {
				ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
				return EXIT_FAILURE;
			}
			respON = depthStream.readFrame(&depthImage);
			if (respON != STATUS_OK) {
				ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
				return EXIT_FAILURE;
			}
			if (vm.count("printTime")) systime2 = system_clock::now();
			if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

			if (!sendImage) {
				
				const openni::DepthPixel* depthBuffer = (const openni::DepthPixel*)depthImage.getData();
				Mat depthdata(depthImage.getHeight(), depthImage.getWidth(), CV_16UC1);
				memcpy(depthdata.data, depthBuffer, depthImage.getHeight()*depthImage.getWidth()*sizeof(uint16_t));
				flip(depthdata, depthdata, 1);
				
				calculateSectors(depthdata, sectorResults, sectors);
				sectorFocus2Motor(sectorResults, LightInformer);
				
			} else {
				// Convert to cv::Mat
				if (vm.count("printTime")) systime1 = system_clock::now();
				if (vm.count("printTime")) DEBUG_MSG("Conversion: Transform OpenNI to OpenCV");
				
				const openni::RGB888Pixel* rgbBuffer = (const openni::RGB888Pixel*)rgbImage.getData();
				rgbMat.create(rgbImage.getHeight(), rgbImage.getWidth(), CV_8UC3);
				memcpy(rgbMat.data, rgbBuffer, 3*rgbImage.getHeight()*rgbImage.getWidth()*sizeof(uint8_t));
				
				const openni::DepthPixel* depthBuffer = (const openni::DepthPixel*)depthImage.getData();
				Mat depthdata(depthImage.getHeight(), depthImage.getWidth(), CV_16UC1);
				memcpy(depthdata.data, depthBuffer, depthImage.getHeight()*depthImage.getWidth()*sizeof(uint16_t));
							
				if (vm.count("printTime")) systime2 = system_clock::now();
				if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
				if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
				
				if (vm.count("printTime")) systime1 = system_clock::now();
				if (vm.count("printTime")) DEBUG_MSG("Conversion: Convert to correctly flipped RGB images");
				
				cvtColor(rgbMat, rgbMat, CV_BGR2RGB);
				flip(rgbMat, rgbMat, 1);
				
				flip(depthdata, depthdata, 1);
				Mat depthDataImage1C;
				minMaxLoc(depthdata, &minVal, &maxVal);
				alpha = 255.0 / maxVal;
				beta = 255.0 - maxVal*alpha;
				depthdata.convertTo(depthDataImage1C, CV_8UC1, alpha, beta);
				cvtColor(depthDataImage1C, depthMat, CV_GRAY2RGB);
				
				if (vm.count("printTime")) systime2 = system_clock::now();
				if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
				if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
				
				if (vm.count("printTime")) systime1 = system_clock::now();
				if (vm.count("printTime")) DEBUG_MSG("Sectorial Calculation");
				
				calculateSectors(depthdata, sectorResults, sectors);
				sectorFocus2Motor(sectorResults, LightInformer);
				for (int s=0; s<sectors.size(); s++) {
					int value = int(float(sectorResults[s/sectorCount][s % sectorCount]) * 255.0/maxVal);
					sectorMat[s] *= 0.0;
					sectorMat[s] += cv::Scalar(value, value, value);
				}
				
				// draw sectors
	/*			cv::Scalar sectorColor(0, 255, 255);
				Point ul(0, 0);
				Point ur(depthMat.cols-1, 0);
				Point dl(0, depthMat.rows-1);
				Point dr(depthMat.cols-1, depthMat.rows-1);
				cv::line(depthMat, ul, ur, sectorColor);
				cv::line(depthMat, ur, dr, sectorColor);
				cv::line(depthMat, dr, dl, sectorColor);
				cv::line(depthMat, dl, ul, sectorColor);
				cv::line(sectorialImage, ul, ur, sectorColor);
				cv::line(sectorialImage, ur, dr, sectorColor);
				cv::line(sectorialImage, dr, dl, sectorColor);
				cv::line(sectorialImage, dl, ul, sectorColor);
				for (int s=0; s<sectors.size(); s++) {
					Point ul(sectors[s][0], sectors[s][2]);
					Point ur(sectors[s][1]-1, sectors[s][2]);
					Point dl(sectors[s][0], sectors[s][3]-1);
					cv::line(depthMat, ul, ur, sectorColor);
					cv::line(depthMat, dl, ul, sectorColor);
					cv::line(sectorialImage, ul, ur, sectorColor);
					cv::line(sectorialImage, dl, ul, sectorColor);
				}*/
				
				if (vm.count("printTime")) systime2 = system_clock::now();
				if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
				if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

				// Compress image
				if (vm.count("printTime")) systime1 = system_clock::now();
				if (vm.count("printTime")) DEBUG_MSG("Compressing and sending images");
				Mat publicMat(max(rgbMat.rows, depthMat.rows), rgbMat.cols+2*depthMat.cols, CV_8UC3);
				Mat part[3];
				part[0] = Mat(publicMat, Rect(0, 0, rgbMat.cols, rgbMat.rows));
				part[1] = Mat(publicMat, Rect(rgbMat.cols, 0, depthMat.cols, depthMat.rows));
				part[2] = Mat(publicMat, Rect(rgbMat.cols+depthMat.cols, 0, depthMat.cols, depthMat.rows));
				rgbMat.copyTo(part[0]);
				depthMat.copyTo(part[1]);
				sectorialImage.copyTo(part[2]);
				imencode(".jpg", publicMat, bufRgb, compression_params);
				boost::shared_ptr< std::string > rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
				RGBInformer->publish(rgbFrameJpg);
			}
			systime2 = system_clock::now();
			mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
		}

		usleep(period*1000);
	}
	
	rgbStream.stop();
	rgbStream.destroy();
	depthStream.stop();
	depthStream.destroy();
	
	device.close();
	OpenNI::shutdown();

	return EXIT_SUCCESS;
}
