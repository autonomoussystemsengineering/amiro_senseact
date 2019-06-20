/*
 * Takes one image out of NI and calculates the closest distance for every column, to simulate a laserscan over the whole image
 * Note: It works because of assumption/fact that the ground plane can not be seen directly from the height of AMiRo in Orbbec Camera
 * To start the program run:  sudo ./<name> -d 1000
 * spread should be running for RSB
 * publishes data at /lidar
 * argument d is delay in msecond at which lidar is published
 *Author: ssharma
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

// RST
#include <rsb/converter/ProtocolBufferConverter.h>
// RST Proto types
// #include <types/LocatedLaserScan.pb.h>
// #include <rst/geometry/Pose.pb.h>
#include <rst/vision/LaserScan.pb.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

//using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
// using namespace rst::converters::opencv;

class ReadData: public rsc::threading::PeriodicTask {
public:

    ReadData(rsb::Informer<rst::vision::LaserScan>::DataPtr laserScanSimulation,
             rsb::Informer<rst::vision::LaserScan>::Ptr informerLaserSim,
             bool justCenter,
             const unsigned int& ms = 1000) :
             rsc::threading::PeriodicTask(ms) {

      ///////////////////////////////////////////
      // Configuration
      ///////////////////////////////////////////
      this->laserScanSimulation = laserScanSimulation ; this->informerLaserSim = informerLaserSim;

      this->justCenter = justCenter;

      //initialize depthstream
      rc = OpenNI::initialize();
      if (rc != STATUS_OK)
      {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      rc = device.open(ANY_DEVICE);
      if (rc != STATUS_OK)
      {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
      {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
          printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
          isValid = false;
        }
      }

      rc = depth.start();
      if (rc != STATUS_OK)
      {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      VideoStream* pStreamTemp = &depth;
      int changedStreamDummy;
      rc = OpenNI::waitForAnyStream(&pStreamTemp, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

      if (rc != STATUS_OK)
      {
        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        isValid = false;
      }

      rc = depth.readFrame(&frame);
      if (rc != STATUS_OK)
      {
        printf("Read failed!\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      if (isValid) {
        this->frameWidth = (int) frame.getWidth();
        this->frameHeight = (int) frame.getHeight();
        this->fovH = depth.getHorizontalFieldOfView()*(180/M_PI);
        this->fovV = depth.getVerticalFieldOfView()*(180/M_PI);

        INFO_MSG("Width: " << this->frameWidth << " px")
        INFO_MSG("Height: " << this->frameHeight << " px")
        INFO_MSG("FOV Horizontal: " << this->fovH << " deg")
        INFO_MSG("FOV Vertical: " << this->fovV << " deg")

        this->degreePerPixelH = this->fovH/((float) this->frameWidth);
        this->degreePerPixelV = this->fovV/((float) this->frameHeight);
        // const float ORBBEC_ASTRA_S_MIN_RANGE = 0.2; // meter
        // const float ORBBEC_ASTRA_S_MAX_RANGE = 5.8; // meter

        const int scanSkip=1;
        //const double radPerStep = (360.0 /*°*/ / 1024.0 /*Steps*/) * (M_PI /*rad*/ / 180.0f /*°*/);
        // const double radPerStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/);
        const double radPerSkipStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/) * scanSkip;
        // const double startAngle =  -(this->fovH/2) * M_PI / 180.0f;
        // const double endAngle =  (this->fovH/2) * M_PI / 180.0f;
        this->laserScanSimulation->set_scan_angle(this->fovH - radPerSkipStep);
        // this->laserScanSimulation->set_scan_angle_start(startAngle); //- radPerSkipStep / 2.0f);
        // this->laserScanSimulation->set_scan_angle_end(endAngle);//  + radPerSkipStep / 2.0f);
        // this->laserScanSimulation->set_scan_values_min(ORBBEC_ASTRA_S_MIN_RANGE);
        // this->laserScanSimulation->set_scan_values_max(ORBBEC_ASTRA_S_MAX_RANGE);
        // this->laserScanSimulation->set_scan_angle_increment(radPerSkipStep);

        // Reserve data
        this->laserScanSimulation->mutable_scan_values()->Reserve(this->frameWidth);
        for(std::size_t idx = 1; idx <= this->frameWidth; ++idx) {
          this->laserScanSimulation->mutable_scan_values()->Add(0.0f);
        }
      }

  }

    virtual ~ReadData() {
      depth.stop();
      depth.destroy();
      device.close();
      OpenNI::shutdown();
    }

    virtual void execute() {

      if (this->isValid) {
        this->depth.readFrame(&frame);
        pDepth = (DepthPixel*)frame.getData();
        // Get the laser scan
        simpleDistanceFinder();

        // Send the data.
        informerLaserSim->publish(this->laserScanSimulation);
        //this->laserScanSimulation->clear_scan_values();
      } else {
        ERROR_MSG("No valid configuration")
      }
    }

    void simpleDistanceFinder() {

      // iterate over the width of image
      for(std::size_t i=0; i < this->frameWidth ; i++)
      {
        //large initial value
        float minDist= 99999.0f;

        if (justCenter) {
          minDist = (float) this->pDepth[this->frameHeight/2 * this->frameWidth + i];
          if (minDist == 0.0f) {
            minDist = 8.0;
          }
        } else {
          for(std::size_t j=0; j < this->frameHeight; j++) {
            float d = (float) this->pDepth[i + j*this->frameWidth];
            //TODO replace this if
            if(d != 0.0f) {
              if(d < minDist) {
                minDist = d;
              }
            }
          }
        }
        // Convert from mm to m and store the value
        this->laserScanSimulation->mutable_scan_values()->Set(i, float(minDist) / 1000.0f);

//        // Debug output
        //INFO_MSG( "Laser: " <<  i << "  ,  "<< this->laserScanSimulation->mutable_scan_values()->Get(i) );
//        if(i%10==0) {
//          //convert angle in terms of laser data (emerging from one point)
//          int pixelFromCenter= i - (width/2.0f) ;
//          float angle = pixelFromCenter*this->degreePerPixelH;
//          INFO_MSG( "Laser: " << angle <<  " : "<< minDist  << " : " << width << " : " <<height << " " << this->fovH );
//        }
      }
    }


private:
    rsb::Informer<rst::vision::LaserScan>::Ptr informerLaserSim;
    rsb::Informer<rst::vision::LaserScan>::DataPtr laserScanSimulation;

    DepthPixel* pDepth = NULL;
    VideoFrameRef frame;
    Status rc;
    VideoStream depth;
    Device device;
    //default values, actual values come from constructor
    // Field of View ASUS XTION PRO
    //58 H, 45 V, 70 D (Horizontal, Vertical, Diagonal)
    //ORBBEC
    //60 horiz x 49.5 vert. (73 diagonal)
    float fovH = 60.0f;  // degree
    float fovV = 49.5f;  // degree
    float degreePerPixelH = 0;  // degree
    float degreePerPixelV = 0;  // degree
    std::size_t frameWidth = 320;  // px
    std::size_t frameHeight = 240;  // px
    bool isValid = true;
    bool justCenter = false;
};


int main(int argc, char **argv) {

    namespace po = boost::program_options;
    std::string outScope = "/lidar";
    std::size_t intervalMs = 1000;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&outScope),"Scope for sending lidar data")
            ("lidarPublishDelay,d", po::value < std::size_t > (&intervalMs),"Period for publishing laser data in ms")
            ("justCenter,c", "Flag, if not the minimum, but just the vertical centered value shall be used as laser scan.");

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

    rsb::Factory &factory = rsb::getFactory();

    // Register
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan >());
    rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
    // Create the informer
    rsb::Informer<rst::vision::LaserScan>::Ptr informerLaserSim = factory.createInformer<rst::vision::LaserScan> (outScope);
    rsb::Informer<rst::vision::LaserScan>::DataPtr laserScanSimulation(new rst::vision::LaserScan);

    rsc::threading::ThreadedTaskExecutor exec;
    exec.schedule( rsc::threading::TaskPtr( new ReadData( laserScanSimulation, informerLaserSim, vm.count("justCenter"), intervalMs ) ) );

    rsc::misc::initSignalWaiter();
    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

}
