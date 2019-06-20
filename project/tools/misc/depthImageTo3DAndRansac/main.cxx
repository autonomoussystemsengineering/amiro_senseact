#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/iplImageConverter/IplImageConverter.h>

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

// RST
#include <rsb/converter/ProtocolBufferConverter.h>
// RST Proto types
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>

//#include <OniSampleUtilities.h>

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
using namespace rst::converters::opencv;

static std::string g_sOutScope = "/depthImage";

//TODO remove this
static std::string outScope = "/lidar";
//std::vector<float> fitPlane(std::vector<int>,boost::shared_ptr<IplImage>);
std::vector<float> fitPlane(std::vector<int> maybeInlier , std::vector< std::vector<float> > distances, int width , int height);
float calcDistanceToPlane( std::vector<float> point, std::vector<float> normal, std::vector<float> pointOnPlane);
float calcError(std::vector<float> betterModel,std::vector<int> maybeInlier,std::vector<int> alsoInlier, std::vector< std::vector<float> > distances);
std::vector< std::vector<float> > simpleDistanceFinder(DepthPixel* pDepth, int width, int height);
//void ransac(boost::shared_ptr<IplImage> sendIplImageCopy);
void ransac(std::vector< std::vector<float> > distances, int width , int height);
//std::vector<float> fitPlane2( std::vector<int> maybeInlier , boost::shared_ptr<IplImage> sendIplImageCopy);
std::vector<float> fitPlane2( std::vector<int> maybeInlier , std::vector< std::vector<float> > distances, int width, int height);
std::vector<std::vector<float> > convertDepthToXYZ(DepthPixel* pDepth,int height,int width);//convertDepthToXYZ(boost::shared_ptr<IplImage> img);

// Field of View ASUS XTION PRO
//58 H, 45 V, 70 D (Horizontal, Vertical, Diagonal)
//ORBBEC
//60 horiz x 49.5 vert. (73 diagonal)
float fovH = 60.0f;
float fovV = 49.5f;

/*
float FindLargestEntry(const Matrix33 &m);
Vector3 FindEigenVectorAssociatedWithLargestEigenValue(const Matrix33 &m);
void FindLLSQPlane(Vector3 *points,int count,Vector3 *destCenter,Vector3 *destNormal);*/
//TODO remove this

int main(int argc, char **argv) {

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images.");

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
    
    INFO_MSG( "Scope: " << g_sOutScope)

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Register our converter within the collection of converters for
    // the string wire-type (which is used for arrays of octets in
    // C++).
    boost::shared_ptr<IplImageConverter> converter(new IplImageConverter());
    converterRepository<std::string>()->registerConverter(converter);

    rsb::Factory &factory = rsb::getFactory();

    // Create the informer
    //Informer<rst::vision::Image>::Ptr informer = getFactory().createInformer<rst::vision::Image> (Scope(g_sOutScope));
    Informer<IplImage>::Ptr informer = getFactory().createInformer<IplImage> (Scope(g_sOutScope));
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
      printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
      return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
      printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
      return 2;
    }

    VideoStream depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
      rc = depth.create(device, SENSOR_DEPTH);
      if (rc != STATUS_OK)
      {
        printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        return 3;
      }
    }

    rc = depth.start();
    if (rc != STATUS_OK)
    {
      printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
      return 4;
    }

    VideoFrameRef frame;

  
  // Allocate a frame object to store the picture
  //  boost::shared_ptr<cv::Mat> frame(new cv::Mat);

  // not part of driver ////
  
  // Get the RSB factory
  //rsb::Factory& factory = rsb::getFactory();
    
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  // Create the informer
  rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim = factory.createInformer<rst::vision::LocatedLaserScan> (outScope);
  rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation(new rst::vision::LocatedLaserScan);

  // not part of driver ////
  
  while (true)
  {

    int changedStreamDummy;
    VideoStream* pStream = &depth;
    rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != STATUS_OK)
    {
      printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
      continue;
    }

    rc = depth.readFrame(&frame);
    if (rc != STATUS_OK)
    {
      printf("Read failed!\n%s\n", OpenNI::getExtendedError());
      continue;
    }

    if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
    {
      printf("Unexpected frame format\n");
      continue;
    }

    DepthPixel* pDepth = (DepthPixel*)frame.getData();

    int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2 + ((int) frame.getWidth())*8; //TODO middle index always return 0, problem with driver or camera
    printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);

    // Convert to an image which can be send via RSB
    boost::shared_ptr<cv::Mat> image(new cv::Mat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1, (void*)pDepth, cv::Mat::AUTO_STEP));

    boost::shared_ptr<IplImage> sendIplImage(new IplImage(*image));
    
    /* ////////////  not part of driver ///////////*/

    //std::vector<std::vector<float> > pointsXYZ = convertDepthToXYZ(sendIplImage);
    std::vector<std::vector<float> > pointsXYZ = convertDepthToXYZ(pDepth,(int) frame.getHeight(),(int) frame.getWidth());
    INFO_MSG( "Data: " << pointsXYZ.at(middleIndex) << " " << frame.getHeight() << " " << frame.getWidth() );
    std::vector< std::vector<float> > distances = simpleDistanceFinder(pDepth,(int) frame.getWidth(),(int) frame.getHeight());
    //ransac(sendIplImage);
    ///ransac(distances,(int) frame.getWidth(),(int) frame.getHeight());
    //////////send simulation data over rsb/////////////////////////
    
    float degreePerPixelH = fovH/((int) frame.getWidth());
    float degreePerPixelV = fovV/((int) frame.getHeight());

    const int scanSkip=1;
    //const double radPerStep = (360.0 /*°*/ / 1024.0 /*Steps*/) * (M_PI /*rad*/ / 180.0f /*°*/);
    const double radPerStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/);
    const double radPerSkipStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/) * scanSkip;
    const double startAngle =  -(fovH/2) * M_PI / 180.0f;
    const double endAngle =  (fovH/2) * M_PI / 180.0f;
    laserScanSimulation->set_scan_angle(fovH - radPerSkipStep);
    laserScanSimulation->set_scan_angle_start(startAngle); //- radPerSkipStep / 2.0f);
    laserScanSimulation->set_scan_angle_end(endAngle);//  + radPerSkipStep / 2.0f);
    laserScanSimulation->set_scan_values_min(0.001); 
    laserScanSimulation->set_scan_values_max(10.0);
    laserScanSimulation->set_scan_angle_increment(radPerSkipStep);

    // Reserve data
    for(int idx = 1; idx <= distances.size(); ++idx)
    {
      laserScanSimulation->mutable_scan_values()->Add(0.0f);
    }
    
    //while(1)
    for(int idx = 0; idx < distances.size(); idx++) 
    {
      // Convert the data from millimeter to meter
      laserScanSimulation->mutable_scan_values()->Set(idx, static_cast<float>( distances.at(idx).at(1) / 1000.0f));
      //INFO_MSG( "Laser: " <<  idx << "  ,  "<< static_cast<float>( distances.at(idx).at(1) / 1000.0f) );
      if(idx%7==0)
      {
	//INFO_MSG( "Laser: " << distances.at(idx).at(1) );
      }
    }
    // Send the data.
    informerLaserSim->publish(laserScanSimulation);

    //////////send simulation data over rsb/////////////////////////
    
    /* ////////////  not part of driver ///////////*/

    informer->publish(sendIplImage);
  }

  depth.stop();
  depth.destroy();
  device.close();
  OpenNI::shutdown();

  return 0;

}

//void ransac(boost::shared_ptr<IplImage> sendIplImageCopy)
void ransac(std::vector< std::vector<float> > distances, int width , int height)
{
  /* ////////// Process data  ///////// */
    //RANSAC
    //plane model
    std::vector<float> bestFitModel;
    //min. number of points for model
    int n = 3;
    // max iterations
    float ik = 0.5;
  
    //Mat(const IplImage* img, bool copyData=false);
    int cloudSize = width * height;
    int k = cloudSize * ik;
    //threshhold mmeters
    int t = 30;
    //min. number of inliers
    int d = 100;
    int i = 0; //iterations
    float bestErr = FLT_MAX_EXP;//std::numeric_limits.max_exponent;//FLT_MAX_EXP //large number

    // get data from image 
    //cv::Mat dMat = cv::Mat(cv::Size(sendIplImageCopy->height, sendIplImageCopy->width), CV_16UC1, (void*)sendIplImageCopy->imageData, cv::Mat::AUTO_STEP);
    
    while(i<k)
    { 
      //indexs
      std::vector<int> maybeInlier;
      std::vector<int> alsoInlier;
      std::vector<float> maybeFitModel;
      for(int j = 0; j< n;j++)
      {
        int index = rand() % cloudSize ;
        maybeInlier.push_back(index);
      }
      maybeFitModel = fitPlane(maybeInlier, distances, width , height);
      
      for(int x = 0; x< cloudSize; x++)
      {
        // if x index not in maybeInlier
        if(std::find(maybeInlier.begin(), maybeInlier.end(), x) == maybeInlier.end())
        {
          //find error
          
          // point on the plane
          std::vector<float>  pntOnPlane;
          //row
          //int a = maybeInlier.at(0) / width;
          //column
          //int b = maybeInlier.at(0) % width;
          pntOnPlane.push_back(distances.at(maybeInlier.at(0)).at(0));
          pntOnPlane.push_back(distances.at(maybeInlier.at(0)).at(1));
          pntOnPlane.push_back(distances.at(maybeInlier.at(0)).at(2));

          // normal of the plane
          std::vector<float> normal;
          normal.push_back(maybeFitModel.at(0));
          normal.push_back(maybeFitModel.at(1));
          normal.push_back(maybeFitModel.at(2));


          // point with index x
          std::vector<float>  pnt;
          //row
          //int c = x / width;
          //column
          //int d1 = x % width;
          pnt.push_back(distances.at(x).at(0));
          pnt.push_back(distances.at(x).at(1));
          pnt.push_back(distances.at(x).at(2));

          // calcDistanceToPlane(std::vector<float> point, std::vector<float> normal, std::vector<float> pointOnPlane)
          float dist = calcDistanceToPlane(pnt,normal,pntOnPlane);
          if(dist < t)
          {
            alsoInlier.push_back(x);
          }
        }
      }
      // if good model found
      if(alsoInlier.size() > d) 
      {
	std::vector<int> allInlier = alsoInlier;
	for(int ind = 0; ind< n;ind++)
	{
	  allInlier.push_back(maybeInlier.at(ind));
	}
        std::vector<float> betterModel = fitPlane2(allInlier,distances,width,height);
        float thisErr = calcError(betterModel,maybeInlier,alsoInlier,distances);
        if(thisErr < bestErr)
        {
          bestFitModel = betterModel;
          bestErr = thisErr;
        }
      }
      i++;
    }
    //result
    //return bestFitModel if not null;

/* ////////// Process data  ///////// */  

}

//std::vector<float> fitPlane(std::vector<int> maybeInlier, boost::shared_ptr<IplImage> img)
std::vector<float> fitPlane(std::vector<int> maybeInlier , std::vector< std::vector<float> > distances, int width , int height)
{
  //cv::Mat depthMat = cv::Mat(cv::Size(img->height, img->width), CV_16UC1, (void*)img->imageData, cv::Mat::AUTO_STEP);
  std::vector<std::vector<float> > points;
  for(int x=0; x<maybeInlier.size();x++)
  {
      //row
      //int i = maybeInlier.at(x) / width;
      //column
      //int j = (maybeInlier.at(x) % width) ;
      std::vector<float> point;
      point.push_back(distances.at(maybeInlier.at(x)).at(0));
      point.push_back(distances.at(maybeInlier.at(x)).at(1));
      point.push_back(distances.at(maybeInlier.at(x)).at(2));
      points.push_back(point);
  }
  std::vector<float> vector1;
  std::vector<float> vector2;
  //x1-x2
  vector1.push_back(points.at(1).at(0) - points.at(0).at(0));
  //y1-y2
  vector1.push_back(points.at(1).at(1) - points.at(0).at(1));
  //z1-z2
  vector1.push_back(points.at(1).at(2) - points.at(0).at(2));
  //x1-x2
  vector2.push_back(points.at(2).at(0) - points.at(0).at(0));
  //y1-y2
  vector2.push_back(points.at(2).at(1) - points.at(0).at(1));
  //z1-z2
  vector2.push_back(points.at(2).at(2) - points.at(0).at(2));
  //cross product of normal
  std::vector<float> normal;
  //a2b3-a3b2
  normal.push_back(vector1.at(1)*vector2.at(2) - vector1.at(2)*vector2.at(1));
  //a3b1-a1b3
  normal.push_back(vector1.at(2)*vector2.at(0) - vector1.at(0)*vector2.at(2));
  //a1b2-a2b1
  normal.push_back(vector1.at(0)*vector2.at(1) - vector1.at(1)*vector2.at(0));
  
  std::vector<float> maybeFitModel;
  maybeFitModel.push_back(normal.at(0));
  maybeFitModel.push_back(normal.at(1));
  maybeFitModel.push_back(normal.at(2));
  //d = ax+by+cz
  float dis = normal.at(0)*points.at(0).at(0) + normal.at(1)*points.at(0).at(1) + normal.at(2)*points.at(0).at(2) ;
  maybeFitModel.push_back(dis);
  return maybeFitModel;
}

//std::vector<float> fitPlane2( std::vector<int> maybeInlier , boost::shared_ptr<IplImage> sendIplImageCopy)
std::vector<float> fitPlane2( std::vector<int> maybeInlier , std::vector< std::vector<float> > distances, int width, int height)
{
  //using least squares to fit a plane
  
  //cv::Mat depthMat = cv::Mat(cv::Size(sendIplImageCopy->height, sendIplImageCopy->width), CV_16UC1, (void*)sendIplImageCopy->imageData, cv::Mat::AUTO_STEP);
  Eigen::Matrix<double,3,3> A;
  float m00= 0.0f;
  float m01= 0.0f;
  float m02= 0.0f;
  float m10= 0.0f;
  float m11= 0.0f;
  float m12= 0.0f;
  float m20= 0.0f;
  float m21= 0.0f;
  float m22= maybeInlier.size();

  Eigen::Matrix<double,3,1> b;
  //Eigen::VectorXf b(3);
  float b00= 0.0f;
  float b01= 0.0f;
  float b02= 0.0f;
  
  std::vector<float> pointSingle;
  for(int k=0; k<maybeInlier.size();k++)
  {
      //row
      int i = maybeInlier.at(k) / width;
      //column
      int j = (maybeInlier.at(k) % width) ;
      m00 = m00 + distances.at(maybeInlier.at(k)).at(0) * distances.at(maybeInlier.at(k)).at(0);//xi*xi
      m01 = m01 + distances.at(maybeInlier.at(k)).at(0) * distances.at(maybeInlier.at(k)).at(1);//xi*yi
      m02 = m02 + distances.at(maybeInlier.at(k)).at(0);//xi
      
      m10 = m10 + distances.at(maybeInlier.at(k)).at(0) * distances.at(maybeInlier.at(k)).at(1);//xi*yi
      m11 = m11 + distances.at(maybeInlier.at(k)).at(1) * distances.at(maybeInlier.at(k)).at(1);//yi*yi
      m12 = m12 + distances.at(maybeInlier.at(k)).at(1);//yi
      
      m20 = m20 + distances.at(maybeInlier.at(k)).at(0);//xi
      m21 = m21 + distances.at(maybeInlier.at(k)).at(1);//yi
      
      b00= b00 + distances.at(maybeInlier.at(k)).at(0) * distances.at(maybeInlier.at(k)).at(2);//xi*zi
      b01= b01 + distances.at(maybeInlier.at(k)).at(1) * distances.at(maybeInlier.at(k)).at(2);//yi*zi
      b02= b02 + distances.at(maybeInlier.at(k)).at(2);//zi
      
      //save one point to solve plane equation later
      if(k==0)
      {
	pointSingle.push_back(distances.at(maybeInlier.at(k)).at(0));
	pointSingle.push_back(distances.at(maybeInlier.at(k)).at(1));
	pointSingle.push_back(distances.at(maybeInlier.at(k)).at(2));
      }
  }
  A(0,0)= m00;
  A(0,1)= m01;
  A(0,2)= m02;
  A(1,0)= m10;
  A(1,1)= m11;
  A(1,2)= m12;
  A(2,0)= m20;
  A(2,1)= m21;
  A(2,2)= m22;

  b(0,0) = b00;
  b(1,0) = b01;
  b(2,0) = b02;
  Eigen::Matrix<double,3,1> x = (A.transpose() * A).ldlt().solve(A.transpose() * b);

  std::vector<float> maybeFitModel;
  maybeFitModel.push_back(x(0,0));
  maybeFitModel.push_back(x(0,1));
  maybeFitModel.push_back(x(0,2));
  //ax+by+cz=d
  float d = maybeFitModel.at(0)*pointSingle.at(0) + maybeFitModel.at(1)*pointSingle.at(1) + maybeFitModel.at(2)*pointSingle.at(2);
  maybeFitModel.push_back(d);
  return maybeFitModel;
}

// formula from http://www.xbdev.net/maths_of_3d/collision_detection/point_to_plane/index.php
float calcDistanceToPlane( std::vector<float> point, std::vector<float> normal, std::vector<float> pointOnPlane)
{
  std::vector<float> vec;
  vec.push_back( point.at(0) - pointOnPlane.at(0));
  vec.push_back( point.at(1) - pointOnPlane.at(1));
  vec.push_back( point.at(2) - pointOnPlane.at(2));
  //dot product with normal
  float dis = normal.at(0)*vec.at(0) + normal.at(1)*vec.at(1) + normal.at(2)*vec.at(2);
  return dis;
}

float calcError(std::vector<float> betterModel,std::vector<int> maybeInlier,std::vector<int> alsoInlier, std::vector< std::vector<float> > distances)
{
  // normal of the plane
  std::vector<float> normal;
  normal.push_back(betterModel.at(0));
  normal.push_back(betterModel.at(1));
  normal.push_back(betterModel.at(2));

  // an arbitary point on the plane
  std::vector<float> pntOnPlane;
  pntOnPlane.push_back(1.0f);
  pntOnPlane.push_back(1.0f);
  // -(ax+by-d)/c = z
  float z = -(betterModel.at(0)*1.0 + betterModel.at(1)*1.0 - betterModel.at(3))/betterModel.at(2);
  pntOnPlane.push_back(z);

  float error;
  for(int x=0; x<maybeInlier.size();x++)
  {
    // point on the plane
    std::vector<float>  pt;
    //row
    //int a = maybeInlier.at(x) / m.cols;
    //column
    //int b = maybeInlier.at(x) % m.cols;
    pt.push_back(distances.at(maybeInlier.at(x)).at(0));
    pt.push_back(distances.at(maybeInlier.at(x)).at(1));
    pt.push_back(distances.at(maybeInlier.at(x)).at(2));

    error = error + calcDistanceToPlane( pt,normal,pntOnPlane);
  }
  for(int x=0; x<alsoInlier.size();x++)
  {
   // point on the plane
    std::vector<float>  pt;
    //row
    //int a = alsoInlier.at(x) / m.cols;
    //column
    //int b = alsoInlier.at(x) % m.cols;
    pt.push_back(distances.at(alsoInlier.at(x)).at(0));
    pt.push_back(distances.at(alsoInlier.at(x)).at(1));
    pt.push_back(distances.at(alsoInlier.at(x)).at(2));

    error = error + calcDistanceToPlane( pt,normal,pntOnPlane);
  }
  return error; 
}

//convert Depth image to XYZ image
std::vector<std::vector<float> > convertDepthToXYZ(DepthPixel* pDepth,int height,int width)
{  
  //float focalX = img->width/(2 * tan(fovH/2)); //float focalY = img->height/(2 * tan(fovV/2));
  float focalX = width/(2.0f * tan(fovH/2.0f));
  float focalY = height/(2.0f * tan(fovV/2.0f));
  
  std::vector<std::vector<float> > points;
  for(int x=0; x < height*width ;x++) //for(int x=0; x < img->nSize ;x++)
  {
      //row
      int i = x / width;
      //column
      int j = x % width ;
      std::vector<float> point;
      
      //xworld = zworld * ximage/focalX
      float pointX = ((float) pDepth[x]) * (j - (width/2.0f) )/focalY; // TODO focalX, using here focalY instead of focalX as Y gives incorrect values ;
      //yworld = zworld * yimage/focalY
      float pointY = ((float) pDepth[x]) * (i - (height/2.0f) )/focalY;
      point.push_back(pointX);
      point.push_back(pointY);
      point.push_back( (float) pDepth[x]);  //point.push_back((float) depthMat.at<uchar>(i,j));
      points.push_back(point);
  }
  return points;
}


std::vector< std::vector<float> > simpleDistanceFinder(DepthPixel* pDepth, int width, int height)
{
  float degreePerPixelH = fovH/width;
  float degreePerPixelV = fovV/height;
  std::vector< std::vector<float> > distances;
  //iterate over the width of image
  for(int i=0; i < width ; i++)
  {
    //large initial value
    float minDist= 9999.0f;
    for(int j=0; j<height; j++)
    {
      float d = (float) pDepth[i + j*width];
      //TODO replace this if
      if(d != 0.0f)
      {
	if(d < minDist)
	{
	  minDist = d;
	}
      }
      
    } 
    //convert angle in terms of laser data (emerging from one point)
    int pixelFromCenter= i - (width/2.0f) ;
    float angle = pixelFromCenter*degreePerPixelH;
    std::vector<float> v;
    v.push_back(angle);
    v.push_back(minDist);
    distances.push_back(v);
  }  
  return distances;
}