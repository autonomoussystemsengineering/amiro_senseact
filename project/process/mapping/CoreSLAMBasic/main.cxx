
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
 #define ERROR_MSG_
#include <MSG.h>

#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include "CoreSLAM.h"
#ifdef __cplusplus
}
#endif

// Parameters needed for Marcov sampling
static double sigma_xy_ = 0.01;  // m
static double sigma_theta_ = 0.05;  // rad
static int samples = 100; // Number of resampling steps
// parameters for coreslam
static double hole_width_ = 0.1;  // m
static double delta_ = 0.02;  // Meter per pixel
static ts_map_t ts_map_;
static ts_state_t state_;
// static ts_position_t position_;
// static ts_position_t first_odom_;
static ts_position_t prev_odom_;
static ts_position_t pose;
static ts_position_t odom_pose;
static bool gotFirstOdometry = false;
static ts_laser_parameters_t lparams_;
#define TS_POSE_HISTORY_SIZE    1000
static ts_position_t ts_poseHistory[TS_POSE_HISTORY_SIZE];
static size_t ts_poseHistorySize = 0;
static bool got_first_scan_ = false;
//static bool got_map_ = false;
static int laser_count_ = 0;
static int throttle_scans_ = 1;
// Check "http://de.wikipedia.org/wiki/Sinussatz"!
// c ist the first ray, b the second. If beta is 90°, it means that c is hitting a surface very perpendiular.
// Every deviation of the 90° is an incident, which means that the surface is not perpendicular to the ray.
// The value rayPruningAngleDegree gives the maximal allowed deviation from 90 degrees.
// Every deviation above that angle results in a pruning of the ray c
static float rayPruningAngleDegree = 60; /* [0 .. 90] */
float rayPruningAngle(){return asin((90 - rayPruningAngleDegree) / 180 * M_PI);}
// Converting helpers
#include <Eigen/Geometry>

static double transX, transY, transZ;
static double rotX, rotY, rotZ;

inline Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// Convinience
static bool sendMapAsCompressedImage = false;


// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <converter/matConverter/matConverter.hpp>

// RST Proto types
//#include <rst0.11/stable/rst/vision/LaserScan.pb.h>
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize

// Include CAN funtionality for odometry
#include <ControllerAreaNetwork.h>
// Declare the CAN interface
ControllerAreaNetwork *CAN;

// For sending the localization
rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

#include "pathPlanner.hpp"

// object that calculates paths
PathPlanner *pathPlanner;

#include <Constants.h>
#include <utils.h>

using namespace amiro::constants;
using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace muroxConverter; // The namespace for the own converters

#include <mutex>          // std::mutex
std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;
static double pathObstacleErosion = 0.1; // m
static double detectionObstacleErosion = 0.1; // m

rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListPublish(new twbTracking::proto::Pose2DList);
rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListPushPublish(new twbTracking::proto::Pose2DList);
rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListObjectsPublish(new twbTracking::proto::Pose2DList);

// Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
void mapErosion(int erosion_size, cv::Mat &map) {
  cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             cv::Point( erosion_size, erosion_size ) );
  cv::erode( map, map, element );
}

cv::Mat getObstacleMap(bool colored = false, bool withUnknown = false) {
  // Convert the map to a cv::Mat image
  const cv::Size size(TS_MAP_SIZE,TS_MAP_SIZE);
  cv::Mat map(size, CV_8UC1); // destination image for sending
  cv::Mat tmp = cv::Mat(TS_MAP_SIZE, TS_MAP_SIZE, CV_16UC1, static_cast<void*>(&ts_map_.map[0]));
  tmp.convertTo(map, CV_8UC1, 0.00390625);  // Convert to 8bit depth image
  if (!withUnknown) {
    for (ssize_t idx = 0; idx < map.rows * map.cols; ++idx)
      if (map.at<uint8_t>(idx) > 127) // Delete all unsure guesses
        map.at<uint8_t>(idx) = 0xFF;
  }

  if (colored) {
    cv::Mat mapColor(size, CV_8UC3); // Color image
    cv::cvtColor(map, mapColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image
    return mapColor;
  } else {
    return map;
  }
}

void changeObstacleMap(cv::Mat map) {
  // TODO Save given map into ts_map_.map!
}

// callBack for path to point
class pathCallback: public rsb::patterns::LocalServer::Callback<twbTracking::proto::Pose2D, twbTracking::proto::Pose2DList> {
  boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
      boost::shared_ptr<twbTracking::proto::Pose2D> pose) {

    INFO_MSG("Path Callback received.");
/*
    INFO_MSG("Path to " << pose.get()->x() << "/" << pose.get()->y() << " requested.");
    // generate the obstacle map
    cv::Mat obstacleMap(getObstacleMap());
    const int erosion_size = pathObstacleErosion / delta_;
    mapErosion(erosion_size, obstacleMap);
    int xSize = obstacleMap.size().width*delta_;
    int ySize = obstacleMap.size().height*delta_;

    // convert pose to cv::point2f
    // note: 3. coordinate is ignored
    Point2f target(pose.get()->x()+xSize/2, pose.get()->y()+ySize/2);

    // convert robot position to cv::point3f
    mtxOdom.lock();
    Point3f robotPose(odomData->mutable_translation()->x(), odomData->mutable_translation()->y(), odomData->mutable_rotation()->yaw());
    mtxOdom.unlock();

    // calculate a path
    // Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
    INFO_MSG("Starting path calculation.");
    std::vector<cv::Point2f> path = pathPlanner->getPathToTarget(obstacleMap, robotPose, target);

    // convert that path to a pose2DList
    INFO_MSG("Send path.");
    rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
    for (Point2f p : path) {
      twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
      pose2D->set_x(p.x-xSize/2);
      pose2D->set_y(p.y-ySize/2);
      pose2D->set_orientation(0);
      pose2D->set_id(0);
    }
    return pose2DList;*/

    return pose2DListPublish;
  }
};


void pathRequestFunction(twbTracking::proto::Pose2D pose) {
    INFO_MSG("Path to " << pose.x() << "/" << pose.y() << " requested.");
    // generate the obstacle map
    cv::Mat obstacleMap(getObstacleMap());
    //const int erosion_size = pathObstacleErosion / delta_;
    //mapErosion(erosion_size, obstacleMap);
    float xSize = ((float)obstacleMap.size().width) * delta_;
    float ySize = ((float)obstacleMap.size().height) * delta_;

    // convert pose to cv::point2f
    // note: 3. coordinate is ignored
    Point2f target(pose.x()+xSize/2.0, pose.y()+ySize/2.0);

    // convert robot position to cv::point3f
    mtxOdom.lock();
    Point3f robotPose(odomData->mutable_translation()->x(), odomData->mutable_translation()->y(), odomData->mutable_rotation()->yaw());
    mtxOdom.unlock();
    robotPose.x = robotPose.x + xSize/2.0;
    robotPose.y = robotPose.y + ySize/2.0;

    // calculate a path
    // Erode the map by with an eliptic pattern of pixel radius erosion_size = <size in meter> / delta_;
    INFO_MSG("Starting path calculation.");
    std::vector<cv::Point2f> path = pathPlanner->getPathToTarget(obstacleMap, robotPose, target);

    // convert that path to a pose2DList
    INFO_MSG("Send path.");
    //rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DListPublish(new twbTracking::proto::Pose2DList);
    pose2DListPublish->clear_pose();
    for (Point2f p : path) {
      twbTracking::proto::Pose2D *pose2D = pose2DListPublish->add_pose();
      pose2D->set_x(p.x - xSize/2.0);
      pose2D->set_y(p.y - ySize/2.0);
      pose2D->set_orientation(0);
      pose2D->set_id(0);
    }
}

// callBack for path to point
class pushingPathCallback: public rsb::patterns::LocalServer::Callback<twbTracking::proto::Pose2DList, twbTracking::proto::Pose2DList> {
  boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/,
      boost::shared_ptr<twbTracking::proto::Pose2DList> inputPointList) {
/*    // convert pose to cv::point2f
    Point3f startPos(inputPointList->pose(0).x(), inputPointList->pose(0).y(), 0);
    Point2f target(inputPointList->pose(1).x(), inputPointList->pose(1).y());

    // generate the obstacle map
    cv::Mat obstacleMap(getObstacleMap());

    // calculate a path
    std::vector<cv::Point2f> path = pathPlanner.getPathToTarget(obstacleMap, startPos, target);

    // convert that path to a pose2DList
    rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
    for (Point2f p : path) {
      twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
      pose2D->set_x(p.x);
      pose2D->set_y(p.y);
      pose2D->set_orientation(0);
      pose2D->set_id(0);
    }*/
    return pose2DListPushPublish;
  }
};

// callBack for path to point
void pushingPathRequestFunction(twbTracking::proto::Pose2DList inputPointList) {

  // generate the obstacle map
  cv::Mat obstacleMap(getObstacleMap());
  float xSize = ((float)obstacleMap.size().width) * delta_;
  float ySize = ((float)obstacleMap.size().height) * delta_;

  // convert pose to cv::point2f
  Point3f startPos(inputPointList.pose(0).x()+xSize/2.0, inputPointList.pose(0).y()+ySize/2.0, 0);
  Point2f target(inputPointList.pose(1).x()+xSize/2.0, inputPointList.pose(1).y()+ySize/2.0);

  // calculate a path
  std::vector<cv::Point2f> path = pathPlanner->getPathToTarget(obstacleMap, startPos, target);

  // convert that path to a pose2DList
  pose2DListPushPublish->clear_pose();
  for (Point2f p : path) {
    twbTracking::proto::Pose2D *pose2D = pose2DListPushPublish->add_pose();
    pose2D->set_x(p.x-xSize/2.0);
    pose2D->set_y(p.y-ySize/2.0);
    pose2D->set_orientation(0);
    pose2D->set_id(0);
  }
};

// callBack for path to point
class objectsCallback: public rsb::patterns::LocalServer::Callback<bool, twbTracking::proto::Pose2DList> {
  boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string& /*methodName*/, boost::shared_ptr<bool> draw_debug) {

    if (*draw_debug) {
      // Get the obstacles
      cv::Mat map = getObstacleMap();

      // Expand them
      const int erosion_size = detectionObstacleErosion / delta_;
      mapErosion(erosion_size, map);
      float xSize = ((float)map.size().width) * delta_;
      float ySize = ((float)map.size().height) * delta_;

      // Obstacle list
      vector<vector<cv::Point2i> > contours;

      // Temporary stuff
      cv::Mat mask, thresholded, debug;

      cv::cvtColor(map, debug, CV_GRAY2BGR);

      // Get area inside of walls
      cv::threshold(map,thresholded,127,255,cv::THRESH_BINARY);

      thresholded.copyTo(mask);

      cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      mask = Mat::zeros(map.size(),CV_8UC1);

      cv::drawContours(mask, contours, -1, cv::Scalar(255),-1);

      cv::Mat help = Mat::ones(map.size(),CV_8UC1)*255;

      thresholded.copyTo(help,mask);

      // Switch black & white
      cv::Mat objects = Mat::ones(map.size(), CV_8UC1)*255;
      cv::subtract(objects, help, objects);

      // Find objects
      cv::findContours(objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      int numObjects = contours.size();
      cv::Point2f centers[numObjects];
      float objectRadius[numObjects];
      cv::Moments moments;
      boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList(new twbTracking::proto::Pose2DList);

      for(int i = 0; i < numObjects; ++i) {
        if (cv::contourArea(contours[i]) < 5) continue;

        // Calculate objects center of gravity
        moments = cv::moments(contours[i], true);
        centers[i] = Point2f(moments.m10/moments.m00 , moments.m01/moments.m00);

        // Get objects radius
        cv::Point2f center;
        cv::minEnclosingCircle(contours[i],center,objectRadius[i]);

        // Add object as pose
        twbTracking::proto::Pose2D *pose2D1 = pose2DList->add_pose();
        pose2D1->set_x(centers[i].x * delta_ - xSize/2.0);
        pose2D1->set_y(centers[i].y * delta_ - ySize/2.0);
        pose2D1->set_orientation(objectRadius[i] * delta_);
        pose2D1->set_id(0);

        if (draw_debug) {
          cv::drawContours(debug, contours, i, cv::Scalar(255,191,0),-1);
          cv::circle(debug, centers[i], 3, cv::Scalar(139,0,0),-1);
          cv::circle(debug, center, objectRadius[i], cv::Scalar(0,0,139),1);
        }
      }
#ifndef __arm__
      cv_utils::imshowf("objects", debug);
      cv::waitKey(1);
#endif
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(3);
      imwrite("detection.png", debug, compression_params);
      INFO_MSG("Server returns obstacle list")
    }

    return pose2DListObjectsPublish;
  }
};

void objectsRequestFunction() {

  // Get the obstacles
  cv::Mat map = getObstacleMap();

  // Expand them
  const int erosion_size = detectionObstacleErosion / delta_;
  mapErosion(erosion_size, map);
  float xSize = ((float)map.size().width) * delta_;
  float ySize = ((float)map.size().height) * delta_;

  // Obstacle list
  vector<vector<cv::Point2i> > contours;

  // Temporary stuff
  cv::Mat mask, thresholded;

  // Get area inside of walls
  cv::threshold(map,thresholded,127,255,cv::THRESH_BINARY);

  thresholded.copyTo(mask);

  cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  mask = Mat::zeros(map.size(),CV_8UC1);

  cv::drawContours(mask, contours, -1, cv::Scalar(255),-1);
  cv::Mat help = Mat::ones(map.size(),CV_8UC1)*255;

  thresholded.copyTo(help,mask);

  // Switch black & white
  cv::Mat objects = Mat::ones(map.size(), CV_8UC1)*255;
  cv::subtract(objects, help, objects);

  // Find objects
  cv::findContours(objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  int numObjects = contours.size();
  cv::Point2f centers[numObjects];
  float objectRadius[numObjects];
  cv::Moments moments;
  pose2DListObjectsPublish->clear_pose();

  for(int i = 0; i < numObjects; ++i) {
    if (cv::contourArea(contours[i]) < 5) continue;

    // Calculate objects center of gravity
    moments = cv::moments(contours[i], true);
    centers[i] = Point2f(moments.m10/moments.m00 , moments.m01/moments.m00);

    // Get objects radius
    cv::Point2f center;
    cv::minEnclosingCircle(contours[i],center,objectRadius[i]);

    // Add object as pose
    twbTracking::proto::Pose2D *pose2D1 = pose2DListObjectsPublish->add_pose();
    pose2D1->set_x(centers[i].x * delta_ - xSize/2.0);
    pose2D1->set_y(centers[i].y * delta_ - ySize/2.0);
    pose2D1->set_orientation(objectRadius[i] * delta_);
    pose2D1->set_id(0);
  }
};

namespace mapServerReq {

std::string req = "map"; // Map produced by CoreSLAM
std::string erosion = "erosion"; // Eroded map of CoreSLAM
std::string path = "mapPath"; // Map with driven path

boost::shared_ptr<cv::Mat> getMap(bool erode = false, bool path = false, std::vector<cv::Point> *positions = NULL,
                                  std::vector<std::string> *positionsText = NULL) {
  const bool colored = (path || (positions != NULL)) ? true : false;

  boost::shared_ptr<cv::Mat> dst(new cv::Mat(getObstacleMap(colored, true)));
  if (erode) { // Expand them
    const int erosion_size = detectionObstacleErosion / delta_;
    mapErosion(erosion_size, *dst);
  }
  if (path) {
    // Get the current path pointer (actually, we need a mutex here, but we dont have any creepy r/w constelations)
    const int ts_poseHistorySizeCurrent = ts_poseHistorySize;

    // Define the polygon
    vector<cv::Point> path(ts_poseHistorySize);
    for (int idx = 0; idx < ts_poseHistorySizeCurrent; ++idx) {
      ::conversion::xyPose2cvPoint(float(ts_poseHistory[idx].x), float(ts_poseHistory[idx].y), float(delta_),
                                 path.at(idx).x, path.at(idx).y);
    }
    const cv::Point* pathpt[1] = { &path[0] };
    cv::polylines(*dst, pathpt, &ts_poseHistorySizeCurrent, 1 /*ncontours*/, false /*isClosed*/, cv::Scalar(255,0,0) /*red line*/);
  }
  if (positions != NULL) {
    if (!positions->empty()) {
      cv::Scalar colorMap;
      for (size_t idx = 0; idx < positions->size(); ++idx) {
        const int greyValue = 255.0f / positions->size() * idx;
        cv::circle( *dst, positions->at(idx), (positions->size() - idx) * 4/*radius*/, cv::Scalar(greyValue, greyValue, greyValue));
        // Draw the text
        if (positionsText != NULL) {
          if (positionsText->size() == positions->size()) {
            const double fontScale = 1;
            const int thickness = 1;
            cv::putText(*dst, positionsText->at(idx), positions->at(idx), cv::FONT_HERSHEY_PLAIN, fontScale, Scalar::all(0), thickness);
          } else {
            ERROR_MSG("positionsText->size() != positions->size()")
          }
        }
      }
    }
  }

  return dst;
}

// RSB Server function for the mapping server which replies with a 8bit eroded map of the environment
class mapEroded: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    INFO_MSG("Server returns obstacle map")
    return getMap(true);
  }
};

// RSB Server function for the mapping server which replies with a 8bit map of the environment
class map: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    INFO_MSG("Server returns map")
    return getMap();
  }
};

// RSB Server function for the RGB map with path and current location
class withCurrentPath: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    INFO_MSG("Server returns map with driven path")
    // Draw MCMC position
    cv::Point robotPosition; ::conversion::xyPose2cvPoint(pose.x, pose.y, float(delta_), robotPosition.x, robotPosition.y);
    std::vector<cv::Point> positions;
    positions.push_back(robotPosition);
    return getMap(false, true, &positions);
  }
};
}

void insertObject(boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr) {
  cv::Mat map = getObstacleMap();
  // cv::circle(map, cv::Point2f(objectPtr->x()/delta_,objectPtr->y()/delta_), (objectPtr->orientation()-0.05)/delta_, cv::Scalar(-255),-1);
  changeObstacleMap(map);
}

void deleteObject(boost::shared_ptr<twbTracking::proto::Pose2D> objectPtr) {
  cv::Mat map = getObstacleMap();
  // cv::circle(map, cv::Point2f(objectPtr->x()/delta_,objectPtr->y()/delta_), (objectPtr->orientation()-0.05)/delta_, cv::Scalar(255),-1);
  changeObstacleMap(map);
}

#define NUM_STATES 3
enum states {
  idle,
  slam,
  localization
};

static states slamState = slam;

std::string statesString[NUM_STATES] {
  "idle",
  "slam",
  "localization"
};

// Control the SLAM behaviour
void controlCoreSLAMBehaviour(boost::shared_ptr<std::string> e) {

  // Control the behaviour
  if ((e->compare("start") == 0) || (e->compare("slam") == 0))
    slamState = slam;
  else if ((e->compare("finish") == 0) || (e->compare("localization") == 0)) {
    if (slamState == slam) {
      cv::Mat map = getObstacleMap();
      const int erosion_size = detectionObstacleErosion / delta_;
      mapErosion(erosion_size, map);
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(3);
      imwrite("mapLoc.png", map, compression_params);
    }
    slamState = localization;
  } else if (e->compare("idle") == 0) {
    if (slamState == slam) {
      cv::Mat map = getObstacleMap();
      const int erosion_size = detectionObstacleErosion / delta_;
      mapErosion(erosion_size, map);
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(3);
      imwrite("mapIdle.png", map, compression_params);
    }
    slamState = idle;
  } else
    WARNING_MSG("STATEMACHINE: Received string " << *e << ". Remain in state " << statesString[slamState])

    INFO_MSG("STATEMACHINE: Received " << *e << " State is: " << statesString[slamState])
}

static types::position robotInitPosition;
static double mapOffset = 0;

void storeOdomData(boost::shared_ptr<rst::geometry::Pose> event) {
  mtxOdom.lock();
    odomTrans = event->translation();
    odomRot = event->rotation();
  mtxOdom.unlock();
}

bool
getOdomPose(ts_position_t& ts_pose)
{
  rst::geometry::Translation translation;
  rst::geometry::Rotation rotation;
  mtxOdom.lock();
    translation = odomTrans;
    rotation = odomRot;
  mtxOdom.unlock();

  // Convert from quaternion to euler
  Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
  Eigen::Matrix<double,3,1> rpy;
  ::conversion::quaternion2euler(&lidar_quat, &rpy);
  const double yaw = rpy(2);

  DEBUG_MSG( "CoreSLAM(RPY): " <<  rpy(0) << ", "<< rpy(1) << ", "<< rpy(2))
  DEBUG_MSG( "CoreSLAM(WXYZ): " <<  rotation.qw() << ", "<< rotation.qx() << ", "<< rotation.qy() << ", " << rotation.qz())

  ts_pose.x = translation.x()*millimeterPerMeter + ((TS_MAP_SIZE/2)*delta_*millimeterPerMeter); // convert to mm
  ts_pose.y = translation.y()*millimeterPerMeter + ((TS_MAP_SIZE/2)*delta_*millimeterPerMeter); // convert to mm
  ts_pose.theta = (yaw * 180/M_PI);

  DEBUG_MSG( "-------------------------------------------------------------------------------------------" )
  DEBUG_MSG( "Odometry: x(m): " <<  translation.x() << " y(m): " << translation.y() << " theta(rad): " << yaw)
  DEBUG_MSG( "Odometry map-centered: x(mm):" << ts_pose.x << " y(mm): " << ts_pose.y << " theta(deg): " << ts_pose.theta)

  return true;
}

bool
initMapper(const rst::vision::LocatedLaserScan& scan)
{

  // configure previous_odom
  if(!getOdomPose(prev_odom_))
     return false;

  // configure laser parameters
  lparams_.offset = 0.0;  // No offset of the lidar base
  lparams_.scan_size = scan.scan_values_size();
  lparams_.angle_min = scan.scan_angle_start()  * 180/M_PI;
  lparams_.angle_max = scan.scan_angle_end()  * 180/M_PI;
  lparams_.detection_margin = 0;
  lparams_.distance_no_detection = scan.scan_values_max() * millimeterPerMeter;

  // new coreslam instance
  ts_map_init(&ts_map_);
  ts_state_init(&state_, &ts_map_, &lparams_, &prev_odom_, sigma_xy_, sigma_theta_*180/M_PI, (int)(hole_width_*1000), 0, samples);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << ", hole_width=" << hole_width_ << ", delta=" << delta_);
  INFO_MSG("Initialization complete");
  return true;
}

void convertToScan(const rst::vision::LocatedLaserScan &scan , ts_scan_t &ranges) {
  ranges.nb_points = 0;
  const float delta_angle = scan.scan_angle_increment();

    for(int i=0; i < lparams_.scan_size; i++) {
      // Must filter out short readings, because the mapper won't
      if(scan.scan_values(i) > scan.scan_values_min() && scan.scan_values(i) < scan.scan_values_max()){
        // HACK "+ 120" is a workaround, and works only for startStep 44 and enStep 725!!!!
        ranges.x[ranges.nb_points] = cos((lparams_.angle_min + 120 )* M_PI/180.0f + i*delta_angle ) * (scan.scan_values(i)*millimeterPerMeter);
        ranges.y[ranges.nb_points] = sin((lparams_.angle_min + 120) * M_PI/180.0f + i*delta_angle) * (scan.scan_values(i)*millimeterPerMeter);
        ranges.value[ranges.nb_points] = TS_OBSTACLE;
        ranges.nb_points++;
      }
    }

}




bool addScan(const rst::vision::LocatedLaserScan &scan, ts_position_t &pose)
{
  // update odometry
  ts_position_t odom_pose;
  if(!getOdomPose(odom_pose))
     return false;

  state_.position.x += odom_pose.x - prev_odom_.x;
  state_.position.y += odom_pose.y - prev_odom_.y;
  state_.position.theta += odom_pose.theta - prev_odom_.theta; //- first_odom_.theta;
  prev_odom_ = odom_pose;

  ts_position_t prev = state_.position;

  // Do marcov localization on the map (this is done already in ts_iterative_map_building, but we can already do here for debugging)
//  ts_scan_t ranges;
//  convertToScan(scan , ranges);
//  INFO_MSG( "Pose1st " << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//  ts_monte_carlo_search(&state_.randomizer, &ranges, &ts_map_, &state_.position, state_.sigma_xy, state_.sigma_theta, -10000, NULL);
//  INFO_MSG( "Pose2st " << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//  INFO_MSG( "PoseOdom " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)

  ts_sensor_data_t data;
  data.position[0] = state_.position;
  if(lparams_.angle_max < lparams_.angle_min){
    // flip readings
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*millimeterPerMeter);
  }else{
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(i)*millimeterPerMeter);
  }

//  state_.position.theta = 90;
  DEBUG_MSG("Initial position step: "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta << ")")

  // Mapping
//  if(laser_count_ < 10){
//    INFO_MSG("BOOTSTRAP -- I assume that I stand still")
//  // not much of a map, let's bootstrap for now
//    ts_scan_t ranges;
//    ts_build_scan(&data, &ranges, &state_, 3 /*widening of the ray*/);
//    ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
//    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//  }else{

    // Monte carlo localization is done inside
    if (slamState == slam)
      ts_iterative_map_building(&data, &state_, true /*do Map Update*/);
    else /*if (slamState == localization)*/
      ts_iterative_map_building(&data, &state_, false /*do only localization*/);

    DEBUG_MSG("End postition step: "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta << ")")
    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
//  }
  // Set the new pose
  pose = state_.position;


  // Publish the new odometry data
  mtxOdom.lock();
  odomData->mutable_translation()->set_x((pose.x - mapOffset) * 1e-3); // From mm to m
  odomData->mutable_translation()->set_y((pose.y - mapOffset) * 1e-3); // From mm to m
  odomData->mutable_translation()->set_z(0.0f);
  odomData->mutable_rotation()->set_roll(0.0f);
  odomData->mutable_rotation()->set_pitch(0.0f);
  odomData->mutable_rotation()->set_yaw(pose.theta * M_PI / 180);
  mtxOdom.unlock();
  DEBUG_MSG("SLAM POSE: " << (pose.x - mapOffset) * 1e-3 << " m " << (pose.y - mapOffset) * 1e-3 << " m " << pose.theta * M_PI / 180 << " rad")
  informerOdometry->publish(odomData);

  // Copy the current pose, for the map server request with path drawn in
  if (ts_poseHistorySize < TS_POSE_HISTORY_SIZE) {
    ts_poseHistory[ts_poseHistorySize++] = pose;
  }

  return true;
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";
//  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  std::string localizationOutScope = "/localization";
  std::string serverScope = "/AMiRo_Hokuyo/server/slam";
  std::string mapAsImageOutScope = "/AMiRo_Hokuyo/image";
  std::string sExplorationScope = "/exploration";
  std::string sExplorationCmdScope = "/command";
  std::string sExplorationAnswerScope = "/answer";
  std::string sPathInputScope = "/pathReq/request";
  std::string sPathOutputScope = "/pathReq/answer";
  std::string sObjectsInputScope = "/objectsReq/request";
  std::string sObjectsOutputScope = "/objectsReq/answer";
  std::string sPushPathInputScope = "/pushPathServer/request";
  std::string sPushPathOutputScope = "/pushPathServer/answer";
  std::string insertObjectInscope = "/mapGenerator/insertObject";
  std::string deleteObjectInscope = "/mapGenerator/deleteObject";
  std::string pathServerReq = "path";
  std::string pushingPathServerReq = "getPushingPath";
  std::string obstacleServerReq = "getObjectsList";
  std::string remoteHost = "localhost";
  std::string remotePort = "4803";
  int robotID = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("robotID", po::value <int> (&robotID), "ID of robot for communication (default=0)")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("localizationOutScope", po::value < std::string > (&localizationOutScope), "Scope sending the odometry data")
    ("serverScope", po::value < std::string > (&serverScope), "Scope for handling server requests")
    ("mapServerReq", po::value < std::string > (&mapServerReq::req), "Map server request string (Std.: map)")
    ("mapServerObstacleReq", po::value < std::string > (&mapServerReq::erosion), "Map server obstacle request string (Std.: erosion)")
    ("mapServerPathReq", po::value < std::string > (&mapServerReq::path), "Map server obstacle request string with path drawn in (Std.: path)")
    ("pathServerReq", po::value < std::string > (&pathServerReq), "Path server request string (Std.: path)")
    ("obstacleServerReq", po::value < std::string > (&obstacleServerReq), "Obstacle server request string (Std.: getObjectsList)")
    ("remoteHost", po::value < std::string > (&remoteHost), "Remote spread daemon host name")
    ("remotePort", po::value < std::string > (&remotePort), "Remote spread daemon port")
    ("senImage", po::value < bool > (&sendMapAsCompressedImage), "Send map as compressed image")
    ("pathObstacleErosion", po::value < double > (&pathObstacleErosion), "Radius in meter of the erosion of objects for the path planer")
    ("detectionObstacleErosion", po::value < double > (&detectionObstacleErosion), "Radius in meter of the erosion of objects for the tabletop obstacle detection")
    ("mapAsImageOutScope", po::value < std::string > (&mapAsImageOutScope), "Scope for sending the map as compressed image to a remote spread daemon")
    ("sigma_xy", po::value < double > (&sigma_xy_), "XY uncertainty for marcov localization [m]")
    ("sigma_theta", po::value < double > (&sigma_theta_), "Theta uncertainty for marcov localization [m]")
    ("throttle_scans", po::value < int > (&throttle_scans_), "Only take every n'th scan")
    ("samples", po::value < int > (&samples), "Sampling steps of the marcov localization sampler")
    ("hole_width", po::value < double > (&hole_width_), "Width of impacting rays [m]")
    ("delta", po::value < double > (&delta_), "Resolution [m/pixel]")
    ("rayPruningAngleDegree", po::value < float > (&rayPruningAngleDegree), "Pruning of adjiacent rays if they differ to much on the impacting surface [0° .. 90°]")
    ("transX", po::value < double > (&transX),"Translation of the lidar in x [m]")
    ("transY", po::value < double > (&transY),"Translation of the lidar in y [m]")
    ("transZ", po::value < double > (&transZ),"Translation of the lidar in z [m]")
    ("rotX", po::value < double > (&rotX),"Rotation of the lidar around x (roll) [rad]")
    ("rotY", po::value < double > (&rotY),"Rotation of the lidar around y (pitch) [rad]")
    ("rotZ", po::value < double > (&rotZ),"Rotation of the lidar around z (yaw) [rad]");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);
  if (vm.count("robotID")) {
    serverScope.append(std::to_string(robotID));
  }

  // tinySLAM init
  ts_map_set_scale(meterPerMillimeter/delta_);  // Set TS_MAP_SCALE at runtime
  mapOffset = ((TS_MAP_SIZE/2)*delta_*millimeterPerMeter);

  // Create path planner
  PathPlanner pathPlannerObj(delta_);
  pathPlanner = &pathPlannerObj;

  // Get the RSB factory

#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif
  
  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ///////////////////////////////////////////////////////////////////////////////
  // Get the global participant config as a template
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
        {
          // disable socket transport
          rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();
          tmpPropSocket["enabled"] = boost::any(std::string("0"));

          // Get the options for spread transport, because we want to change them
          rsc::runtime::Properties tmpPropSpread  = tmpPartConf.mutableTransport("spread").getOptions();

          // enable socket transport
          tmpPropSpread["enabled"] = boost::any(std::string("1"));

          // Change the config
          tmpPropSpread["host"] = boost::any(std::string(remoteHost));

          // Change the Port
          tmpPropSpread["port"] = boost::any(std::string(remotePort));

          // Write the tranport properties back to the participant config
          tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
          tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
        }
  ///////////////////////////////////////////////////////////////////////////////
  
  // Register 
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);
  boost::shared_ptr<muroxConverter::MatConverter> matConverter(new muroxConverter::MatConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>> pose2DConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DConverter);
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB listener for incomming path request
  rsb::ListenerPtr pathListener = factory.createListener(sPathInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>>pathQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
  pathListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(pathQueue)));
  // Prepare RSB listener for incomming objects request
  rsb::ListenerPtr objectsListener = factory.createListener(sObjectsInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>objectsQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  objectsListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(objectsQueue)));
  // Prepare RSB listener for incomming pushing path request
  rsb::ListenerPtr pushPathListener = factory.createListener(sPushPathInputScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>pushPathQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
  pushPathListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(pushPathQueue)));
  // Prepare RSB informer for sending the path answer
  rsb::Informer<std::string>::Ptr pathInformer = factory.createInformer<std::string> (sPathOutputScope);
  // Prepare RSB informer for sending the objects answer
  rsb::Informer<std::string>::Ptr objectsInformer = factory.createInformer<std::string> (sObjectsOutputScope);
  // Prepare RSB informer for sending the pushing path answer
  rsb::Informer<std::string>::Ptr pushPathInformer = factory.createInformer<std::string> (sPushPathOutputScope);
  // Prepare RSB listener for controling the programs behaviour
  rsb::ListenerPtr expListener = factory.createListener(sExplorationScope);
  expListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&controlCoreSLAMBehaviour)));
  // Prepare RSB informer for sending the map as an compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<std::string> (mapAsImageOutScope, tmpPartConf);
  // Prepare RSB informer for sending the map as an compressed image
  informerOdometry = factory.createInformer<rst::geometry::PoseEuler> (localizationOutScope);
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));
  // Prepare RSB server for the map server
  rsb::patterns::LocalServerPtr server = factory.createLocalServer(serverScope, tmpPartConf, tmpPartConf);
  server->registerMethod(pathServerReq, rsb::patterns::LocalServer::CallbackPtr(new pathCallback()));
  server->registerMethod(pushingPathServerReq, rsb::patterns::LocalServer::CallbackPtr(new pushingPathCallback()));
  server->registerMethod(mapServerReq::req, rsb::patterns::LocalServer::CallbackPtr(new mapServerReq::map()));
  server->registerMethod(mapServerReq::erosion, rsb::patterns::LocalServer::CallbackPtr(new mapServerReq::mapEroded()));
  server->registerMethod(mapServerReq::path, rsb::patterns::LocalServer::CallbackPtr(new mapServerReq::withCurrentPath()));
  server->registerMethod(obstacleServerReq, rsb::patterns::LocalServer::CallbackPtr(new objectsCallback()));


  // prepare RSB listener for commands to insert an object in the map
  rsb::ListenerPtr insertObjectListener = factory.createListener(insertObjectInscope);
  insertObjectListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&insertObject)));

  // prepare RSB listener for commands to delete an object from the map
  rsb::ListenerPtr deleteObjectListener = factory.createListener(deleteObjectInscope);
  deleteObjectListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<twbTracking::proto::Pose2D>(&deleteObject)));

  // Init the CAN controller
  ::CAN = new(ControllerAreaNetwork);

  rst::vision::LocatedLaserScan scan;
  // Do SLAM
  while( true ){
    // Idle arround if the state says so
    if (slamState == idle) {
      usleep(50000); // Sleep for 50 ms
      continue;
    }

    if (!pathQueue->empty()) {
      INFO_MSG("Checking path requests.");
      pathRequestFunction(*pathQueue->pop());
      boost::shared_ptr<std::string> stringPublisher(new std::string("Fin"));
      pathInformer->publish(stringPublisher);
      INFO_MSG("Path request check finished.");
    }
    if (!pushPathQueue->empty()) {
      INFO_MSG("Checking pushing path requests.");
      pushingPathRequestFunction((twbTracking::proto::Pose2DList)*pushPathQueue->pop());
      boost::shared_ptr<std::string> stringPublisher(new std::string("Fin"));
      pushPathInformer->publish(stringPublisher);
      INFO_MSG("Pushing path request check finished.");
    }
    if (!objectsQueue->empty()) {
      INFO_MSG("Checking objects requests.");
      objectsRequestFunction();
      objectsQueue->pop();
      boost::shared_ptr<std::string> stringPublisher(new std::string("Fin"));
      objectsInformer->publish(stringPublisher);
      INFO_MSG("Objects request check finished.");
    }

    ++laser_count_;
    if ((laser_count_ % throttle_scans_) != 0)
      continue;

    // Fetch a new scan and store it to scan
    scan = *(lidarQueue->pop());
    getOdomPose(odom_pose);
    // We can't initialize CoreSLAM until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(scan))
        continue;
      got_first_scan_ = true;
    } else {

      ///////////////////////////////////////////////
      if(addScan(scan, pose))
      {
        DEBUG_MSG("scan processed");
        DEBUG_MSG("Updated the map");
      }
    }

    if (sendMapAsCompressedImage) {
      // Show the map as a cv Image with drawn in pose
      cv::Point robotPosition; ::conversion::xyPose2cvPoint(pose.x, pose.y, float(delta_), robotPosition.x, robotPosition.y);  // Draw MCMC position
      cv::Point robotOdomPosition; ::conversion::xyPose2cvPoint(odom_pose.x, odom_pose.y, float(delta_), robotOdomPosition.x, robotOdomPosition.y);  // Draw odometry
      std::vector<cv::Point> positions; std::vector<std::string> positionsText;
      positions.push_back(robotPosition); positionsText.push_back(std::string("S"));
      positions.push_back(robotOdomPosition); positionsText.push_back(std::string("L"));
      boost::shared_ptr<cv::Mat> mapImage(mapServerReq::getMap(false, true, &positions, &positionsText));
      DEBUG_MSG( "Pose " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)
      #ifndef __arm__
      cv_utils::imshowf("input", *mapImage);
      cv::waitKey(1);
      #endif
      // Send the map as image
      std::vector<uchar> buf;
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(85/*g_uiQuality [ 0 .. 100]*/);
      imencode(".jpg", *mapImage, buf, compression_params);

      // Send the data.
      //rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
      //informer->publish(frameJpg);
    }
  }

  return 0;
}

