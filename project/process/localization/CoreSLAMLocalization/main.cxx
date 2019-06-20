// ===== defines =====
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include "MSG.h"

// HACK for CMake/qtcreator
#ifndef __arm__
#ifdef IS_AMIRO
#define __arm__
#endif
#endif

// ===== Includes =====
#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// For reading and writing maps to file
#include <fstream>

// Converting helpers
#include <Eigen/Geometry>

// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>
#include <types/TargetPoseEuler.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/navigation/Path.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize

#include "pathplanner.h"

// To move AMiRo
#include <ControllerAreaNetwork.h>

#include <mutex> // std::mutex

#include <utils.h>

// tinySLAM
#ifdef __cplusplus
extern "C"{
#endif
#include <libs/CoreSLAM/CoreSLAM.h>
#ifdef __cplusplus
}
#endif

// Include own converter for setting new position
#include <converter/vecIntConverter/main.hpp>

#include "map-io.h"

// ===== Namespaces =====
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace muroxConverter;

// ===== Global variables =====
// Parameters needed for Marcov sampling
static double sigma_xy_ = 10;  // mm
static double sigma_theta_ = 0.05;  // rad
static double sigma_xy_new_position = 1000;  // used when a new position is set via RSB (mm)
static double sigma_theta_new_position = 0.3;  // used when a new position is set via RSB (rad)
static double sigma_decrease_rate = 0.75; // rate by which sigma is decreased (until it reached sigma_*_)

static double initialX = 0;
static double initialY = 0;
static double initialTheta = 0;

static int samples = 100; // Number of resampling steps
// parameters for coreslam
static double hole_width_ = 0.1;  // m
static float delta_ = 0.02;  // Meter per pixel
static ts_map_t ts_map_;
static ts_state_t state_;
static ts_position_t position_;
static ts_position_t prev_odom_;
static ts_laser_parameters_t lparams_;
#define METERS_TO_MM    1000
#define METERS_TO_UM    1e6
#define MM_TO_METERS    0.001
#define MM_TO_UM        1000
#define PIXEL_TO_MM(x) ( x * delta_ * METERS_TO_MM )
#define PIXEL_TO_UM(x) ( x * delta_ * METERS_TO_UM )
#define SECONDS_TO_MS   1000
#define DEGREE_TO_RAD   M_PI / 180
#define RAD_TO_URAD     1e6
#define SECONDS_TO_US   1000000
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

// Switch for localization
std::mutex mtxDoMapUpdate;
static bool doMapUpdate = false;

// path to goal
ts_position_t homePose = { 0, 0, 0 };
bool savedHomePose = false;
ts_position_t targetPose = {0,0,0};

// Convinience
static bool sendMapAsCompressedImage = false;

cv::Size debugImageSize; // size of debug image

// Paths to read map from
static std::string mapImagePath = "";
static std::string mapPGMPath = "";
static std::string mapValidPositionsPNGPath = "";
static bool flipHorizontal = false;

static bool precomputeOccupancyMap = false;
cv::Mat1b occupancyMap;

// Erosion for obstacle map including antenna and Hokuyo cable and some safety distance
static float erosionRadius = 0.15;

// init path planner
PathPlanner pathplanner = PathPlanner();

ControllerAreaNetwork can;

std::mutex map_to_odom_mutex_;
std::mutex mtxOdom;       // mutex for odometry messages
std::mutex mtxPathToDraw; // mutex for the path to draw
std::list<cv::Point2i> pathToDraw; // Path for drawing

static rst::geometry::Translation odomTrans;
static rst::geometry::Rotation odomRot;

ts_position_t pose = {0,0,0};
ts_position_t odom_pose = {0,0,0};
ts_position_t offset = {0,0,0};

// Local informer
rsb::Informer< rst::geometry::TargetPoseEuler >::Ptr targetInformer;
rsb::Informer< std::string >::Ptr emergencyStopSwitchInformer;
rsb::Informer< std::vector<int> >::Ptr motorInformer;
// Global rsb informer for debug images
rsb::Informer<std::string>::Ptr informer;
rsb::Informer<rst::navigation::Path>::Ptr pathInformer;
rsb::Informer<std::string>::Ptr homingInformer;

// Local listener for emergency halts
rsb::ListenerPtr emergencyHaltListener;
boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> emergencyHaltQueue;

// abort navigation
bool abortNavigation = false;
std::mutex mtxAbortNavigation;

// ===== Functions =====

inline float normalizeAngle(float angle) {
    // normalize angle to (-PI; +PI]
    if (angle < -M_PI || angle >= M_PI) {
        angle = fmod(angle + M_PI, 2*M_PI);
        if (angle < 0)
            angle += 2 * M_PI;
        angle = angle - M_PI;
    }

    return angle;
}

ts_position_t convertRSBPoseToTsPose(boost::shared_ptr< rst::geometry::Pose > newPosition) {
    ts_position_t position;
    position.x = newPosition->translation().x() * METERS_TO_MM;
    position.y = newPosition->translation().y() * METERS_TO_MM;

    // Convert from quaternion to euler
    rst::geometry::Rotation rotation = newPosition->rotation();
    Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
    Eigen::Matrix<double,3,1> rpy;
    ::conversion::quaternion2euler(&lidar_quat, &rpy);
    const double yaw = rpy(2);
    position.theta = yaw * 180.0f / M_PI;

    return position;
}

std::mutex sendMapAsCompressedImageMutex;
void setSendMapAsCompressedImage(boost::shared_ptr<std::string> v) {
    sendMapAsCompressedImageMutex.lock();
    bool b = (*v == "enable") || (*v == "true") || (*v == "1");
    INFO_MSG("Setting sendMapAsCompressedImage to " << b);
    sendMapAsCompressedImage = b;
    sendMapAsCompressedImageMutex.unlock();
}

cv::Mat getOccupancyMap() {
  DEBUG_MSG("Generating occupancy map...");
  // Convert the map to a cv::Mat image
  cv::Mat map(cv::Size(ts_map_.size,ts_map_.size), CV_8UC1);
  cv::Mat tmp = cv::Mat(ts_map_.size, ts_map_.size, CV_16UC1, static_cast<void*>(&ts_map_.map[0]));
  tmp.convertTo(map, CV_8UC1, 255.0f / TS_NO_OBSTACLE);  // Convert to 8bit depth image

  // Threshold image
  //for (ssize_t idx = 0; idx < map.rows * map.cols; ++idx)
  //    map.at<uint8_t>(idx) = (map.at<uint8_t>(idx) > 127 /*126*/) ? 255 : 0; // note: 126 due to rounding errors
  cv::threshold(map, map, 127, 255, CV_THRESH_BINARY);
  DEBUG_MSG("done thresholding, eroding now...");

  // Dilate to remove small errors
  /*cv::dilate(map, map, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                 cv::Size(2,2),
                                                 cv::Point(1,1)));*/
  // Erode w.r.t. robot size
  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                            cv::Size(2*erosionRadius/delta_ + 1, 2*erosionRadius/delta_ + 1),
                            cv::Point(erosionRadius/delta_, erosionRadius/delta_));
  cv::erode(map, map, structuringElement);

 // cv::imwrite("occupancymap.png", map);

  return map;
}

std::list<cv::Point2i> getPath(const ts_position_t &targetPose) {
    // Convert from tinySLAM coordinates (mm) to pixel coordinates
    cv::Point2i start(pose.x * MM_TO_METERS / delta_, pose.y * MM_TO_METERS / delta_);
    cv::Point2i goal(targetPose.x * MM_TO_METERS / delta_, targetPose.y * MM_TO_METERS / delta_);

    // Generate occupacy map if not done yet
    if (!occupancyMap.data) {
        occupancyMap = getOccupancyMap();
    }
    // Make sure start and goal are reachable
    cv::circle(occupancyMap, start, erosionRadius/delta_ + 1, cv::Scalar(255,255,255), -1/*thickness/filled*/);
    cv::circle(occupancyMap, goal, erosionRadius/delta_ + 1, cv::Scalar(255,255,255), -1/*thickness/filled*/);

    DEBUG_MSG("Got occupancy map");

    // DEBUG START
    if (sendMapAsCompressedImage) {
        DEBUG_MSG("Sending occupancy map via rsb");
        cv::Mat tmpMat(occupancyMap.size(), CV_8UC3);
        cv::cvtColor(occupancyMap, tmpMat, CV_GRAY2RGB, 3);
        cv::resize(tmpMat, tmpMat, cv::Size(tmpMat.size().height / 2, tmpMat.size().width / 2));

        std::vector<uchar> buf;
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(85/*g_uiQuality [ 0 .. 100]*/);
        imencode(".jpg", tmpMat, buf, compression_params);

        // Send the data.
        rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
        informer->publish(frameJpg);

        #ifndef __arm__
        cv::imshow("Map for path calculation", tmpMat);
        cv::waitKey(1);
        #endif
    }
    // DEBUG END

    // Get path
    boost::uint64_t startTime = rsc::misc::currentTimeMillis();
    std::list<cv::Point2i> path = pathplanner.getPath(start, goal, occupancyMap);
    boost::uint64_t stopTime = rsc::misc::currentTimeMillis();
    SUCCESS_MSG("calculating Path took: " << (stopTime - startTime) << " ms");

    // Optimize path
    DEBUG_MSG("before removing redundant nodes: " << path.size());
    pathplanner.removeRedundantNodes(path);
    DEBUG_MSG("before optimizing path: " << path.size());
    pathplanner.optimizePath(path, occupancyMap);
    DEBUG_MSG("after optimizing path: " << path.size());

    INFO_MSG("calculated and optimized path");

    // TODO: convert to coordinates to outer space?
    return path;
}

float targetSpeed0 = 0.1f;
float targetSpeed1 = 1.0f;
void drivePath(std::list<cv::Point2i> &path) {
    // calculate distance to checkpoint
    cv::Point2i immediatePosePX = path.front();
    ts_position_t immediatePoseMM = {
        PIXEL_TO_MM(immediatePosePX.x),
        PIXEL_TO_MM(immediatePosePX.y),
        0
    };
    DEBUG_MSG("Next waypoint: " << immediatePosePX << " (left: " << path.size() << ")");
    DEBUG_MSG("in MM: " << immediatePoseMM.x << "," << immediatePoseMM.y);

    float distance = MM_TO_METERS * sqrt( (immediatePoseMM.x - pose.x) * (immediatePoseMM.x - pose.x)
                           + (immediatePoseMM.y - pose.y) * (immediatePoseMM.y - pose.y) );

    const float minDistanceToTarget = 0.1f; // meter
    DEBUG_MSG("distance to target: " << distance << " m");
    if (distance < minDistanceToTarget) {
        // arrived at checkpoint -> remove it
        DEBUG_MSG("Reached waypoint, deleting it");
        path.pop_front();

        // Stop the robot
        //boost::shared_ptr< std::vector<int> > targetSpeed( new std::vector<int>(2,0) );
        //motorInformer->publish(targetSpeed);
    } else {
        // update target position
        double globalTargetAngle = atan2( immediatePoseMM.y - pose.y,
                                          immediatePoseMM.x - pose.x); // in rad [-PI; +PI]
        double globalRobotAngle = pose.theta * DEGREE_TO_RAD; // in rad [?? ; ??]
        double relativeAngle = globalTargetAngle - globalRobotAngle; // in rad
        relativeAngle = normalizeAngle(relativeAngle);

        // firstly rotate to waypoint
        DEBUG_MSG("Angle to target: " << relativeAngle);
        if (abs(relativeAngle) > 0.30f /*M_PI / 45*/) { // precision of rotation (0.15 rad ~ 8.59°)//(PI/45 = 4°)
            DEBUG_MSG("angle to waypoint too big -> rotating");

            boost::shared_ptr< std::vector<int> > targetSpeed( new std::vector<int>(2,0) );
            // trick 17 (tm): drive forward when rotating?!
            targetSpeed->at(0) = targetSpeed0 * METERS_TO_UM;

            float angularVelocity = targetSpeed1;
            if (relativeAngle < 0) {
                angularVelocity = -angularVelocity;
            }

            targetSpeed->at(1) = angularVelocity * RAD_TO_URAD;
            motorInformer->publish(targetSpeed);

            //usleep(0.05f * SECONDS_TO_US);

        } else { // angle is small enough
            DEBUG_MSG("angle to waypoint small enough, driving towards it now");

            const float velocity = 0.3f; // m/s

            // Set the motor speed
            // Velocity send in µm/s
            // Angular velocity send in µrad/s
            boost::shared_ptr< std::vector<int> > targetSpeed( new std::vector<int>(2,0) );
            targetSpeed->at(0) = velocity * METERS_TO_UM;
            float t = distance / velocity;
            float w_z = relativeAngle / t;
//            DEBUG_MSG("w_z: " << w_z);
            w_z = std::max<float>(std::min<float>(M_PI, w_z), -M_PI);
            targetSpeed->at(1) = w_z * RAD_TO_URAD;
            motorInformer->publish(targetSpeed);
        }

        usleep(100000);
    }
}

int convertDataToScan(boost::shared_ptr< rst::vision::LocatedLaserScan > data , rst::vision::LocatedLaserScan &rsbScan) {
  
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return 1;

//  DEBUG_MSG( "Scan rec.")
//  Eigen::Quaterniond lidar_quat(rotation.qw(), rotation.qx(), rotation.qy(), rotation.qz());
//  Eigen::AngleAxisd lidar_angle(lidar_quat);
//  Eigen::Matrix<double,3,1> rpy = lidar_angle.toRotationMatrix().eulerAngles(0,1,2);
//  const double yaw = rpy(2);

  // Copy the whole scan
  rsbScan = *data;

  // Shifting the rays
//  DEBUG_MSG("Startscan: " << rsbScan.scan_angle_start() * 180.0 / M_PI)
//  DEBUG_MSG("Endscan: " << rsbScan.scan_angle_end() * 180.0 / M_PI)
//  DEBUG_MSG("Angle: " << rsbScan.scan_angle() * 180.0 / M_PI)
//  DEBUG_MSG("Size: " << rsbScan.scan_values_size())
//  DEBUG_MSG("Inc: " << rsbScan.scan_angle_increment() * 180.0 / M_PI)
//  rsbScan.set_scan_angle_increment(-rsbScan.scan_angle_increment);

  return 0;
}

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

  //DEBUG_MSG( "CoreSLAM(RPY): " <<  rpy(0) << ", "<< rpy(1) << ", "<< rpy(2))
  //DEBUG_MSG( "CoreSLAM(WXYZ): " <<  rotation.qw() << ", "<< rotation.qx() << ", "<< rotation.qy() << ", " << rotation.qz())

  ts_pose.x = translation.x()*METERS_TO_MM + ((ts_map_.size/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.y = translation.y()*METERS_TO_MM + ((ts_map_.size/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.theta = (yaw * 180/M_PI);

//  DEBUG_MSG( "-------------------------------------------------------------------------------------------" )
//  DEBUG_MSG( "Odometry: x(m): " <<  translation.x() << " y(m): " << translation.y() << " theta(rad): " << yaw)
//  DEBUG_MSG( "Odometry map-centered: x(mm):" << ts_pose.x << " y(mm): " << ts_pose.y << " theta(deg): " << ts_pose.theta)

  return true;
}

bool
initMapper(const rst::vision::LocatedLaserScan& scan)
{
  // configure previous_odom
  if(!getOdomPose(prev_odom_))
     return false;
  //position_ = prev_odom_;
  position_.x = initialX;
  position_.y = initialY;
  position_.theta = initialTheta;

  // configure laser parameters
  lparams_.offset = 0.0;  // No offset of the lidar base
  lparams_.scan_size = scan.scan_values_size();
  lparams_.angle_min = scan.scan_angle_start()  * 180/M_PI;
  lparams_.angle_max = scan.scan_angle_end()  * 180/M_PI;
  lparams_.detection_margin = 0;
  lparams_.distance_no_detection = scan.scan_values_max() * METERS_TO_MM;

  // new coreslam instance
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, sigma_xy_, sigma_theta_*180/M_PI , (int)(hole_width_*1000), 0, samples);

  INFO_MSG("Initialized with sigma_xy=" << sigma_xy_<< ", sigma_theta=" << sigma_theta_ << ", hole_width=" << hole_width_ << ", delta=" << delta_);
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
        ranges.x[ranges.nb_points] = cos((lparams_.angle_min + 120 )* M_PI/180.0f + i*delta_angle ) * (scan.scan_values(i)*METERS_TO_MM);
        ranges.y[ranges.nb_points] = sin((lparams_.angle_min + 120) * M_PI/180.0f + i*delta_angle) * (scan.scan_values(i)*METERS_TO_MM);
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

  float odomIncrement = sqrt( pow(odom_pose.x - prev_odom_.x, 2) + pow(odom_pose.y - prev_odom_.y, 2) );

  float phi = atan2(odom_pose.y - prev_odom_.y, odom_pose.x - prev_odom_.x); // orientation of movement
  bool drivingForwards = sqrt( pow(cos(phi) - cos(odom_pose.theta * M_PI/180), 2) + pow(sin(phi) - sin(odom_pose.theta * M_PI/180), 2) ) < 1.0f;
  if (!drivingForwards) {
      odomIncrement = -odomIncrement;
  }

  state_.position.x += odomIncrement * cos(pose.theta * M_PI/180);
  state_.position.y += odomIncrement * sin(pose.theta * M_PI/180);
  state_.position.theta += odom_pose.theta - prev_odom_.theta;
  prev_odom_ = odom_pose;

//  ts_position_t prev = state_.position;

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
      data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*METERS_TO_MM);
  }else{
    for(int i=0; i < scan.scan_values_size(); i++)
      data.d[i] = (int) (scan.scan_values(i)*METERS_TO_MM);
  }

  // Mapping
  if(laser_count_ < 10){
    WARNING_MSG("BOOTSTRAP")
  // not much of a map, let's bootstrap for now
    ts_scan_t ranges;
    ts_build_scan(&data, &ranges, &state_, 3 /*widening of the ray*/);
    mtxDoMapUpdate.lock();
    if (doMapUpdate) {
        ts_map_update(&ranges, &ts_map_, &state_.position, 50, (int)(hole_width_*1000));
    }
    mtxDoMapUpdate.unlock();
    DEBUG_MSG("Update step, " << laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
  }else{

    // Monte carlo localization is done inside
    mtxDoMapUpdate.lock();
    ts_iterative_map_building(&data, &state_, doMapUpdate);
    mtxDoMapUpdate.unlock();

//    DEBUG_MSG("Iterative step, "<< laser_count_ << ", now at (" << state_.position.x << ", " << state_.position.y << ", " << state_.position.theta)
//    DEBUG_MSG("Correction: "<< state_.position.x - prev.x << ", " << state_.position.y - prev.y << ", " << state_.position.theta - prev.theta)
  }
  // Set the new pose
  pose = state_.position;

  return true;
}

/**
 * @brief getClosestValidPosition Calculates the closest valid pose
 * @param currentPose Current pose
 * @param targetPose Target pose set by calculating the shortest path given a map with valid positions
 * @return true on success, otherwise false
 */
bool getClosestValidPosition(const ts_position_t &currentPose, ts_position_t &targetPose) {
  ts_map_t map;
  INFO_MSG("path " <<  mapValidPositionsPNGPath);
  if (!loadMapFromImage(&map, mapValidPositionsPNGPath, flipHorizontal)) {
    ERROR_MSG("No valid map");
    return false;
  }


  cv::Mat image = cv::Mat(map.size, map.size, CV_16U, static_cast<void*>(&map.map[0]));
  std::vector<cv::Point2i> validCells;

//  dst.convertTo(dst, CV_8U, 0.00390625);  // Convert to 8bit depth image
//  cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image


  INFO_MSG("image.cols " <<  image.cols);
  INFO_MSG("image.rows " <<  image.rows);
  // Get the valid cells
  for(int col = 0; col < image.cols; ++col) {
    for(int row = 0; row < image.rows; ++row) {
      // Store all the gray values (52480_dec==CD00_hex)
//        INFO_MSG("map("<< row << ", " << col << ") = " << image.at<unsigned short>(row,col));
      if(uint(image.at<unsigned short>(row,col)) == 52480) {
        validCells.push_back(cv::Point2i(col, row));
      }
    }
  }

  // Calculate the current position in the pixel coordinates
  cv::Point robotPosition(pose.x * MM_TO_METERS / delta_ , pose.y * MM_TO_METERS / delta_ );
  // cv::Point robotPosition(505 , 388 ); // Set the position for testing near the bed

  // Calculate the pixel with the shortest distance
  int minNorm = 999999;
  size_t minIdx;
  for(size_t idx=0; idx < validCells.size(); ++idx) {
    int norm = cv::norm(robotPosition-validCells.at(idx));
    if ( minNorm > norm) {
      minNorm = norm;
      minIdx = idx;
    }
  }
  INFO_MSG("minimal norm " << minNorm);
  INFO_MSG("minimal index " << minIdx);

  targetPose.x = float(validCells.at(minIdx).x) * float(delta_) / MM_TO_METERS;
  targetPose.y = float(validCells.at(minIdx).y) * float(delta_) / MM_TO_METERS;
  targetPose.theta = 0.0f;  // NOTE The path planner does not respect the orientation

#ifndef __arm__
  /////////////////////////////////////////////////
  // Convert to RGB image
  cv::Mat imageColor, imageTmp;
  image.convertTo(imageTmp, CV_8U, 0.00390625);  // Convert to 8bit depth image
  cv::cvtColor(imageTmp, imageColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image

  // Draw current position
  cv::circle( imageColor, robotPosition, 0, cv::Scalar( 0, pow(2,8)-1, 0), 10, 8 );

  // Draw closest point position
//  for(size_t idx=0; idx < validCells.size(); ++idx) {
//    cv::circle( imageColor, validCells.at(idx), 0, cv::Scalar( 0, 0, pow(2,8)-1), 10, 8 );
//  }
  cv::circle( imageColor, validCells.at(minIdx), 0, cv::Scalar( 0, 0, pow(2,8)-1), 10, 8 );


  cv::imshow("Current and valid position", imageColor);
  cv::waitKey(1);
  /////////////////////////////////////////////////
#endif
  free(map.map);
  return true;
}

void publishPath(const std::list<cv::Point2i> &path) {
    boost::shared_ptr<rst::navigation::Path> rsbPath(new rst::navigation::Path);
    // first add the current position to the path
    rst::geometry::Pose *rsbPose = rsbPath->add_poses();
    rsbPose->mutable_translation()->set_x(pose.x * MM_TO_METERS);
    rsbPose->mutable_translation()->set_y(pose.y * MM_TO_METERS);
    rsbPose->mutable_translation()->set_z(0);

    Eigen::Matrix<double,3,1> rpy;
    rpy(0) = 0;
    rpy(1) = 0;
    rpy(2) = pose.theta * M_PI / 180.0f;
    Eigen::Quaternion<double> quat;
    ::conversion::euler2quaternion(&rpy, &quat);

    rsbPose->mutable_rotation()->set_qx(quat.x());
    rsbPose->mutable_rotation()->set_qy(quat.y());
    rsbPose->mutable_rotation()->set_qz(quat.z());
    rsbPose->mutable_rotation()->set_qw(quat.w());

    // add the rest of the path
    for (auto &point : path) {
        rsbPose = rsbPath->add_poses();
        rsbPose->mutable_translation()->set_x(point.x * delta_);
        rsbPose->mutable_translation()->set_y(point.y * delta_);
        rsbPose->mutable_translation()->set_z(0);
        rsbPose->mutable_rotation()->set_qx(0);
        rsbPose->mutable_rotation()->set_qy(0);
        rsbPose->mutable_rotation()->set_qz(0);
        rsbPose->mutable_rotation()->set_qw(0);
    }
    pathInformer->publish(rsbPath);
}

void rotateToPose(const ts_position_t &targetPose) {
    float targetAngle = targetPose.theta * DEGREE_TO_RAD;
    DEBUG_MSG("targetAngle:     " << targetAngle);
    float currentAngle = pose.theta * DEGREE_TO_RAD;
    DEBUG_MSG("currentAngle:    " << currentAngle);
    float rotateBy = targetAngle - currentAngle;
    DEBUG_MSG("rotateBy:        " << rotateBy);
    rotateBy = normalizeAngle(rotateBy);
    DEBUG_MSG("rotateBy (norm): " << rotateBy);

    // Define the steering message for sending over RSB
    boost::shared_ptr< rst::geometry::TargetPoseEuler > steering(new rst::geometry::TargetPoseEuler);
    // Get the mutable objects
    rst::geometry::RotationEuler *steeringRotation = steering->mutable_target_pose()->mutable_rotation();
    rst::geometry::Translation *steeringTranslation = steering->mutable_target_pose()->mutable_translation();
    steeringRotation->set_roll(0);
    steeringRotation->set_pitch(0);
    steeringRotation->set_yaw(rotateBy);
    steeringTranslation->set_x(0);
    steeringTranslation->set_y(0);
    steeringTranslation->set_z(0);
    steering->set_target_time(3000);

    targetInformer->publish(steering);
}

/**
 * @brief driveToPoseWithObstacleAvoidance plans the path to the given target pose and drives along this path. When a static obstacle is encountered the map is updated and the path recalculated.
 *        When no path is found the robot will rotate to the given pose's orientation.
 * @param targetPose
 * @return wether target position was reached or not
 */
bool driveToPoseWithObstacleAvoidance(const ts_position_t &targetPose) {
    std::list<cv::Point2i> path = getPath(targetPose);

    if (path.empty()) {
        ERROR_MSG("Path is empty!");
        //return false;
    } else {
        INFO_MSG("Path not empty :)");
    }

    mtxAbortNavigation.lock();
    abortNavigation = false;
    mtxAbortNavigation.unlock();

    // Drive the path
    uint obstacleCounter = 0; // how often we saw an obstacle
    while(!path.empty()) {

      mtxAbortNavigation.lock();
      if (abortNavigation) {
          INFO_MSG("Aborting navigation");
          abortNavigation = false;
          mtxAbortNavigation.unlock();

          // stop the robot
          boost::shared_ptr< std::vector<int> > targetSpeed( new std::vector<int>(2,0) );
          motorInformer->publish(targetSpeed);

          return false;
      } else {
        mtxAbortNavigation.unlock();
      }

      // There's an obstacle in the way...
      if (!emergencyHaltQueue->empty()) {
          emergencyHaltQueue->pop();
          obstacleCounter++;

          if (obstacleCounter > 50) { // wait for obstacle to move (~5 seconds)
              DEBUG_MSG("doHoming: obstacle does not move, planning new path");
              obstacleCounter = 0;

              // before driving new path, back up
              emergencyStopSwitchInformer->publish(boost::shared_ptr<std::string>(new std::string("off"))); // disable emergency stop, so moving is possible

              const float velocity = 0.2f; // m/s

              // Set the motor speed
              // Velocity send in µm/s
              // Angular velocity send in µrad/s
              boost::shared_ptr< std::vector<int> > targetSpeed( new std::vector<int>(2,0) );
              targetSpeed->at(0) = -velocity * METERS_TO_UM;
              motorInformer->publish(targetSpeed);

              usleep(1 * SECONDS_TO_US); // wait for movement

              // stop again
              targetSpeed = boost::shared_ptr< std::vector<int> >( new std::vector<int>(2,0) );
              motorInformer->publish(targetSpeed);

              // enable emergency stop again
              emergencyStopSwitchInformer->publish(boost::shared_ptr<std::string>(new std::string("on")));


              // If this is the last waypoint and we're close to it, discard it and assume we reached the global goal
              if (path.size() == 1) {
                  DEBUG_MSG("doHoming: reached last waypoint");
                  cv::Point2i immediatePosePX = path.front();

                  float distance = MM_TO_METERS * sqrt( pow(PIXEL_TO_MM(immediatePosePX.x) - pose.x, 2)
                                         + pow(PIXEL_TO_MM(immediatePosePX.y) - pose.y, 2) );

                  DEBUG_MSG("distance is " << distance << " m");
                  if (distance < 0.8f) {
                    path.pop_front();
                    DEBUG_MSG("discarding last waypoint");
                    break;
                  }
              }

              // must be a new static obstacle, so add it to the map
              mtxDoMapUpdate.lock();
              bool oldDoMapUpdate = doMapUpdate;
              doMapUpdate = true;
              mtxDoMapUpdate.unlock();

              // wait for main thread to update map
              int old_laser_count_ = laser_count_;
              while (laser_count_ <= old_laser_count_) {
                  DEBUG_MSG("doHoming: waiting for main thread to update map");
                  usleep(100000);
              }

              mtxDoMapUpdate.lock();
              doMapUpdate = oldDoMapUpdate;
              mtxDoMapUpdate.unlock();

              // re-compute the occupancy map
              occupancyMap = getOccupancyMap();

              // get the new path
              path = getPath(targetPose);

              if (path.empty()) {
                  ERROR_MSG("No new path was found!");
                  return false;
              }

          } else {
              DEBUG_MSG("doHoming: waiting for obstacle to move");
          }
      } else {
          obstacleCounter = 0;

          drivePath(path);
          mtxPathToDraw.lock();
          pathToDraw = path;
          mtxPathToDraw.unlock();

          continue;
      }

      publishPath(path);
      usleep(100000);
    }

    // After driving path, rotate to requested pose/theta
    DEBUG_MSG("Done driving along path, rotating now");
    emergencyStopSwitchInformer->publish(boost::shared_ptr<std::string>(new std::string("off"))); // disable emergency stop, rotation is always safe
    rotateToPose(targetPose);
    usleep(6 * SECONDS_TO_US);
    emergencyStopSwitchInformer->publish(boost::shared_ptr<std::string>(new std::string("on")));

    return true;
}

void abortHomingRequestHandler(boost::shared_ptr<std::string> e) {
    if (!e->compare(std::string("abort"))) {
        INFO_MSG("Requesting navigation abort");
        mtxAbortNavigation.lock();
        abortNavigation = true;
        mtxAbortNavigation.unlock();
    } else {
        ERROR_MSG("is not abort commando");
    }
}

void homingRequestHandler(boost::shared_ptr<std::string> e) {
    INFO_MSG("Received homing request via RSB")
    bool reachedGoal = false;

    if(!e->compare(std::string("homing"))) {
      INFO_MSG("Do homing");
      reachedGoal = driveToPoseWithObstacleAvoidance(homePose);
    } else if (!e->compare(std::string("save"))) {
      INFO_MSG("Drive to next save position");
      ts_position_t validPose = {0, 0, 0};
      if(!getClosestValidPosition(pose, validPose)) {
        ERROR_MSG("Fail to get next save position");
        return;
      } else {
        INFO_MSG("Start path calculation");
        reachedGoal = driveToPoseWithObstacleAvoidance(validPose);
        INFO_MSG("Finish path calculation");
      }
    } else if (!e->compare(std::string("target"))) {
        INFO_MSG("Drive to target pose");
        reachedGoal = driveToPoseWithObstacleAvoidance(targetPose);
    } else {
      ERROR_MSG("Wrong homing command: " << *e);
      return;
    }

    if (reachedGoal) {
        boost::shared_ptr<std::string> response = boost::shared_ptr<std::string>(new std::string("done"));
        homingInformer->publish(response);
    } else {
        boost::shared_ptr<std::string> response = boost::shared_ptr<std::string>(new std::string("failed"));
        homingInformer->publish(response);
    }
}

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
  std::string odomInScope = "/AMiRo_Hokuyo/gps";
  std::string targetPoseOutScope = "/targetPositions";
  std::string mapAsImageOutScope = "/CoreSLAMLocalization/image";
  std::string saveMapInScope = "/saveMap";
  std::string setPositionScope = "/setPosition";
  bool setPositionUseVec = false;
  std::string positionOutScope = "/position";
  bool publishPoseEuler = false;
  std::string pathOutScope = "/path";
  std::string sendMapSwitchScope = "/sendMap";
  std::string homingInScope = "/homing";
  std::string emergencyHaltInScope = "/AMiRo_Hokuyo/emergencyHalt";
  std::string remoteHost = "localhost";
  std::string remotePort = "4803";
  std::size_t tsMapSize = 2048;
  std::size_t tsMapSizeMaxVisualization = 700;
  std::string loadOccupancyMapFromFile = "";
  std::vector<std::string> targetPoseStrVec;
  std::string targetPoseInScope = "/setTargetPose";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data")
    ("odominscope", po::value < std::string > (&odomInScope), "Scope for receiving odometry data")
    ("targetPoseScope", po::value < std::string > (&targetPoseOutScope), "Scope for sending target positions")
    ("hominginscope", po::value < std::string > (&homingInScope), "Scope for receiving the homing trigger")
    ("emergencyhaltinscope", po::value< std::string > (&emergencyHaltInScope), "Scope for receiving emergency halt notifications")
    ("remoteHost", po::value < std::string > (&remoteHost), "Remote spread daemon host name")
    ("remotePort", po::value < std::string > (&remotePort), "Remote spread daemon port")
    ("senImage", po::value < bool > (&sendMapAsCompressedImage), "Send map as compressed image")
    ("mapAsImageOutScope", po::value < std::string > (&mapAsImageOutScope), "Scope for sending the map as compressed image to a remote spread daemon")
    ("sigma_xy", po::value < double > (&sigma_xy_), "XY uncertainty for marcov localization [mm]")
    ("sigma_theta", po::value < double > (&sigma_theta_), "Theta uncertainty for marcov localization [rad]")
    ("throttle_scans", po::value < int > (&throttle_scans_), "Only take every n'th scan")
    ("samples", po::value < int > (&samples), "Sampling steps of the marcov localization sampler")
    ("hole_width", po::value < double > (&hole_width_), "Width of impacting rays [m]")
    ("delta", po::value < float > (&delta_), "Resolution [m/pixel]")
    ("rayPruningAngleDegree", po::value < float > (&rayPruningAngleDegree), "Pruning of adjiacent rays if they differ to much on the impacting surface [0° .. 90°]")
    ("loadMapFromImage", po::value < std::string > (&mapImagePath),"Load map from image file")
    ("loadMapFromPGM", po::value < std::string > (&mapPGMPath),"Load map from *16bit* PGM file")
    ("offsetX", po::value < float > (&offset.x),"offset x (mm)")
    ("offsetY", po::value < float > (&offset.y),"offset y (mm)")
    ("offsetTheta", po::value < float > (&offset.theta),"offset theta (mm)")
    ("setPositionScope", po::value < std::string > (&setPositionScope), "Scope for receiving a new position")
    ("setPositionUseVec", po::bool_switch(&setPositionUseVec), "If given it's assumed that received positions are of type vector<float> (instead of rst::geometry::Pose).")
    ("positionOutScope", po::value < std::string > (&positionOutScope), "Scope for publishing current position")
    ("publishPoseEuler", po::bool_switch(&publishPoseEuler), "Publish pose as rst::geometry::PoseEuler, instead of rst::geometry::Pose.")
    ("pathOutScope", po::value < std::string > (&pathOutScope), "Scope for publishing navigation path")
    ("sigma_xy_new_position", po::value < double > (&sigma_xy_new_position), "XY uncertainty for marcov localization after new position was set (mm)")
    ("sigma_theta_new_position", po::value < double > (&sigma_theta_new_position), "Theta uncertainty for marcov localization after new position was set (rad)")
    ("loadMapWithValidPositionsFromPNG", po::value < std::string > (&mapValidPositionsPNGPath),"Load map with valid positions from grayscale PNG file")
    ("tsMapSize", po::value < std::size_t > (&tsMapSize),"Size of the map in pixel")
    ("tsMapSizeMaxVisualization", po::value < std::size_t > (&tsMapSizeMaxVisualization),"Maximum size of the map in pixel for visualization")
    ("doMapUpdate", po::value < bool > (&doMapUpdate),"Update the map (false = only localization, default = true)")
    ("flipHorizontal", po::value < bool > (&flipHorizontal), "Flip all images read by loadMapFromImage horizontally")
    ("initialX", po::value < double > (&initialX), "Initial odometry")
    ("initialY", po::value < double > (&initialY), "Initial odometry")
    ("initialTheta", po::value < double > (&initialTheta), "Initial odometry")
    ("erosionRadius", po::value< float > (&erosionRadius), "Erosion radius for obstacle map (m)")
    ("precomputeOccupancyMap", po::value< bool > (&precomputeOccupancyMap), "Precompute occupancy map at start.")
    ("loadOccupancyMapFromFile", po::value< std::string > (&loadOccupancyMapFromFile), "Occupancy map file be read from file.")
    ("targetPose", po::value< std::vector<std::string> > (&targetPoseStrVec)->multitoken(), "Target position (mm, mm, degree) the amiro drives to when triggered.")
    ("targetPoseInScope", po::value< std::string > (&targetPoseInScope), "Scope for receiving a new goal position.")
    ("targetSpeed0", po::value< float > (&targetSpeed0), "")
    ("targetSpeed1", po::value< float > (&targetSpeed1), "");


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

  // tinySLAM init
  ts_map_set_scale(MM_TO_METERS/delta_);  // Set TS_MAP_SCALE at runtime

  // initalize map
  if (!mapImagePath.empty()) {
      INFO_MSG("Reading map from file.");
      bool ret = loadMapFromImage(&ts_map_, mapImagePath, flipHorizontal);
      if (!ret) {
          DEBUG_MSG("Map could not be loaded.");
          return 1;
      }
  } else if (!mapPGMPath.empty()) {
      INFO_MSG("Reading map from PGM.");
      bool ret = loadMapFromPGM(&ts_map_, mapPGMPath);
      if (!ret) {
          DEBUG_MSG("Map could not be loaded.");
          return 1;
      }
  } else {
      ts_map_init(&ts_map_, tsMapSize);
  }

  // if no explicit start positoin was given, set it to center of map
  if (!vm.count("initialX") && !vm.count("initialY") && !vm.count("initialTheta")) {
      initialX = ts_map_.size / 2 * delta_ * METERS_TO_MM;
      initialY = ts_map_.size / 2 * delta_ * METERS_TO_MM;
      initialTheta = 0;
  }

  // validate targetPose and set to center of map if not valid
  std::stringstream ss;
  std::copy(targetPoseStrVec.begin(), targetPoseStrVec.end(), std::ostream_iterator<std::string>(ss," "));

  std::vector<float> targetPoseVec;
  for (float f; ss >> f;) {
      targetPoseVec.push_back(f);
  }

  DEBUG_MSG("targetPoseVec: " << targetPoseVec);

  if (!vm.count("targetPose") || targetPoseVec.size() < 3) {
      INFO_MSG("No target pose given, setting to center.");
      targetPose = { ts_map_.size / 2 * delta_ * METERS_TO_MM, ts_map_.size / 2 * delta_ * METERS_TO_MM, 0 };
  } else {
      INFO_MSG("Setting target pose to " << targetPoseVec);
      targetPose.x = targetPoseVec.at(0);
      targetPose.y = targetPoseVec.at(1);
      targetPose.theta = targetPoseVec.at(2);
  }

  // Handle options for occupancy map
  if (!loadOccupancyMapFromFile.empty()) {
      INFO_MSG("Loading occupancy map from file");
      occupancyMap = cv::imread(loadOccupancyMapFromFile, CV_LOAD_IMAGE_GRAYSCALE);
      if (!occupancyMap.data) {
          ERROR_MSG("Loading occupancy map failed!");
          return 1;
      } else if (occupancyMap.cols != ts_map_.size || occupancyMap.rows != ts_map_.size) {
          ERROR_MSG("Size of the occupancy map does not match map size!");
          return 1;
      }

      if (flipHorizontal) {
          cv::flip(occupancyMap, occupancyMap, 0); // 0 is OpenCV's magic flip code for flipping around x axis
      }
  } else if (precomputeOccupancyMap) {
      INFO_MSG("Precomputing occupancy map now...");
      occupancyMap = getOccupancyMap();
      INFO_MSG("Done precomputing occupancy map.");
  }

  rsb::Factory& factory = rsb::getFactory();

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
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler> > converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter< rst::navigation::Path > > pathConverter(new rsb::converter::ProtocolBufferConverter< rst::navigation::Path >());
  rsb::converter::converterRepository<std::string>()->registerConverter(pathConverter);
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);
  if (publishPoseEuler) {
      boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler> > converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler>());
      rsb::converter::converterRepository<std::string>()->registerConverter(converter);
  }

  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
  // Prepare RSB async listener for odometry messages
  rsb::ListenerPtr listener = factory.createListener(odomInScope);
  listener->addHandler(HandlerPtr(new DataFunctionHandler<rst::geometry::Pose> (&storeOdomData)));

  // Local informer
  targetInformer = factory.createInformer<rst::geometry::TargetPoseEuler> (targetPoseOutScope);
  emergencyStopSwitchInformer = factory.createInformer<std::string>("/following");
  motorInformer = factory.createInformer<std::vector<int>>("/motor");

  // Prepare RSB informer for sending the map as an compressed image
  informer = factory.createInformer<std::string> (mapAsImageOutScope);

  // Listener for setting re-init
  rsb::ListenerPtr setPositionListener = factory.createListener(setPositionScope, tmpPartConf);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<float>>> >setPositionQueueVec(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<float>>>(1));
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>> >setPositionQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
  if (setPositionUseVec) {
    setPositionListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<float>>(setPositionQueueVec)));
  } else {
    setPositionListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(setPositionQueue)));
  }

  rsb::ListenerPtr targetPoseListener = factory.createListener(targetPoseInScope, tmpPartConf);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>> >targetPoseQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
  targetPoseListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(targetPoseQueue)));

  rsb::Informer<rst::geometry::Pose>::Ptr poseInfomer = factory.createInformer<rst::geometry::Pose>(positionOutScope, tmpPartConf);
  rsb::Informer<rst::geometry::PoseEuler>::Ptr poseEulerInfomer = factory.createInformer<rst::geometry::PoseEuler>(positionOutScope, tmpPartConf);

  pathInformer = factory.createInformer<rst::navigation::Path>(pathOutScope, tmpPartConf);

  // RSB listener for disabling/enabling sending the map
  rsb::ListenerPtr sendMapSwitchListener = factory.createListener(sendMapSwitchScope);
  sendMapSwitchListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&setSendMapAsCompressedImage)));

  // Prepare RSB listener for homing and response informer
  rsb::ListenerPtr homingListener = factory.createListener(homingInScope);
  homingListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&homingRequestHandler)));
  homingListener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&abortHomingRequestHandler)));
  std::string homingOutScope = homingInScope + "/response";
  homingInformer = factory.createInformer<std::string>(homingOutScope);

  // Prepare listener for emergency halts
  emergencyHaltListener = factory.createListener(emergencyHaltInScope);
  emergencyHaltQueue = boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  emergencyHaltListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(emergencyHaltQueue)));

  // Prepare RSB listener for saving maps
  rsb::ListenerPtr saveMapListener = factory.createListener(saveMapInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> saveMapQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  saveMapListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(saveMapQueue)));

  // ==== ====

  // Show the map as a cv Image
  const std::size_t maxVis = std::min(std::size_t(ts_map_.size), tsMapSizeMaxVisualization);
  debugImageSize = cv::Size(maxVis, maxVis);
  cv::Mat dst(debugImageSize, CV_16S); // destination image for scaling
  cv::Mat dstColor(debugImageSize, CV_8UC3); // Color image

  rst::vision::LocatedLaserScan scan;

  while( true ){
    // Fetch a new scan and store it to scan
    if (convertDataToScan(lidarQueue->pop(), scan))
      continue;
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
        DEBUG_MSG("scan processed.");

        // Save home position after first scan was processed
        if (!savedHomePose) {
            homePose = pose;
            savedHomePose = true;
        }
      }
    }

    if (!saveMapQueue->empty()) {
        INFO_MSG("Received request to save map as PGM")
        std::string path = *(saveMapQueue->pop());
        saveMapAsPGM(&ts_map_, path);
    }

    if (!setPositionQueue->empty() || !setPositionQueueVec->empty()) {
        INFO_MSG("Received setPosition request");

        if (setPositionUseVec) {
            boost::shared_ptr< std::vector<float> > newPosition = boost::static_pointer_cast< std::vector<float> >(setPositionQueueVec->pop());
            DEBUG_MSG("New position is: " << *newPosition);
            // Convert from debug map pixel to tiny slam position
            state_.position.x = newPosition->at(0) * (ts_map_.size / (float)debugImageSize.width) * delta_ * METERS_TO_MM;
            state_.position.y = newPosition->at(1) * (ts_map_.size / (float)debugImageSize.height) * delta_ * METERS_TO_MM;
            state_.position.theta = newPosition->at(2) * 180.0f / M_PI;
        } else {
            boost::shared_ptr< rst::geometry::Pose > newPosition = boost::static_pointer_cast< rst::geometry::Pose >(setPositionQueue->pop());
            DEBUG_MSG("New position is: " << newPosition->DebugString());

            state_.position = convertRSBPoseToTsPose(newPosition);
        }

        DEBUG_MSG("translated to ts_position: " << state_.position.x << " " << state_.position.y << " " << state_.position.theta);
        std::ofstream f;
        f.open("currentPose.txt");
        f << state_.position.x << " " << state_.position.y << " " << state_.position.theta << std::endl;
        f.close();

        // Set higher sigma to make it possible to converge into right position
        state_.sigma_theta = sigma_theta_new_position;
        state_.sigma_xy = sigma_xy_new_position;
    } else {
        // Decrease sigma over time
        if (state_.sigma_theta != sigma_theta_) {
            state_.sigma_theta = max(sigma_theta_, sigma_decrease_rate * state_.sigma_theta);
        }

        if (state_.sigma_xy != sigma_xy_) {
            state_.sigma_xy = max(sigma_xy_, sigma_decrease_rate * state_.sigma_xy);
        }
    }

    if (!targetPoseQueue->empty()) {
        INFO_MSG("Received targetPose request");
        boost::shared_ptr< rst::geometry::Pose > newPosition = boost::static_pointer_cast< rst::geometry::Pose >(targetPoseQueue->pop());
        targetPose = convertRSBPoseToTsPose(newPosition);
        DEBUG_MSG("translated to ts_position: " << targetPose.x << " " << targetPose.y << " " << targetPose.theta);
        std::ofstream f;
        f.open("targetPose.txt");
        f << targetPose.x << " " << targetPose.y << " " << targetPose.theta << std::endl;
        f.close();
    }

    //DEBUG_MSG("current sigma_xy: " << state_.sigma_xy);
    //DEBUG_MSG("current sigma_theta: " << state_.sigma_theta);

    // Publish estimated pose
    if (publishPoseEuler) {
        boost::shared_ptr<rst::geometry::PoseEuler> rsbPose(new rst::geometry::PoseEuler);
        rsbPose->mutable_translation()->set_x(pose.x * MM_TO_METERS);
        rsbPose->mutable_translation()->set_y(pose.y * MM_TO_METERS);
        rsbPose->mutable_translation()->set_z(0);

        rsbPose->mutable_rotation()->set_yaw(pose.theta * M_PI / 180.0f);
        rsbPose->mutable_rotation()->set_pitch(0);
        rsbPose->mutable_rotation()->set_roll(0);

        poseEulerInfomer->publish(rsbPose);
    } else {
        boost::shared_ptr<rst::geometry::Pose> rsbPose(new rst::geometry::Pose);
        rsbPose->mutable_translation()->set_x(pose.x * MM_TO_METERS);
        rsbPose->mutable_translation()->set_y(pose.y * MM_TO_METERS);
        rsbPose->mutable_translation()->set_z(0);

        Eigen::Matrix<double,3,1> rpy;
        rpy(0) = 0;
        rpy(1) = 0;
        rpy(2) = pose.theta * M_PI / 180.0f;
        Eigen::Quaternion<double> quat;
        ::conversion::euler2quaternion(&rpy, &quat);

        rsbPose->mutable_rotation()->set_qx(quat.x());
        rsbPose->mutable_rotation()->set_qy(quat.y());
        rsbPose->mutable_rotation()->set_qz(quat.z());
        rsbPose->mutable_rotation()->set_qw(quat.w());

        poseInfomer->publish(rsbPose);
    }

    if (sendMapAsCompressedImage) {

      // Convert to RGB image
      cv::Mat image = cv::Mat(ts_map_.size, ts_map_.size, CV_16U, static_cast<void*>(&ts_map_.map[0]));
      cv::resize(image,dst,dst.size());//resize image
      dst.convertTo(dst, CV_8U, 0.00390625);  // Convert to 8bit depth image
      cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2RGB, 3);  // Convert to color image

      // Draw home position
      cv::Point targetPosition(homePose.x * MM_TO_METERS / delta_ * debugImageSize.width / ts_map_.size,
                               homePose.y * MM_TO_METERS / delta_ * debugImageSize.height / ts_map_.size);
      cv::circle( dstColor, targetPosition, 0, cv::Scalar(pow(2,8)-1), 10, 8 );

      // Draw tinySLAM position
      cv::Point robotPosition(pose.x * MM_TO_METERS / delta_ * debugImageSize.width / ts_map_.size,
                              pose.y * MM_TO_METERS / delta_ * debugImageSize.height / ts_map_.size);  // Draw MCMC position
      cv::circle( dstColor, robotPosition, 0, cv::Scalar( 0, 0, pow(2,8)-1), 10, 8 );

      cv::line( dstColor, robotPosition, cv::Point(robotPosition.x + (cos(pose.theta * M_PI/180) * 20),
                                                   robotPosition.y + (sin(pose.theta * M_PI/180) * 20) ), cv::Scalar( 0, 0, pow(2,8)-1));

      // Draw odom position
      cv::Point robotOdomPosition(odom_pose.x * MM_TO_METERS / delta_ * debugImageSize.width / ts_map_.size,
                                  odom_pose.y * MM_TO_METERS / delta_ * debugImageSize.height / ts_map_.size);  // Draw odometry
      cv::circle( dstColor, robotOdomPosition, 0, cv::Scalar( 0, pow(2,8)-1), 0, 10, 8 );

      // Draw waypoints
      mtxPathToDraw.lock();
      if (!pathToDraw.empty()) {
          cv::Point2i p0 = robotPosition;
          cv::Point2i p1;
          for (auto p = pathToDraw.begin(); p != pathToDraw.end(); p++) {
              p1 = cv::Point(p->x * debugImageSize.width / ts_map_.size, p->y * debugImageSize.height / ts_map_.size);
              cv::line(dstColor, p0, p1, cv::Scalar(255,0,0));
              p0 = p1;
          }
      }
      mtxPathToDraw.unlock();

      // Send the map as image
      std::vector<uchar> buf;
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(85/*g_uiQuality [ 0 .. 100]*/);
      imencode(".jpg", dstColor, buf, compression_params);

      // Send the data.
      rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
      informer->publish(frameJpg);

      // Optionally directly show image with openCV (when on host machine)
      #ifndef __arm__
      cv::imshow("input", dstColor);
      cv::waitKey(1);
      #endif
    }

//    DEBUG_MSG( "--------------------------------------------")
//    DEBUG_MSG( "Pose TinySLAM: " << pose.x << ", " << pose.y << ", " << pose.theta)
//    DEBUG_MSG( "Pose Odometry: " << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta)
//    DEBUG_MSG( "Target pose: " << homePose.x << ", " << homePose.y << ", " << homePose.theta)
//    DEBUG_MSG( "------------ END OF MAIN LOOP --------------");
  }

  return 0;
}
