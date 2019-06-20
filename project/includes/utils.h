#ifndef UTILS_H
#define UTILS_H
#endif // UTILS_H nedds to be here, because it strongly depends on what you have included before

// RSB
#if defined RSB_EXPORT
#include <rsb/MetaData.h>
#endif

#include <math.h>
#include <algorithm>
#include <chrono>  // c++11
#include <Constants.h>
// using namespace claas::constants;
// using namespace claas::constants::mappingLayers;
// namespace claasNumeric = claas::constants::numeric;

namespace utils
{

namespace time {

// Usage:
//int main()
//{
//    Timer tmr;
//    double t = tmr.elapsed();
//    std::cout << t << std::endl;
//
//    tmr.reset();
//    t = tmr.elapsed();
//    std::cout << t << std::endl;
//    return 0;
//}
  class Timer
  {
  public:
      Timer() : beg_(clock_::now()) {}
      void reset() { beg_ = clock_::now(); }
      double elapsed() const {
          return std::chrono::duration_cast<second_>
              (clock_::now() - beg_).count(); }

  private:
      typedef std::chrono::high_resolution_clock clock_;
      typedef std::chrono::duration<double, std::ratio<1> > second_;
      std::chrono::time_point<clock_> beg_;
  };

#if defined ROS_TIME_H_INCLUDED && defined RSB_EXPORT
  inline ros::Time rsbCreateTime2ros(const rsb::EventPtr event) {
    const rsb::MetaData md = event->getMetaData();
    const boost::uint64_t time_ms = md.getCreateTime();
//    const msg.header.stamp.nsec = (time_ms % 1000) * 1000;
//    const msg.header.stamp.sec = time_ms / 1000;
    const boost::uint64_t msPerS = 1000000;
    const boost::uint64_t nsPerMs = 1000;
    return ros::Time(time_ms / msPerS, (time_ms % msPerS) * nsPerMs);
  }

  inline ros::Time getRosTimeFromRsbEvent(int useRosTime = 1, const rsb::EventPtr event = NULL) {
    if (useRosTime) {
        return ros::Time::now();
    } else if (event == NULL && useRosTime) {
        ROS_WARN("No RSB event given, using ROS time");
        return ros::Time::now();
    } else {
        return rsbCreateTime2ros(event);
    }
  }
#endif // defined ROS_TIME_H_INCLUDED && defined RSB_EXPORT
}

enum esitmatorId{
  isCropId = 1,
  isFloorId = 2,
  isWeedId = 3,
};

template<typename T>
inline T odds(T P) {
  return P / (1-P);
}

template<typename T>
inline T lOdds(T P) {
  return log(P) - log(1-P);
}

template<typename T>
inline T pFromLOdds(T lR) {
  return exp(lR) / (1.0 + exp(lR));
}

const int invertModelSize = 128;
const double invertModelResolution = 0.01; /*m*/
const double holWidthSize = 0.1; /*m*/

template<typename T>
inline T sumLOdds(T P1, T P2) {
  return lOdds(P1) + lOdds(P2);
}


namespace conversion
{
#ifdef CONVERSIONS_TF_EIGEN_H
  tf::Transform getTfFromEigen(Eigen::Matrix4d M){
    tf::Transform transform;
    tf::Matrix3x3 rotation;
    tf::Quaternion q;
    tf::Vector3 translation;
    tf::vectorEigenToTF(machine::tf::getTrans<double>(M), translation);
    transform.setOrigin(translation);
    tf::matrixEigenToTF (machine::tf::getRot<double>(M), rotation);
    rotation.getRotation(q);
    transform.setRotation(q);
    return transform;
  }
#endif

#ifdef EIGEN_GEOMETRY_MODULE_H
  const unsigned int NUMAXIS = 3;
  /**
    * @brief Conversion Quaternion to Euler angles
    *
    * Considering Heading along z axis, pitch allow y axis and roll along x axis.
    *
    * @author Timo Korthals
    *
    * @param[in] *q pointer to a quaternion vector.
    * @param[out] *euler pointer to double. The three euler angles Convention Roll, Pitch, Yaw
    *
    * @return void
    *
    */
    void quaternion2euler(Eigen::Quaternion< double >* q, Eigen::Matrix< double, NUMAXIS , 1  >* euler)
    {
      double sqx, sqy, sqz, sqw;

      /** Square terms **/
      sqx = pow (q->x(), 2);
      sqy = pow (q->y(), 2);
      sqz = pow (q->z(), 2);
      sqw = pow (q->w(), 2);

      (*euler)(0) = atan2 (2.0 * (q->y()*q->z() + q->x()*q->w()), (-sqx-sqy+sqz+sqw)); /** Roll **/
      (*euler)(1) = asin (-2.0 * (q->x()*q->z() - q->y()*q->w())/(sqx+sqy+sqz+sqw)); /** Pitch **/
      (*euler)(2) = atan2 (2.0 * (q->x()*q->y() + q->z()*q->w()), (sqx - sqy -sqz + sqw)); /** Yaw **/

      return;
    }

    // Get rotation around z-axis in radiant
    double quaternion2eulerYaw(double qw, double qx, double qy, double qz) {
      Eigen::Quaternion< double > q(qw,qx,qy,qz);
      Eigen::Matrix< double, 3 , 1  > euler;
      utils::conversion::quaternion2euler(&q, &euler);
      return euler(2);
    }

    // Rotate the point (x,y) around the angle alpha (in radiant)
    void rotXY(double &x, double &y, double alpha_rad) {
      double xN = x * cos(alpha_rad) - y * sin(alpha_rad);
      double yN = x * sin(alpha_rad) + y * cos(alpha_rad);
      x = xN;
      y = yN;
    }

    /**
    * @brief Conversion Euler angles to Quaternion
    *
    * Considering Heading along z axis, pitch allow y axis and roll along x axis.
    *
    * @author Timo Korthals
    *
    * @param[out] *euler pointer to double. The three euler angles, the convention is Roll, Pitch, Yaw
    * @param[in] *q pointer to a quaternion vector.
    *
    * @return void
    *
    */
    void euler2quaternion(Eigen::Matrix< double, NUMAXIS , 1  >* euler, Eigen::Quaternion< double >* q)
    {

      double c1 = cos((*euler)(2)/2);
      double s1 = sin((*euler)(2)/2);
      double c2 = cos((*euler)(1)/2);
      double s2 = sin((*euler)(1)/2);
      double c3 = cos((*euler)(0)/2);
      double s3 = sin((*euler)(0)/2);

      q->w() = (double) (c1*c2*c3 + s1*s2*s3);
      q->x() = (double) (c1*c2*s3 - s1*s2*c3);
      q->y() = (double) (c1*s2*c3 + s1*c2*s3);
      q->z() = (double) (s1*c2*c3 - c1*s2*s3);

      return;
    }

    Eigen::Quaterniond
rpy2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}
#endif // EIGEN_GEOMETRY_MODULE_H

#ifdef OPENCV_CORE_HAL_INTERFACE_H

std::size_t type2size(const int type) {
  const uchar depth = type & CV_MAT_DEPTH_MASK;

  switch ( depth ) {
    case CV_8U:
    case CV_8S:  return std::size_t(1); break;
    case CV_16U:
    case CV_16S: return std::size_t(2); break;
    case CV_32S:
    case CV_32F: return std::size_t(4); break;
    case CV_64F: return std::size_t(8); break;
    default:     return std::size_t(1);
  }
}

std::string format2str(const int type) {
  std::string r;

  const uchar depth = type & CV_MAT_DEPTH_MASK;

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  return r;
}

std::string type2str(const int type) {
  std::string r(format2str(type));

  const uchar chans = 1 + (type >> CV_CN_SHIFT);

  r += "C";
  r += (chans+'0');

  return r;
}

#endif // OPENCV_CORE_HAL_INTERFACE_H

}
#ifdef EIGEN_GEOMETRY_MODULE_H
namespace eigen {
  template <typename T>
  inline T getXTranslation(const Eigen::Matrix<T, 4, 4> &M) {
    return M(0,3);
  }

  template <typename T>
  inline T getYTranslation(const Eigen::Matrix<T, 4, 4> &M) {
    return M(1,3);
  }

  template <typename T>
  inline T getZTranslation(const Eigen::Matrix<T, 4, 4> &M) {
    return M(2,3);
  }

  template <typename T>
  inline T getXTranslation(const Eigen::Matrix<T, 4, 1> &M) {
    return M(0);
  }

  template <typename T>
  inline T getYTranslation(const Eigen::Matrix<T, 4, 1> &M) {
    return M(1);
  }

  template <typename T>
  inline T getZTranslation(const Eigen::Matrix<T, 4, 1> &M) {
    return M(2);
  }

  template <typename T>
  inline Eigen::Matrix<T, 4, 1> getPoint(const T x, const T y, const T z) {
    Eigen::Matrix<T, 4, 1>vector;
    vector << x, y, z, 1;
    return vector;
  }
}
#endif // EIGEN_GEOMETRY_MODULE_H

// 1D Gauss function
inline double draw1DGauss(double x, double mu, double sigma) {
  double tmp = pow(x - mu, 2);
  double exponent =  tmp / sigma;
  return 1 / (sqrt(2.0 * M_PI) * sigma) * exp(-0.5 * exponent);
}

#if defined __OPENCV_ALL_HPP__ || defined OPENCV_ALL_HPP
// 2D Gauss function
inline double draw2DGauss(cv::Mat x /*2x1 matrix*/, cv::Mat mu /*2x1 matrix*/, cv::Mat sigma /*2x2 matrix*/) {

  cv::Mat sigmaInv(2, 2, CV_64FC1);
  const double sigmaDet = determinant(sigma);
  sigmaInv.at<double>(0,0) = sigma.at<double>(1,1) / sigmaDet;
  sigmaInv.at<double>(1,0) = -sigma.at<double>(1,0) / sigmaDet;
  sigmaInv.at<double>(0,1) = -sigma.at<double>(0,1) / sigmaDet;
  sigmaInv.at<double>(1,1) = sigma.at<double>(0,0) / sigmaDet;

  cv::Mat tmp = x - mu;
  cv::Mat tmpT; cv::transpose(tmp, tmpT);
  cv::Mat exponent =  tmpT * sigmaInv * tmp;  // Should result in a 1x1 matrix
  return 1 / (2.0 * M_PI * sigmaDet) * exp(-0.5 * exponent.at<double>(0));
}
// Rotate image
cv::Mat rotateImage(const cv::Mat& source, cv::Point2f center, double angle, int flag = cv::INTER_NEAREST, int borderMode = cv::BORDER_CONSTANT, const cv::Scalar& borderValue = cv::Scalar())
{
  cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
  cv::Mat dst;
  cv::warpAffine(source, dst, rot_mat, source.size(), flag, borderMode, borderValue);
  return dst;
}

int rot90(cv::Mat &matImage, int rotflag){
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    transpose(matImage, matImage);
    flip(matImage, matImage,1); //transpose+flip(1)=CW
  } else if (rotflag == 2) {
    transpose(matImage, matImage);
    flip(matImage, matImage,0); //transpose+flip(0)=CCW
  } else if (rotflag ==3){
    flip(matImage, matImage,-1);    //flip(-1)=180
  } else if (rotflag != 0){ //if not 0,1,2,3:
    std::cerr  << "Unknown rotation flag(" << rotflag << ")" << std::endl;
    return -1;
  }
  return 0;
}

// Draw rotated rectangle
void drawRotatedRectInImage(cv::Mat &dst, const cv::RotatedRect rect, const cv::Scalar& color,
                            int thickness = 1, int lineType = cv::LINE_8, int shift = 0) {
  const int rectPoints = 4;
  cv::Point2f rect_points[rectPoints];
  rect.points( rect_points );
  for( int idx = 0; idx < rectPoints; idx++ ) {
    cv::line( dst, rect_points[idx], rect_points[(idx+1)%rectPoints], color, thickness, lineType, shift);
  }
}

void castCopyImage(const cv::Mat& src, cv::Mat& dst, int type)
{
//  if(src.size() != dst.size() || src.channels() != dst.channels() /* || src.type() != dst.type()*/){
//    throw std::runtime_error(std::string("castCopyImage: Failed"));
//  }
  cv::Mat tmp = cv::Mat(src.rows,
                        src.cols,
                        type,
                        (void*) src.data);
  tmp.copyTo(dst);
}

void changeImageType(cv::Mat& src, int type)
{
  src = cv::Mat(src.rows,
                src.cols,
                type,
                (void*) src.data);
}

#ifdef RSB_EXPORT
inline void sendImage (const rsb::Informer<std::string>::Ptr informer, cv::Mat img)
{
  std::vector<uchar> buf;
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
  cv::imencode(".jpg", img, buf, compression_params);

  rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
  informer->publish(frameJpg);
}
#endif


void addLocalToGlobalMap(cv::Mat& localMap, cv::Mat& globalMap, cv::Point2f robotPosition) {
  double cellSize = invertModelResolution;
  // Get the position of the local map in the global map (Left upper corner).
  // The robot is in the middle of the local map.
  int localMapX = (int) (robotPosition.x / cellSize - localMap.cols / 2.0 + globalMap.cols / 2.0);
  int localMapY = (int) (robotPosition.y / cellSize - localMap.rows / 2.0 + globalMap.rows / 2.0);

  // check if the localMap is positioned in the global map
  if (localMapX > -localMap.cols && localMapX < globalMap.cols && localMapY > -localMap.rows
      && localMapY < globalMap.rows) {
    // determine the size of the overlap
    cv::Size s(std::min(std::min( localMap.cols, localMap.cols + localMapX), globalMap.cols - localMapX ),
        std::min(std::min( localMap.rows, localMap.rows + localMapY) , globalMap.rows - localMapY ));

    // Add the local to the global map. Note: OpenCV automatically will clip the values in the global map between -128 and 127.
    cv::Mat globalMapTmp = globalMap(cv::Rect(cv::Point(std::max(localMapX, 0), std::max(localMapY, 0)), s));
    cv::Mat localMapTmp = localMap(cv::Rect(cv::Point(std::max(0, -localMapX), std::max(0, -localMapY)), s));
    for (int idxy = 0; idxy < globalMapTmp.cols - 1; idxy++) {
      for (int idxx = 0; idxx < globalMapTmp.rows - 1; idxx++) {
//        for (int channel = 0; channel < 3; channel++) {
//          std::cout << "d :" << globalMapTmp.depth() << std::endl << std::flush;
//          std::cout << "ch :" << globalMapTmp.channels() << std::endl << std::flush;
//          globalMapTmp.at<uchar>(idxy, idxx)  = 0;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<cv::Vec3b>(idxx, idxy)  = cv::Vec3b(255, 0, 0);
//          globalMapTmp.at<uchar>(512, 513)  = 255;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<uchar>(512, 514)  = 255;//;cv::Scalar(255, 255, 255);
//          globalMapTmp.at<uchar>(512, 515)  = 255;//;cv::Scalar(255, 255, 255);
//          cv::Vec3b g = globalMapTmp.at<cv::Vec3b>(idxx, idxy);
//          cv::Vec3b l = localMapTmp.at<cv::Vec3b>(idxx, idxy);

//          globalMapTmp.at<cv::Vec3b>(idxx, idxy) = cv::Vec3b( uchar(pFromLOdds(sumLOdds(double(g[0]) / 255.0, double(l[0]) / 255.0) * 255.0)),
//                                                              uchar(pFromLOdds(sumLOdds(double(g[1]) / 255.0, double(l[1]) / 255.0) * 255.0)),
//                                                              uchar(pFromLOdds(sumLOdds(double(g[2]) / 255.0, double(l[2]) / 255.0) * 255.0)));
          globalMapTmp.at<cv::Vec3b>(idxx, idxy) = cv::Vec3b( uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0),
                                                              uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0),
                                                              uchar(pFromLOdds(sumLOdds(0.5,0.5)) * 255.0));
//              pFromLOdds(sumLOdds(localMapTmp.at<double>(idxy, idxx) / 255.0, globalMapTmp.at<double>(idxy, idxx) / 255.0)) * 250.0;
//        }
      }
    }
    std::cout << "P"<< pFromLOdds(sumLOdds(0.5,0.5)) << std::endl;
              std::cout << "R"<< sumLOdds(0.5,0.5) << std::endl;
//    globalMapTmp.at<uchar>(512, 512)  = cv::Scalar(255, 255, 255);
//    globalMap(cv::Rect(cv::Point(max(localMapX, 0), max(localMapY, 0)), s)) += localMap(
//        cv::Rect(cv::Point(max(0, -localMapX), max(0, -localMapY)), s));

//    std::cout << s << std::endl;
  }
}


#ifdef CVPLOT_H
#ifdef RSB_EXPORT
template<typename T>
inline void sendPlot (const std::string figure_name, const rsb::Informer<std::string>::Ptr informer, const T* p, int count, int step, int R, int G, int B)
{
  if(count == 0 || p == NULL || informer == NULL)
    return;

  IplImage * plot = CvPlot::plot(figure_name, p, count, step, R, G, B);
  sendImage(informer, cv::Mat(plot));
  CvPlot::clear(figure_name);
  cvReleaseImage(&plot);
}
#endif // RSB_EXPORT
#endif // CVPLOT_H


/*
 * Shrinks a rect to fit inside image bounds
 */
void constrainRectToImageBounds(cv::Mat img,cv::Rect& rect){
  if(rect.x<0){
    rect.width+=rect.x;
    rect.x=0;
  }
  if(rect.y<0){
    rect.height+=rect.y;
    rect.y=0;
  }
  if(rect.x+rect.width>img.cols){
    rect.width=img.cols-rect.x;
  }
  if(rect.y+rect.height>img.rows){
    rect.height=img.rows-rect.y;
  }
}


/*
* cutView: Requested view in the machineRoi frame
*          Returns always CV_32F
*
*/
cv::RotatedRect cutView(const cv::Mat &src, cv::Mat &dst, const double srcResolution,
                        const double xTranslation_m, const double yTranslation_m, const double width_m, const double height_m, const double zRotation_rad,
                        const Eigen::Matrix4d &roi_machineRoi, int targetFormat = int(CV_32F)){
  cv::RotatedRect rectView;
  const double xView = xTranslation_m; // Meter
  const double yView = yTranslation_m; // Meter
  const double wView = width_m < srcResolution ? srcResolution : width_m; // Meter
  const double hView = height_m < srcResolution ? srcResolution : height_m; // Meter
  const double zRotView = zRotation_rad; // rad

  Eigen::Matrix4d machineRoi_view = machine::tf::trans<double>(xView, yView, 0.0) * machine::tf::rotZ<double>(zRotView);
  Eigen::Matrix4d roi_view = roi_machineRoi * machineRoi_view;
  Eigen::Matrix4d roiOrigin_view = machine::frames::roi_roiOrigin.inverse() * roi_view;
  double xViewOrig = utils::eigen::getXTranslation<double>(roiOrigin_view);
  double yViewOrig = utils::eigen::getYTranslation<double>(roiOrigin_view);
  // Get the rotation of the view with respect to the ROI-KS (middle of the image)
  const Eigen::Matrix<double, 4, 1> wViewOrig = roiOrigin_view * utils::eigen::getPoint(0.0,wView,0.0);
  double wXViewOrig = utils::eigen::getXTranslation<double>(wViewOrig);
  double wYViewOrig = utils::eigen::getYTranslation<double> (wViewOrig);
  double rotationOfView_roi = atan2(wYViewOrig - yViewOrig, wXViewOrig - xViewOrig);

  const double x_px = xViewOrig / srcResolution;
  const double y_px = yViewOrig / srcResolution;
  const double w_px = wView / srcResolution;
  const double h_px = hView / srcResolution;
  const float angle_rad = rotationOfView_roi;

//  DEBUG_MSG("roiOrigin_view: " << roiOrigin_view);
//  DEBUG_MSG("xViewOrig: " << xViewOrig);
//  DEBUG_MSG("yViewOrig: " << yViewOrig);
//  DEBUG_MSG("x: " << x_px);
//  DEBUG_MSG("y: " << y_px);
//  DEBUG_MSG("w: " << w_px);
//  DEBUG_MSG("h: " << h_px);
//  DEBUG_MSG("angle: " << angle_rad * rad2deg);

  // Get the center of the rectangular in the originRoi frame
  const Eigen::Matrix<double, 4, 1> rectCenter_roiOrigin = roiOrigin_view * utils::eigen::getPoint(hView/2, wView/2,0.0);
  const double rectCenterX_px = utils::eigen::getXTranslation<double>(rectCenter_roiOrigin) / srcResolution;
  const double rectCenterY_px = utils::eigen::getYTranslation<double>(rectCenter_roiOrigin) / srcResolution;
  // Get the requested view
  rectView = cv::RotatedRect(cv::Point2f(rectCenterX_px, rectCenterY_px), cv::Size2f(w_px,h_px), angle_rad * rad2deg /*- (M_PI / 2.0f * rad2deg)*/);

  cv::Mat srcConverted;
  src.convertTo(srcConverted, targetFormat == src.type() ? int(-1) : targetFormat);

   // crop to roi
  cv::Rect boundingRect=rectView.boundingRect();
  constrainRectToImageBounds(srcConverted,boundingRect);
  cv::Mat croppedROI=srcConverted(boundingRect);

  // perform the affine transformation
  cv::Scalar boarderExtrapolationScalar;
  switch (src.type()) {
    case CV_32SC1: boarderExtrapolationScalar = cv::Scalar_<int32_t>(claasNumeric::invalidValue_int32); break;
    case CV_16SC1: boarderExtrapolationScalar = cv::Scalar_<int16_t>(claasNumeric::invalidValue_int16); break;
    case CV_8SC1:  boarderExtrapolationScalar = cv::Scalar_<int8_t>(claasNumeric::invalidValue_int8); break;
    case CV_64FC1: boarderExtrapolationScalar = cv::Scalar_<double>(claasNumeric::invalidValue_double); break;
    case CV_32FC1: boarderExtrapolationScalar = cv::Scalar_<float>(claasNumeric::invalidValue_float); break;
    default: throw std::runtime_error(std::string("cutView: Unsupported source type")); break;
  }
  // get angle and size from the bounding box
  float angle = rectView.angle;

  cv::Point2f center((float)rectView.center.x-boundingRect.x,(float)rectView.center.y-boundingRect.y);

  cv::Mat warpM = cv::getRotationMatrix2D(center, angle, 1.0);

  cv::Rect dstSize = cv::Rect(0,0,w_px,h_px);
  // adjust transformation matrix
  warpM.at<double>(0,2) += dstSize.width/2.0 - center.x;
  warpM.at<double>(1,2) += dstSize.height/2.0 - center.y;

  cv::warpAffine(croppedROI, dst, warpM, dstSize.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, boarderExtrapolationScalar);

  return rectView;
}


#endif // __OPENCV_ALL_HPP__ or OPENCV_ALL_HPP

// ML estimator for the lidar
void getLidarEstimations(utils::esitmatorId id, const float *scan_values ,std::vector<unsigned char> &isPresent, int size) {
  // ML estimator for the different {Weed, Crop, Floor} features /////
  double muWeed  = 0.0627; /*m*/
  double muCrop  = 0.07277; /*m*/
  double muFloor = 0.116286; /*m*/
  double sigmaLidar = 0.02; /*m²:  s.t. noise of the lidar*/

  for(int idx = 0; idx < size; ++idx) {
    double nWeed = draw1DGauss(scan_values[idx], muWeed, sigmaLidar);
    double nCrop = draw1DGauss(scan_values[idx], muCrop, sigmaLidar);
    double nFloor = draw1DGauss(scan_values[idx], muFloor, sigmaLidar);
    if (nWeed > nCrop /* && nWeed > nFloor */){
      isPresent[idx] = id == utils::isWeedId ? true : false;
    } else if (nCrop > nFloor) {
      isPresent[idx] = id == utils::isCropId ? true : false;
    } else /*nWeed < nFloor*/ {
      isPresent[idx] = id == utils::isFloorId ? true : false;
    }
  }

}

namespace filesystemIO {

#ifdef BOOST_FILESYSTEM_FILESYSTEM_HPP

namespace fs = boost::filesystem;

/**
* @brief Get the list of files inside a given folder
*
*
* @author Timo Korthals
*
* @param[out] &filelilst List of files inside the folder
* @param[in] &inputPath Systempath.
*
* @return 0 if OK, 1 if error occures
*
*/
int fileList(const std::string &inputPath, std::vector<std::string> &filelist, bool withPathExtension = true) {
  int file_count = 0, err_count = 0;
  fs::path fsInputPath(fs::system_complete(fs::path(inputPath)));

  // Flush the input
  filelist.clear();

  if ( !fs::exists( fsInputPath ) ) {
    std::cout << "\nNot found: " << fsInputPath << std::endl;
    return -1;
  }
  if ( fs::is_directory( fsInputPath ) ) {
    std::cout << "\nIn directory: " << fsInputPath << "\n\n";
    fs::directory_iterator end_iter;
    for ( fs::directory_iterator dir_itr( fsInputPath ); dir_itr != end_iter; ++dir_itr ) {
      try{
        if ( fs::is_regular_file( dir_itr->status() ) ) {
          ++file_count;
          std::cout << dir_itr->path().filename() << "\n";
          if (withPathExtension) {
            filelist.push_back(std::string(std::string(fsInputPath.c_str()) + std::string("/") + std::string(dir_itr->path().filename().c_str())));
          } else {
            filelist.push_back(std::string(dir_itr->path().filename().c_str()));
          }

        }
      }
      catch ( const std::exception & ex ) {
        ++err_count;
        std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
      }
    }
    std::cout << "\n" << file_count << " files\n"
              << err_count << " errors\n";
  }
  else // must be a file
  {
    std::cout << "\nFound: " << fsInputPath << "\n";
    return -1;
  }
  return file_count;
}

/**
* @brief Get a digit denoted by pre- and suffix from a string
*
*
* @author Timo Korthals
*
* @param[out] &filelilst List of files inside the folder
* @param[in] &inputPath String containing digit.
* @param[in] &prefix prefix as regular expression
* @param[in] &suffix suffix as regular expression
* @param[in] &suffix suffix as regular expression
*
* @return 0 if OK, 1 if string was empty
*
*/
int getDigitFromFileName(const std::string &input, const std::string &prefix, const std::string &suffix, double &digit) {
  const std::string realNumberString("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?");
  const std::regex realNumberRegEx(realNumberString);
  const std::regex realNumberXRegEx(prefix + realNumberString + suffix);
  std::smatch matchWithPreAndSuffix, matchDigit;
  // Get the first full match with prefix and suffix (all the rest are submatches)
  std::regex_search ( input, matchWithPreAndSuffix, realNumberXRegEx);
  const std::string digitWithPreAndSuffix(matchWithPreAndSuffix[0]);
  // Get the digit
  std::regex_search ( digitWithPreAndSuffix, matchDigit, realNumberRegEx);
  const std::string digitString(matchDigit[0]);
//  std::cout << "Number (String): " << digitString << std::endl;
  if (digitString.size() == 0) {
    return 1;
  }
  digit = std::stod(digitString);
//  std::cout << "Number (double): " << digit << std::endl;
  return 0;
}

#endif

}

}
