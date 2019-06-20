#include <log4cxx/logger.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class Driver {
public:
    Driver() {
	}
    virtual ~Driver() {
	}

    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr startDriver(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    std::vector<float> pointCloudTo2DScan(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudTo2DScan2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudTo2DScanPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOnPlane);
};
