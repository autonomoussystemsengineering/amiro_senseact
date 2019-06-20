/* 
 * File:   
 * Author: ssharma
 * Description: 
 * 
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_grabber.h>

#include "Driver.h"

#include <stdlib.h>

class readKinect {
public:
	readKinect() :
    viewer("PCL OpenNI Viewer") {}
    pcl::visualization::CloudViewer viewer;
    Driver driver;
    

    //callback function
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
        if (!viewer.wasStopped()) 
	{
      	    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr outCloud;
	    outCloud = driver.startDriver(cloud);
            viewer.showCloud(outCloud);
            //this->process(cloud);
	}
    }

    void run()
    {
		//read file
		// for oni file uncomment code below
		pcl::Grabber* interface = new pcl::ONIGrabber("../../Captured.oni",
                        false, false);
		/*
		//for pcd file uncomment code below
		boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > interface;
		interface.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> ("../data/other/kitchen/Rf3.pcd", 1, true)); 			
		*/
		
		//register callback
		boost::function<
				void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
				boost::bind(&readKinect::cloud_cb_, this, _1);

		interface->registerCallback(f);
		interface->start();

		//start viewer
        while (!viewer.wasStopped())
        {
			interface->start();
            //boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
        PCL_INFO("Completed reading file.\n");
		interface->stop();
	}

	/*void process( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud )
	{
	  std::cout << "In function process" << std::endl;
	}*/
};

int main() {
    readKinect rK;
    rK.run();
    //Driver d;
    //d.startDriver();
	return 0;
}