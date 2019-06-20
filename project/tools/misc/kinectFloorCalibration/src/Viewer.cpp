/*
 * Viewer.cpp
 * Author: caschebe
 */

#include <pcl/io/openni_grabber.h>

#include "Viewer.h"

void Viewer::cloud_cb_(const pcl::PointCloud<PointType>::ConstPtr &cloud) {
	if (!viewer.wasStopped()) {
		pcl::PointCloud<PointType>::ConstPtr result = analyzer->analyze(cloud);

		viewer.showCloud(result, "result");
	}
}

void Viewer::run() {
	pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> f =
			boost::bind(&Viewer::cloud_cb_, this, _1);

	interface->registerCallback(f);

	interface->start();

	while (!viewer.wasStopped() && !analyzer->finished) {
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	interface->stop();
}
