#ifndef VIEWER_H_
#define VIEWER_H_

#include <pcl/visualization/cloud_viewer.h>

#include "Analyzer.h"

typedef pcl::PointXYZRGBA PointType;

class Viewer {
public:
	Viewer(Analyzer* analyzer) :
			viewer("RobocupKinectFloorCalibration"), analyzer(analyzer) {
	}

	void cloud_cb_(const pcl::PointCloud<PointType>::ConstPtr &cloud);

	void run();

	pcl::visualization::CloudViewer viewer;
	Analyzer* analyzer;
};

#endif /* VIEWER_H_ */
