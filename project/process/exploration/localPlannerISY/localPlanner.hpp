/*
 * LocalPlanner.hpp
 *
 *  Created on: Dec 14, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#ifndef LOCALPLANNER_HPP_
#define LOCALPLANNER_HPP_

#include <opencv2/core/core.hpp>
#include <rsb/Informer.h>
#include <types/twbTracking.pb.h>

enum PATH_STATUS {
	PATH_FINISHED, PATH_ERROR, PATH_INPROGRESS,
};

class LocalPlanner {
public:
	LocalPlanner(rsb::Informer<std::vector<int>>::Ptr steeringInformer, rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathUpdateInformer);
	PATH_STATUS updatePose(cv::Point3f robotPose);
	void setPath(std::vector<cv::Point2f> path);
	void setPath(twbTracking::proto::Pose2DList path);
	void stopRobot();
	std::vector<cv::Point2f> getPath();
	virtual ~LocalPlanner();

private:
	bool pointReached(cv::Point2f currentPose, cv::Point2f nextPose);
	void driveToPoint(cv::Point3f currentPose, cv::Point2f nextPose);
	bool setSteering(int v, int w);
	void publishPath();

	static const float cellSize;
	static const int maxV;
	static const int maxW;

	const rsb::Informer<std::vector<int>>::Ptr steeringInformer_;
	const rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathUpdateInformer_;
	const boost::shared_ptr<std::vector<int>> vecSteering_;
	std::vector<cv::Point2f> path_;
};

#endif /* LOCALPLANNER_HPP_ */
