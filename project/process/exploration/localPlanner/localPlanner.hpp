/*
 * LocalPlanner.hpp
 */

#ifndef LOCALPLANNER_HPP_
#define LOCALPLANNER_HPP_

#include <opencv2/core/core.hpp>
#include <rsb/Informer.h>
#include <types/twbTracking.pb.h>
#include <ControllerAreaNetwork.h>

enum PATH_STATUS {
	PATH_FINISHED, PATH_ERROR, PATH_INPROGRESS,
};

class LocalPlanner {
public:
	LocalPlanner();
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
	void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN);

	static const float cellSize;
	static const int maxV;
	static const int maxW;

	const boost::shared_ptr<std::vector<int>> vecSteering_;
	std::vector<cv::Point2f> path_;
	ControllerAreaNetwork myCAN;
};

#endif /* LOCALPLANNER_HPP_ */
