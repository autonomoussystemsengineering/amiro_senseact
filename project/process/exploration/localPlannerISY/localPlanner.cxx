/*
 * LocalPlanner.cxx
 *
 *  Created on: Dec 14, 2014
 *      Author: florian
 */

#include "localPlanner.hpp"
#include <iostream>

using namespace std;

const float LocalPlanner::cellSize = 0.01f;
// maximum angular velocity
const int LocalPlanner::maxW = 600000;
// maximum velocity
const int LocalPlanner::maxV = 45000;

// constructor, steering commands will be published with the given informer
LocalPlanner::LocalPlanner(rsb::Informer<std::vector<int>>::Ptr steeringInformer,
		rsb::Informer<twbTracking::proto::Pose2DList>::Ptr pathUpdateInformer) :
		steeringInformer_(steeringInformer), pathUpdateInformer_(pathUpdateInformer), vecSteering_(
				new std::vector<int>(3)) {
	vecSteering_->at(2) = 500000000;
}

LocalPlanner::~LocalPlanner() {

}

// Update the robots pose.
// This will automatically trigger an adjustment of the robots steering towards the next cell on the given path.
PATH_STATUS LocalPlanner::updatePose(cv::Point3f robotPose) {
	cv::Point2f robotPose2f(robotPose.x, robotPose.y);

	bool pathChanged = false;

	// remove reached points from the path
	while (!path_.empty() && pointReached(robotPose2f, path_.back())) {
		path_.pop_back();
		pathChanged = true;
	}

	// if the path is empty stop the robot
	if (path_.empty()) {
		setSteering(0, 0);
		return PATH_FINISHED;
	}

	// set steering commands to approach the next pose on the path
	driveToPoint(robotPose, path_.back());

	if (pathChanged) {
		publishPath();
	}

	return PATH_INPROGRESS;
}

// stop the robot
void LocalPlanner::stopRobot() {
	setSteering(0, 0);
}

// calculate and publish a steering from currentPose to nextPose
void LocalPlanner::driveToPoint(cv::Point3f currentPose, cv::Point2f nextPose) {

	// calculate the pose difference
	float diffX = nextPose.x - currentPose.x;
	float diffY = nextPose.y - currentPose.y;
	float theta = currentPose.z;

	if (theta > M_PI) {
		theta -= 2.0 * M_PI;
	} else if (theta < -M_PI) {
		theta += 2.0 * M_PI;
	}

	// calculate the angular difference
	float angleDiff = (float) atan2(diffY, diffX) - theta;
	if (angleDiff > M_PI) {
		angleDiff -= 2.0 * M_PI;
	} else if (angleDiff < -M_PI) {
		angleDiff += 2.0 * M_PI;
	}

	int v = 0;
	int w = 0;

	// if the robot is oriented towards the cell, drive forward
	if (abs(angleDiff) < M_PI / 6.0) {
		float dist = sqrt(diffX * diffX + diffY * diffY);
		v = maxV;
		w = angleDiff > 0 ? 0.3*maxW : -0.3*maxW;
	} else {
		w = angleDiff > 0 ? maxW : -maxW;
	}

	// send the steering command
	setSteering(v, w);
}

// send new steering commands to the motorControl
// only send the new steering command if it differs from the last steering command
// returns true if a new command was send
bool LocalPlanner::setSteering(int v, int w) {
	if (v != vecSteering_->at(0) || w != vecSteering_->at(1)) {
		vecSteering_->at(0) = v;
		vecSteering_->at(1) = w;
		steeringInformer_->publish(vecSteering_);
		return true;
	} else {
		return false;
	}
}

// Adjust the robots path
// important note: the path is sorted like a stack, the last point (goal is the first element)
void LocalPlanner::setPath(std::vector<cv::Point2f> path) {
	path_ = path;
	publishPath();
}

// Adjust the robots path
// important note: the path is sorted like a stack, the last point (goal is the first element)
void LocalPlanner::setPath(twbTracking::proto::Pose2DList path) {
	path_.clear();
	for (int i = 0; i < path.pose_size(); i++) {
		cv::Point2f p(path.pose(i).x(), path.pose(i).y());
		path_.push_back(p);
	}
	publishPath();
}

// returns the current path
std::vector<cv::Point2f> LocalPlanner::getPath() {
	return path_;
}

// check whether the robot is within a certain distance of the next position
bool LocalPlanner::pointReached(cv::Point2f currentPose, cv::Point2f nextPose) {
	return cv::norm(currentPose - nextPose) < cellSize;
}

// publish the path for visualization
void LocalPlanner::publishPath() {
	rsb::Informer<twbTracking::proto::Pose2DList>::DataPtr pose2DList(new twbTracking::proto::Pose2DList);
	for (cv::Point2f p : path_) {
		twbTracking::proto::Pose2D *pose2D = pose2DList->add_pose();
		pose2D->set_x(p.x);
		pose2D->set_y(p.y);
		pose2D->set_orientation(0);
		pose2D->set_id(0);
	}
	pathUpdateInformer_->publish(pose2DList);
}
