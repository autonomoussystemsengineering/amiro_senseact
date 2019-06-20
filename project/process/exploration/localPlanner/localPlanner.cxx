/*
 * LocalPlanner.cxx
 */

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

#include "localPlanner.hpp"
#include <iostream>

using namespace std;

const float LocalPlanner::cellSize = 0.02f;
//const int LocalPlanner::maxW = 300000;
//const int LocalPlanner::maxV = 30000;
const int LocalPlanner::maxW = 300000;
const int LocalPlanner::maxV = 60000;

// constructor
LocalPlanner::LocalPlanner() : vecSteering_(new std::vector<int>(3)) {
	vecSteering_->at(2) = 500000000;
}

LocalPlanner::~LocalPlanner() {
	// TODO Auto-generated destructor stub
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

	DEBUG_MSG("Focussing: " << currentPose.x << "/" << currentPose.y << " to position " << nextPose.x << "/" << nextPose.y);
	// calculate the pose difference
	float diffX = nextPose.x - currentPose.x;
	float diffY = nextPose.y - currentPose.y;
	float theta = currentPose.z;


	while (theta > M_PI) {
		theta -= 2.0 * M_PI;
	}
	while (theta < -M_PI) {
		theta += 2.0 * M_PI;
	}


	// calculate the angular difference
	float angleDiff = (float) atan2(diffY, diffX) - theta;
	if (angleDiff > M_PI) {
		angleDiff -= 2.0 * M_PI;
	} else if (angleDiff < -M_PI) {
		angleDiff += 2.0 * M_PI;
	}

/*
	while (theta >= 2.0*M_PI) {
		theta -= 2.0 * M_PI;
	}
	while (theta < 0) {
		theta += 2.0 * M_PI;
	}

	float angle = atan2(diffY, diffX);
	float turnAngle = 0;
	float angleDiff = abs(angle-theta);
	if (angleDiff < M_PI) { // && angleDiff > turnThreshold) {
		turnAngle = angle-theta;
	} else {
		turnAngle = angle-theta+2*M_PI;
	}
	float dist = sqrt(diffX * diffX + diffY * diffY);
	DEBUG_MSG("Distance: " << dist << " m, Angle: " << turnAngle << " rad");

	int v = 0;
	int w = 0;
	int fac = 1;

	int waitingTime_us = 0;
	if (abs(turnAngle) < M_PI/18.0) {
		v = maxV;
		waitingTime_us = (int)(((dist*1000000.0) / ((float)maxV*fac)) * 1000000);
		INFO_MSG("Driving for " << dist << " m for " << (waitingTime_us/1000) << " ms with a speed of " << maxV/1000000.0 << " m/s");
	} else {
		if (turnAngle < 0) fac = -1;
		w = fac*maxW;
		waitingTime_us = (int)(((turnAngle*1000000.0) / ((float)maxW*fac)) * 1000000);
		INFO_MSG("Turning for " << turnAngle << " rad for " << (waitingTime_us/1000) << " ms with a speed of " << fac*maxW/1000000.0 << " rad/s");
	}
*/

	int v = 0;
	// calculate the robots angular velocity
	int w = min(max(-maxW, (int) (angleDiff * maxW / 2 * M_PI)), maxW);
	// if the robot is oriented towards the cell, drive forward
	if (abs(angleDiff) < M_PI / 6.0) {
		float dist = sqrt(diffX * diffX + diffY * diffY);
		v = min(max(6000 + (int) (dist * 20 * 24000), 0), maxV);
	}
	setSteering(v, w);

/*
	// send the steering command
	setSteering(v, w);
	usleep(waitingTime_us);
	setSteering(0, 0);
	usleep(500000);
*/
}

// send new steering commands to the motorControl
// only send the new steering command if it differs from the last steering command
// returns true if a new command was send
bool LocalPlanner::setSteering(int v, int w) {
	//
	if (v != vecSteering_->at(0) || w != vecSteering_->at(1)) {
		vecSteering_->at(0) = v;
		vecSteering_->at(1) = w;
		sendMotorCmd(v, w, myCAN);
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
	DEBUG_MSG("Actual Path:");
	for (cv::Point2f p : path_) {
		DEBUG_MSG(" - " << p.x << "/" << p.y);
	}
}

void LocalPlanner::sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
	CAN.setTargetSpeed(speed, angle);
	DEBUG_MSG( "v: " << speed << " w: " << angle);
}
