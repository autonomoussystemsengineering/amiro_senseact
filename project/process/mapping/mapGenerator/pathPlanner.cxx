/*
 * PathPlanner.cxx
 *
 *  Created on: Dec 14, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#include "pathPlanner.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <functional>
#include <array>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

PathPlanner::PathPlanner(float cellSize) :
		cellSize(cellSize) {

}

PathPlanner::~PathPlanner() {
}

std::vector<cv::Point2f> PathPlanner::getPathToTarget(cv::Mat& map, cv::Point3f pose, cv::Point2f target) {

	std::vector<cv::Point2f> path;
	std::vector<cv::Point2i> cellPath;

	// calculate path to frontier
	dijstraToTarget(Point2i((int) (pose.x / cellSize), (int) (pose.y / cellSize)),
			Point2i((int) (target.x / cellSize), (int) (target.y / cellSize)), pose.z, map, cellPath);

	// optimize the path
	optimizePath(map, cellPath);

	if (!cellPath.empty()) {
		// remove the robots cell from path
		cellPath.pop_back();

		// convert the path to world coordinates
		convertPath(cellPath, path);
	}

	return path;
}

// given a map and the robots pose calculate the path to the closest frontier cell
// important note: the path is sorted like a stack, the last point (goal is the first element)
std::vector<cv::Point2f> PathPlanner::getPathToFrontier(cv::Mat& map, cv::Point3f pose,
		std::vector<Point3f> otherPoses) {

	std::vector<cv::Point2f> path;
	std::vector<cv::Point2i> cellPath;

	// covert other robot poses to cells
	std::vector<cv::Point2i> otherRobotCells;
	for (cv::Point3f p : otherPoses) {
		if (p != cv::Point3f(0, 0, 0)) {
			otherRobotCells.push_back(Point2i((int) (p.x / cellSize), (int) (p.y / cellSize)));
			cout << Point2i((int) (p.x / cellSize), (int) (p.y / cellSize)) << endl;
		}
	}
	// calculate path to frontier
	dijstra(Point2i((int) (pose.x / cellSize), (int) (pose.y / cellSize)), pose.z, map, cellPath, otherRobotCells);
	// optimize the path
	optimizePath(map, cellPath);

	if (!cellPath.empty()) {
		// remove the robots cell from path
		cellPath.pop_back();

		// convert the path to world coordinates
		convertPath(cellPath, path);
	}

	return path;
}

// convert path from cell indices to world coordinates
void PathPlanner::convertPath(std::vector<cv::Point2i> &cellPath, std::vector<cv::Point2f> &path) {
	for (Point2i p : cellPath) {
		path.push_back(cv::Point2f(((float) p.x + 0.5) * cellSize, ((float) p.y + 0.5) * cellSize));
	}
}

// optimize the path with shortcuts
void PathPlanner::optimizePath(cv::Mat &om, std::vector<cv::Point2i> &path) {

	// check if the path is long enough to be optimized
	if (path.size() < 3) {
		return;
	}

	auto p0 = path.begin();
	auto p1 = p0 + 1;
	while (p1 < path.end()) {
		// check if there is a shortcut from p0 onwards
		while (p1 + 1 < path.end() && checkLine(*p0, *(p1 + 1), om)) {
			++p1;
		}
		// remove skipped points
		path.erase(p0 + 1, p1);

		p0++;
		p1 = p0 + 1;
	}
}

// check if there is no obstacle on the line between p0 and p1
// true means there is no obstacle
bool PathPlanner::checkLine(cv::Point2i &p0, cv::Point2i &p1, cv::Mat &om) {

	Point2i difference = p1 - p0;
	float distance = cv::norm(difference);
	// iterate over all cells on the line
	for (float s = 0; s < distance; s += 0.1) {
		// check there if a cell on the line is occupied
		if (om.at<uchar>(p0 + s / distance * difference) < 128) {
			return false;
		}

	}
	return true;
}

// calculate the path to the next unknown cell with the dijstra algorithm
void PathPlanner::dijstraToTarget(cv::Point2i robotCell, cv::Point2i targetCell, float theta, cv::Mat &om,
		std::vector<cv::Point2i> &path) {
	// macro for quick point access
#define connectedPoints(p) {Point2i(p.x+1,p.y+1), Point2i(p.x+1,p.y-1), Point2i(p.x-1,p.y+1), Point2i(p.x-1,p.y-1), Point2i(p.x+1,p.y), Point2i(p.x-1,p.y), Point2i(p.x,p.y+1), Point2i(p.x,p.y-1)}
	// initialize
	Mat dist(om.cols, om.rows, CV_32FC1, Scalar(std::numeric_limits<float>::max()));
	cv::Rect rect(cv::Point(0, 0), dist.size());

	// sanity check
	if (!rect.contains(robotCell)) {
		cerr << "PathPlanner: Robot pose unknown or not in the map!" << endl;
		return;
	}
	if (!rect.contains(targetCell)) {
		cerr << "PathPlanner: Target pose unknown or not in the map!" << endl;
		return;
	}
	std::vector<Point2i> qs = { robotCell };
	dist.at<float>(robotCell) = 0.0;
	Point2f direction(0, 0);
	if (theta != 0) {
		direction = Point2f(cos(theta), sin(theta));
	}

	path.clear();
	// comparator for vertices
	auto cmp = [&dist](const Point2i& p, const Point2i& q) {return dist.at<float>(p) < dist.at<float>(q);};

	// initialize goal with value out of the map
	Point2i goal(-1, -1);

	// try to find a route until there are no free cells left
	while (!qs.empty()) {

		// access the closest unvisited cell from the start cell
		std::vector<Point2i>::iterator up = std::min_element(qs.begin(), qs.end(), cmp);
		Point2i u = *up;
		qs.erase(up);

		if (dist.at<float>(u) >= 6 * (om.cols + om.rows)) {
			continue;
		}

		// check if an unknown cell was found
		if (u == targetCell) { // && cv::norm(u-robotCell)/cellSize > amiroRadius + 6*cellSize) {
			goal = u;
			qs.clear();
			break;
		}

		// iterate over connected cells
		for (Point2i v : connectedPoints(u) ) {

			if (!rect.contains(v)) {
				continue;
			}

			float prev = dist.at<float>(v);

			// update the distance to the connected cell
			Point2i difference = v - u;
			float distance = (float) cv::norm(difference);
			dist.at<float>(v) = min(
					dist.at<float>(u) + distance + (float) ((om.at<uchar>(v) < 128) ? om.cols + om.rows : 0),
					dist.at<float>(v));

			// add connected cell to list of unvisited cells
			if (prev == std::numeric_limits<float>::max()) {
				qs.push_back(v);
			}
		}
	}

	// check if a frontier was found
	if (goal != Point2i(-1, -1)) {
		cv::Point2i diff(0, 0), next;
		// backtracking to get the route to the start cell
		while (goal != robotCell) {
			std::vector<Point2i> bs = connectedPoints(goal);
			next = *std::min_element(bs.begin(), bs.end(), cmp);

			if (next - goal != diff) {
				path.push_back(goal);
			}
			diff = next - goal;

			goal = next;
		}
		path.push_back(robotCell);
	}

	// reverse to get route to the goal cell
#undef connectedPoints
}

// calculate the path to the next unknown cell with the dijstra algorithm
void PathPlanner::dijstra(cv::Point2i robotCell, float theta, const cv::Mat &om, std::vector<cv::Point2i> &path,
		std::vector<cv::Point2i> otherPoses) {
	// macro for quick point access
#define connectedPoints(p) {Point3i(p.x,p.y,(p.z+1)%8), Point3i(p.x,p.y,(p.z-1)%8), Point3i(p.x+round(cos(p.z*M_PI/4.0)),p.y+round(sin(p.z*M_PI/4.0)),p.z) }
#define connectedCells(p) {Point3i(p.x+1,p.y+1,0), Point3i(p.x+1,p.y-1,0), Point3i(p.x-1,p.y+1,0), Point3i(p.x-1,p.y-1,0), Point3i(p.x+1,p.y,0), Point3i(p.x-1,p.y,0), Point3i(p.x,p.y+1,0), Point3i(p.x,p.y-1,0)}
#define access_mat(m,p) m.at<float>(cv::Point2i(p.x+m.rows*p.z ,p.y))

	// average forward speed [m/s]
	float vT = 0.06;
	// average turning speed [rad/s]
	float vR = 0.6;

	float repellDistance = 15.0;
	float repellStrength = 10.0;
	float repellMax = 6 * (om.cols + om.rows);
	float repellSigma = repellDistance / sqrt(-2.0 * log(repellStrength / repellMax));

	// distance from end to path to the goal cell in cm
	float minApproachDistance = 5;

	// minimum length of a path in cm
	float minPathLength = 5;

	// distance in witch refined path planning is used in cm
	float preciseDistance = 15;

	// initialize
	Mat dist(om.rows, om.cols * 8, CV_32FC1, Scalar(std::numeric_limits<float>::max()));
	Mat prevX(om.rows, om.cols * 8, CV_32FC1);
	Mat prevY(om.rows, om.cols * 8, CV_32FC1);
	Mat prevTheta(om.rows, om.cols * 8, CV_32FC1);
	cv::Rect rect(cv::Point(0, 0), om.size());

	// sanity check
	if (!rect.contains(robotCell)) {
		cerr << "PathPlanner: Robot pose unknown or not in the map!" << endl;
		return;
	}
	theta = theta < 0 ? theta + 2 * M_PI : theta;
	std::vector<cv::Point3i> qs = { cv::Point3i(robotCell.x, robotCell.y, (int) round(4 * theta / M_PI)) };

	cv::Point3i rp(robotCell.x, robotCell.y, (int) round(4 * theta / M_PI));
	access_mat(dist,rp) = 0.0;

	path.clear();

	// comparator for vertices
	auto cmp = [&dist](const Point3i& p, const Point3i& q) {return access_mat(dist,p) < access_mat(dist,q);};

	// initialize goal with value out of the map
	Point3i goal(-1, -1, -1);

	// try to find a route until there are no free cells left
	while (!qs.empty()) {

		// access the closest unvisited cell from the start cell
		std::vector<Point3i>::iterator up = std::min_element(qs.begin(), qs.end(), cmp);
		Point3i u = *up;
		qs.erase(up);

		if (access_mat(dist,u) >= 6 * (om.cols + om.rows)) {
			continue;
		}

		cv::Point2i uCell(u.x, u.y);
		// check if an unknown cell was found

		float distanceToCell = cv::norm(uCell - robotCell);

		if (om.at<uchar>(uCell) == 128 && distanceToCell >= minPathLength) {
			goal = u;
			qs.clear();
			break;
		}

		std::vector<Point3i> neighbors;
		if (distanceToCell < preciseDistance) {
			neighbors = connectedPoints(u);
		} else {
			neighbors = connectedCells(u);
		}

		// iterate over connected cells
		for (Point3i v : neighbors) {

			Point2i vCell(v.x, v.y);
			if (!rect.contains(vCell) || v.z > 7 || v.z < 0) {
				continue;
			}
			float prev = access_mat(dist, v);

			// update the distance to the connected cell
			// distance is the estimated time to move from u to v
			float distanceFromStart = cv::norm(vCell - robotCell);
			float repell = 0;
			for (cv::Point2i o : otherPoses) {

				float robotDistance = cv::norm(vCell - o);

				if (abs(distanceFromStart - robotDistance) < 1) {
					repell += om.cols + om.rows;
				} else if (distanceFromStart > robotDistance) {
					repell += repellMax * exp(-0.5 * pow(robotDistance / repellSigma, 2));
				}
			}

			float distance = (float) 0.01 * cv::norm(vCell - uCell) / vT + repell;
			if (distanceToCell < preciseDistance || distanceFromStart < preciseDistance) {
				distance += min(abs(u.z - v.z), abs(u.z - v.z - 8)) * M_PI / 4.0 / vR;
			} else {
				distance += M_PI / 4.0 / vR;
			}
			float totaldistance = access_mat(dist, u) + distance
					+ (float) ((om.at<uchar>(vCell) < 128) ? om.cols + om.rows : 0) /*+ repell */;
			if (totaldistance < access_mat(dist, v)) {
				access_mat(dist,v) = totaldistance;
				// save previous pose for backtracking
				access_mat(prevX,v) = u.x;
				access_mat(prevY,v) = u.y;
				access_mat(prevTheta,v) = u.z;
			}

			// add connected cell to list of unvisited cells
			if (prev == std::numeric_limits<float>::max()) {
				qs.push_back(v);
			}
		}
	}

	// check if a frontier was found
	if (goal == Point3i(-1, -1, -1)) {
		//cout << "no frontier found" << endl;
	} else {
		cv::Point3i next, current(goal);
		cv::Point2i diff(0, 0), nextCell, goalCell(goal.x, goal.y), currentCell(goalCell);

		// backtracking to get the route to the start cell
		while (currentCell != robotCell) {
			next = cv::Point3i(access_mat(prevX, current), access_mat(prevY, current), access_mat(prevTheta, current));
			nextCell = cv::Point2i(next.x, next.y);

			// skip the last cells on the path
			if (cv::norm(currentCell - goalCell) >= minApproachDistance && nextCell != currentCell) {
				if (nextCell - currentCell != diff) {

					path.push_back(currentCell);
				}
				diff = nextCell - currentCell;
			}
			currentCell = nextCell;
			current = next;
		}
		path.push_back(robotCell);
	}

#undef connectedPoints
#undef connectedCells
#undef access_mat
}

