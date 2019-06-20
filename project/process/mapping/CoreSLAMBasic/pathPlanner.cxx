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

	cout << "Path request received from " << pose.x << "/" << pose.y << " (" << pose.z << ") to " << target.x << "/" << target.y << endl;
	cout << "Given map: Cellsize=" << cellSize << ", cols=" << map.cols << ", rows=" << map.rows << " => max Map size: " << map.cols*cellSize << "x" << map.rows*cellSize << endl;

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
std::vector<cv::Point2f> PathPlanner::getPathToFrontier(cv::Mat& map, cv::Point3f pose, std::vector<Point3f> otherPoses) {

	std::vector<cv::Point2f> path;
	std::vector<cv::Point2i> cellPath;

	// covert other robot poses to cells
	std::vector<cv::Point2i> otherRobotCells;
	for (cv::Point3f p : otherPoses) {
		if (p != cv::Point3f(0,0,0)) {
			otherRobotCells.push_back(Point2i((int) (p.x / cellSize), (int) (p.y / cellSize)));
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
	cv::Rect rect(cv::Point(0,0), dist.size());

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
	Point2f direction(0,0);
	if (theta != 0 ) {
		direction = Point2f(cos(theta), sin(theta));
	}

	path.clear();
	// comparator for vertices
	auto cmp = [&dist](const Point2i& p, const Point2i& q) {return dist.at<float>(p) < dist.at<float>(q);};

	// initialize goal with value out of the map
	Point2i goal(-1, -1);

	cerr << "Search target cell" << endl;
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
			cv::Point2f currentDirection = cv::Point2f(difference.x / distance, difference.y / distance);
			dist.at<float>(v) = min(
					dist.at<float>(u) + distance + (float) ((om.at<uchar>(v) < 128) ? om.cols + om.rows : 0),
					dist.at<float>(v));

			// add connected cell to list of unvisited cells
			if (prev == std::numeric_limits<float>::max()) {
				qs.push_back(v);
			}
		}
	}

	cerr << "Search path to target cell" << endl;
	// check if a frontier was found
	if (goal == Point2i(-1, -1)) {
		//cout << "No Frontier found" << endl;
	} else {
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
//	std::reverse(path.begin(),path.end());
#undef connectedPoints
}

// calculate the path to the next unknown cell with the dijstra algorithm
void PathPlanner::dijstra(cv::Point2i robotCell, float theta, cv::Mat &om, std::vector<cv::Point2i> &path, std::vector<cv::Point2i> otherPoses) {
	// macro for quick point access
#define connectedPoints(p) {Point2i(p.x+1,p.y+1), Point2i(p.x+1,p.y-1), Point2i(p.x-1,p.y+1), Point2i(p.x-1,p.y-1), Point2i(p.x+1,p.y), Point2i(p.x-1,p.y), Point2i(p.x,p.y+1), Point2i(p.x,p.y-1)}
	// initialize
	Mat dist(om.cols, om.rows, CV_32FC1, Scalar(std::numeric_limits<float>::max()));
	cv::Rect rect(cv::Point(0,0), dist.size());

	// sanity check
	if (!rect.contains(robotCell)) {
		cerr << "PathPlanner: Robot pose unknown or not in the map!" << endl;
		return;
	}

	std::vector<Point2i> qs = { robotCell };
	dist.at<float>(robotCell) = 0.0;
	Point2f direction(cos(theta), sin(theta));

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
		if (om.at<uchar>(u) == 128 && cv::norm(u - robotCell) > 8) {
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

			// calculate repell effect
			float repell = 0;
			for (cv::Point2i o : otherPoses) {
				float robotDistance = cv::norm(v-o);
				if (robotDistance > 0) {
					repell += 60.0 / robotDistance;
				} else {
					repell += 100.0;
				}
			}

			// update the distance to the connected cell
			Point2i difference = v - u;
			float distance = (float) cv::norm(difference);
			cv::Point2f currentDirection = cv::Point2f(difference.x / distance, difference.y / distance);
			dist.at<float>(v) = min(
					dist.at<float>(u) + distance + (float) (cv::norm(direction - currentDirection) / 4.0)
							+ (float) ((om.at<uchar>(v) < 128) ? om.cols + om.rows : 0) + repell, dist.at<float>(v));

			// add connected cell to list of unvisited cells
			if (prev == std::numeric_limits<float>::max()) {
				qs.push_back(v);
			}
		}
	}

	int ignore = 5;

	// check if a frontier was found
	if (goal == Point2i(-1, -1)) {

	} else {
		cv::Point2i diff(0, 0), next;
		// backtracking to get the route to the start cell
		while (goal != robotCell) {
			std::vector<Point2i> bs = connectedPoints(goal);
			next = *std::min_element(bs.begin(), bs.end(), cmp);

			// skip the last cells on the path
			if (ignore > 0) {
				goal = next;
				ignore--;
				continue;
			}

			if (next - goal != diff) {
				path.push_back(goal);
			}
			diff = next - goal;

			goal = next;
		}
		path.push_back(robotCell);
	}

	// reverse to get route to the goal cell
//	std::reverse(path.begin(),path.end());
#undef connectedPoints
}

