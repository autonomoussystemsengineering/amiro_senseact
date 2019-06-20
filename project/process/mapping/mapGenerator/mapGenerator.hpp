/*
 * mapGenerator.hpp
 *
 *  Created on: Dec 12, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#ifndef MAP_GENERATOR_H_
#define MAP_GENERATOR_H_

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/shared_ptr.hpp>
#include "mapUpdate.hpp"
#include <sensorModels/VCNL4020Models.h>

// Class that contains methods to update a obstacle grid map.
class MapGenerator {
public:
	// Constructor, cellSize in m
	MapGenerator(float cellSize);
	// Generate a MapUpdate from the given sensorvalues, the robots pose and the robots speed.
	MapUpdate createMapUpdate(const boost::shared_ptr<std::vector<int> > sensorValues, cv::Point3f &robotPose,
			float robotSpeed);
	// Generate a MapUpdate for a detected edge
	MapUpdate createEdgeUpdate(cv::Point3f &robotPose, bool side);
	MapUpdate createEdgePoseUpdate(cv::Point3f edgePose);
	// Apply the MapUpdate to the given map. The cv::Mat will be added at the pose (x,y) given by the MapUpdate.
	void updateMap(MapUpdate& m, cv::Mat& map);

	void generateObstacleMap(cv::Mat &map, cv::Mat &obstacleMap);

private:
	// calculate the smallest periodical angle in [-pi,pi]
	float getSmallestAngleDiff(float angle1, float angle2);
	// normalize the given angle to [-pi,pi]
	float normalizeAngle(float theta);

	// size of the local map
	static const cv::Size mapUpdateSize;
	// radius of the AMiRo in m
	static const float robotRadius;
	// number of sensors
	static const int numSensors = 8;
	// maximal range in m
	static const float maxSensorRange; //0.175;
	// maximal angle in radian
	static const float maxSensorAngle;
	// sensor position offset from front in radian
	static const float sensorPositionOffset;
	// size of a cell in m
	const float cellSize;

	// map constants
	static const int GRID_VALUE_MAX;
	static const int GRID_VALUE_MIN;
	static const int GRID_VALUE_UNSURE;

	static const int BLOCKED_THESHOLD = 112;
	static const int OPEN_THESHOLD = 127;

};

#endif  // MAP_GENERATOR_H_
