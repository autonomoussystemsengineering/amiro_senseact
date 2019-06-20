/*
 * mapGenerator.cxx
 *
 *  Created on: Dec 12, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */
#include "mapGenerator.hpp"

using namespace std;
using namespace cv;

// size of the local map
const cv::Size MapGenerator::mapUpdateSize = cv::Size(60, 60);
// radius of the AMiRo in m
const float MapGenerator::robotRadius = 0.05;

// maximal range in m
const float MapGenerator::maxSensorRange = 0.12; //0.175;
// maximal angle in radian
const float MapGenerator::maxSensorAngle = M_PI * 60.0 / 180.0;
// sensor position offset from front in radian
const float MapGenerator::sensorPositionOffset = M_PI * 7.0 / 8.0;

// IR-sensor constants
const float MapGenerator::IR_CONVERTER_CALC_ALPHA = 0.942693757414292;
const float MapGenerator::IR_CONVERTER_CALC_BETA = -16.252241893638708;
const float MapGenerator::IR_CONVERTER_CALC_DELTA = 0;
const float MapGenerator::IR_CONVERTER_CALC_XI = 1.236518540376969;
const float MapGenerator::IR_CONVERTER_CALC_MEAS_VARIANCE = 20.886268537074187;

const int MapGenerator::GRID_VALUE_MAX = 127;
const int MapGenerator::GRID_VALUE_MIN = -128;
const int MapGenerator::GRID_VALUE_UNSURE = 0;

// constructor
MapGenerator::MapGenerator(float cellSize) :
		cellSize(cellSize) {
}

// Generate a MapUpdate from the sensor values.
// This will be added directly to the global map at the robots position to update it.
MapUpdate MapGenerator::createMapUpdate(const boost::shared_ptr<std::vector<int> > sensorValues, cv::Point3f &robotPose,
		float movedDistance) {

	// calculate the offsets depending on the distance the robot moved
	const float g = 15.0 * movedDistance / 0.01;
	const float gridObstacle = max(-g, (float) GRID_VALUE_MIN);
	const float gridObstacleUnsure = max((float) -(0.7 * g), (float) GRID_VALUE_MIN);
	const float gridFree = min((float) 0.5 * g, (float) GRID_VALUE_MAX);

	// calculate the robots position in the local map
	int robotCellX = (int) (robotPose.x / cellSize);
	int robotCellY = (int) (robotPose.y / cellSize);
	float robotOffsetX = robotPose.x - robotCellX * cellSize;
	float robotOffsetY = robotPose.y - robotCellY * cellSize;

	// calculate the sensor positions
	float sensorPos[numSensors][3];
	// iterate over the sensors
	for (int i = 0; i < numSensors; ++i) {
		// calculate the position and angle of sensor i
		float sensorTheta = normalizeAngle(-2.0 * M_PI * i / numSensors + sensorPositionOffset + robotPose.z);

		// add robot position offset to sensorposition
		float sensorX = cos(sensorTheta) * robotRadius + robotOffsetX;
		float sensorY = sin(sensorTheta) * robotRadius + robotOffsetY;

		sensorPos[i][0] = sensorX;
		sensorPos[i][1] = sensorY;
		sensorPos[i][2] = sensorTheta;
	}

	// initialize mapupdate with 0
	MapUpdate mapUpdate(mapUpdateSize, CV_8SC1, Scalar(0));

	// copy the robot pose
	mapUpdate.x = robotPose.x;
	mapUpdate.y = robotPose.y;
	mapUpdate.theta = robotPose.z;

	// iterate over the map
	for (int y = 0; y < mapUpdateSize.height; ++y) {
		for (int x = 0; x < mapUpdateSize.width; ++x) {

			// get the position of the cell (TODO preprocess this)
			float cellX = cellSize * (x + 0.5 - mapUpdateSize.width / 2.0);
			float cellY = cellSize * (y + 0.5 - mapUpdateSize.height / 2.0);

			// check if cell is under robot
			if (sqrt(cellX * cellX + cellY * cellY) <= robotRadius) {
				mapUpdate.at<char>(y, x) = GRID_VALUE_MAX;
			} else {
				int maxValue = GRID_VALUE_MIN - 1;
				// iterate over the sensors
				for (int s = 0; s < numSensors; s++) {

					float diffX = cellX - sensorPos[s][0];
					float diffY = cellY - sensorPos[s][1];

					// calculate angle and distance between the cell and the sensor
					float cellAngle = getSmallestAngleDiff((float) atan2(diffY, diffX), sensorPos[s][2]);
					float cellDist = sqrt(diffX * diffX + diffY * diffY);

					// check if the cell is in range of the sensor
					if (cellAngle <= maxSensorAngle && cellDist <= maxSensorRange) {
						float relCellSize = cellSize * (abs(sin(cellAngle)) + abs(cos(cellAngle)));

						// use the inverse sensor model to predict the obstacle distance
						float predDist = invSensorModel(cellAngle, (float) sensorValues->at(s));
						float diffDist = abs(cellDist - predDist);
						float err = getDistErrorSensorModel(predDist, cellAngle);

						// check whether the cell is free or blocked
						if (diffDist < relCellSize) {
							// at measured position
							maxValue = max(maxValue, (int) (gridObstacle));
						} else if (diffDist < relCellSize + err) {
							// in variance of measured position
							maxValue = max(maxValue, (int) (gridObstacle));
						} else if (predDist > cellDist) {
							// in free space
							maxValue = max(maxValue, (int) (gridFree));
						} else {
							// behind measurements -> ignoring
							maxValue = max(maxValue, GRID_VALUE_UNSURE);
						}

					}
				}
				if (maxValue < GRID_VALUE_MIN) {
					maxValue = GRID_VALUE_UNSURE;
				}
				mapUpdate.at<char>(y, x) = maxValue;
			}
		}
	}
	return mapUpdate;
}

// Generate a Mapupdate for a detected edge
MapUpdate MapGenerator::createEdgeUpdate(cv::Point3f &robotPose, bool side) {

	// initialize the update with zero

	MapUpdate mapUpdate(mapUpdateSize, CV_8SC1, Scalar(0));

	// set the pose of the update
	mapUpdate.x = robotPose.x;
	mapUpdate.y = robotPose.y;
	mapUpdate.theta = 0;
	float offset = side ? -M_PI / 2 : M_PI / 2;

	// calculate edge coordinates
	cv::Point2i mid(mapUpdateSize.width / 2, mapUpdateSize.height / 2);
	cv::Point2f p0(1.5 * robotRadius * cos(robotPose.z + offset), 1.5 * robotRadius * sin(robotPose.z + offset));
	cv::Point2i p1((int) ((p0.x + robotRadius * cos(robotPose.z + offset + M_PI / 2)) / cellSize),
			(int) ((p0.y + robotRadius * sin(robotPose.z + offset + M_PI / 2)) / cellSize));
	cv::Point2i p2((int) ((p0.x + robotRadius * cos(robotPose.z + offset - M_PI / 2)) / cellSize),
			(int) ((p0.y + robotRadius * sin(robotPose.z + offset - M_PI / 2)) / cellSize));

	// draw a line in front of the robot
	cv::line(mapUpdate, mid + p1, mid + p2, Scalar(GRID_VALUE_MIN));

	return mapUpdate;
}

// Apply the MapUpdate to the given map. The cv::Mat will be added at the pose (x,y) given by the MapUpdate.
void MapGenerator::updateMap(MapUpdate& m, Mat& map) {
	// Get the position of the local map in the global map (Left upper corner).
	// The robot is in the middle of the local map.
	int localMapX = (int) (m.x / cellSize - m.cols / 2.0);
	int localMapY = (int) (m.y / cellSize - m.rows / 2.0);

	// check if the localMap is positioned in the global map
	if (localMapX > -m.cols && localMapX < map.cols && localMapY > -m.rows && localMapY < map.rows) {
		// determine the size of the overlap
		Size s(min( { m.cols, m.cols + localMapX, map.cols - localMapX }),
				min( { m.rows, m.rows + localMapY, map.rows - localMapY }));

		// Add the local to the global map. Note: OpenCV automatically will clip the values in the global map between -128 and 127.
		map(Rect(Point(max(localMapX, 0), max(localMapY, 0)), s)) += m(
				Rect(Point(max(0, -localMapX), max(0, -localMapY)), s));
	}
}

// generate an obstacle map using thresholding and dilate the obstacles
void MapGenerator::generateObstacleMap(cv::Mat &map, cv::Mat &obstacleMap) {
	cv::Mat o0, o1, f0, f1;

	// calculate blocked areas
	map.convertTo(o0, CV_16SC1);
	o0 += 128;
	o0.convertTo(o1, CV_8UC1);
	cv::threshold(o1, o1, BLOCKED_THESHOLD, 255, THRESH_BINARY_INV);

	// erode blocked areas
	int erosion_size = (int) (robotRadius / cellSize) + 1;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));
	dilate(o1, o1, element);

	// calculate map without obstacles
	map.convertTo(f0, CV_16SC1);
	f0.convertTo(f1, CV_8UC1);
	cv::threshold(f1, f1, 0, OPEN_THESHOLD, THRESH_BINARY);
	f1 += 128;

	// calculate the final map with eroded obstacles and show it
	cv::subtract(f1, o1, obstacleMap);

}

// calculate the distance to an obstacle given the obstacles angle relative to the sensor and the sensor value
float MapGenerator::invSensorModel(float angle, float sensorValue) {

	float cosxi = cos(IR_CONVERTER_CALC_XI * angle);
	if (cosxi < 0) {
		cosxi *= -1;
	}
	float divi = sensorValue - IR_CONVERTER_CALC_BETA;
	if (divi <= 0) {
		divi = 1;
	}
	return sqrt(IR_CONVERTER_CALC_ALPHA * cosxi / divi + IR_CONVERTER_CALC_DELTA * cosxi);
}

// error of the predicted distance
float MapGenerator::getDistErrorSensorModel(float dist, float angle) {
	float sig = sqrt(IR_CONVERTER_CALC_MEAS_VARIANCE);

	// calculate error
	float cosxi = abs(cos(IR_CONVERTER_CALC_XI * angle));
	float diffdistdelta = dist * dist / cosxi - IR_CONVERTER_CALC_DELTA;
	float error = (diffdistdelta * diffdistdelta) / (2 * dist * IR_CONVERTER_CALC_ALPHA * sqrt(1 / cosxi)) * sig;

	return error;
}

// normalize a angle to [-pi,pi]
float MapGenerator::normalizeAngle(float theta) {
	return theta - 2.0 * M_PI * floorf((theta + M_PI) / (2.0 * M_PI));
}

// calculate the smallest periodical angle in [-pi,pi]
float MapGenerator::getSmallestAngleDiff(float angle1, float angle2) {
	return M_PI - abs(abs(angle1 - angle2) - M_PI);
}
