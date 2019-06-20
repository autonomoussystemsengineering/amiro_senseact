/*
 * Pathplanner.hpp
 *
 *  Created on: Dec 14, 2014
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <opencv2/core/core.hpp>

class PathPlanner {
public:
	PathPlanner(float cellSize);
	virtual ~PathPlanner();
	std::vector<cv::Point2f> getPathToFrontier(cv::Mat& map, cv::Point3f pose, std::vector<cv::Point3f> otherposes = {});
	std::vector<cv::Point2f> getPathToTarget(cv::Mat& map, cv::Point3f pose, cv::Point2f target);

private:
	void dijstra(cv::Point2i robotCell, float direction, cv::Mat &om, std::vector<cv::Point2i> &path, std::vector<cv::Point2i> otherposes = {});
	void dijstraToTarget(cv::Point2i robotCell, cv::Point2i targetCell, float theta, cv::Mat &om,
			std::vector<cv::Point2i> &path);
	void convertPath(std::vector<cv::Point2i> &cellPath, std::vector<cv::Point2f> &path);
	void optimizePath(cv::Mat &om, std::vector<cv::Point2i> &path);
	bool checkLine(cv::Point2i &p0, cv::Point2i &p1, cv::Mat &om);

	const float cellSize;
};

#endif /* PATHPLANNER_HPP_ */
