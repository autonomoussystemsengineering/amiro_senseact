#ifndef ASTAR_H
#define ASTAR_H

#include <opencv2/core/core.hpp>
#include <list>

class PathPlanner
{
public:
    PathPlanner();
    /**
     * @brief getPath generates a path from start to goal in a given map using the A*-algorithm.
     * @param start
     * @param goal
     * @param occupancyMap represents which cells are occupied. Values less than 128 are regarded as occupied.
     * @return a path from start to goal (including both). If no path exists an empty list is returned.
     */
    std::list<cv::Point2i> getPath(cv::Point2i &start, cv::Point2i &goal, cv::Mat1b &occupancyMap);

    void removeRedundantNodes(std::list<cv::Point2i> &path);

    void optimizePath(std::list<cv::Point2i> &path, cv::Mat1b &om);

    bool checkLine(cv::Point2i &p0, cv::Point2i &p1, cv::Mat1b &om);
};

#endif // ASTAR_H
