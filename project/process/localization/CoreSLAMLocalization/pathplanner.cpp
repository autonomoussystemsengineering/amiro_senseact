#include "pathplanner.h"
#include <algorithm> // for std::min_element
#include <iterator> // for std::prev in optimizing path
#include <iostream>
#include <map>

#include <set>

#define VEC_CONTAINS(v,x) (std::find(v.begin(), v.end(), x) != v.end())

PathPlanner::PathPlanner()
{
}

bool operator<(cv::Point2i const& a, cv::Point2i const& b)
{
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

struct pointpointcmp {
    bool operator()(cv::Point2i const& a, cv::Point2i const& b)
    {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

std::list<cv::Point2i> PathPlanner::getPath(cv::Point2i &start, cv::Point2i &goal, cv::Mat1b &occupancyMap) {
    //std::vector<cv::Point2i> openlist = { start };

    // Stores best-case distances
    cv::Mat f(occupancyMap.cols, occupancyMap.rows, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
    f.at<float>(start) = 0;

    // Stores actual distances
    cv::Mat g(occupancyMap.cols, occupancyMap.rows, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
    g.at<float>(start) = 0;

    // Stores already visited nodes
    cv::Mat closedlist(occupancyMap.cols, occupancyMap.rows, CV_8UC1, cv::Scalar(0));

    // Set up boundary for fast check of node is on map
    cv::Rect boundary(cv::Point2i(0,0), f.size());

    // Sort Mat elements
    auto cmp = [&f](const cv::Point2i& p, const cv::Point2i& q) {
        return f.at<float>(p) < f.at<float>(q);
    };

    std::set<cv::Point2i,pointpointcmp> openlist = std::set<cv::Point2i,pointpointcmp>();
    openlist.insert(start);

    // Store predecessors
    std::map<cv::Point2i,cv::Point2i,pointpointcmp> predecessor = std::map<cv::Point2i,cv::Point2i,pointpointcmp>();

    // Main loop
    bool foundPath = false;
    do {
        // Find most promising candidate
        auto iter = std::min_element(openlist.begin(), openlist.end(), cmp);
        cv::Point2i currentNode = *iter;
        openlist.erase(iter);

        // Are we at goal yet?
        if (currentNode == goal) {
            foundPath = true;
            break;
        }

        // Note this node as visited
        closedlist.at<uchar>(currentNode) = 1;

        /*
         * Expanding node
         */
        const std::vector<cv::Point2i> successors = {
            cv::Point2i(currentNode.x+1,currentNode.y+1),
            cv::Point2i(currentNode.x+1,currentNode.y),
            cv::Point2i(currentNode.x+1,currentNode.y-1),

            cv::Point2i(currentNode.x-1,currentNode.y+1),
            cv::Point2i(currentNode.x-1,currentNode.y),
            cv::Point2i(currentNode.x-1,currentNode.y-1),

            cv::Point2i(currentNode.x,currentNode.y+1),
            cv::Point2i(currentNode.x,currentNode.y-1)
        };

        for (cv::Point2i successor : successors) {
            // Check if on map
            if (!boundary.contains(successor)) {
                continue;
            }

            // Check if already checked
            if (closedlist.at<uchar>(successor)) {
                continue;
            }

            // Check if node is free
            if (occupancyMap.at<uchar>(successor) < 128) {
                continue;
            }

            // New possible distance
            float tentative_g = g.at<float>(currentNode) + cv::norm(successor - currentNode);

            // Node is already in openlist and has a shorter distance -> discard it
            bool successorInOpenList = openlist.find(successor) != openlist.end();
            if (successorInOpenList && tentative_g >= g.at<float>(successor)) {
                continue;
            }

            // Store predecessor and new distance
            predecessor[successor] = currentNode;
            g.at<float>(successor) = tentative_g;

            // Update best-case distance, h = cv::norm(goal - successor) is the heuristic
            f.at<float>(successor) = tentative_g + cv::norm(goal - successor);
            if (!successorInOpenList) {
                openlist.insert(successor);
            }
        }

    } while (!openlist.empty());

    if (!foundPath) {
        return std::list<cv::Point2i>();
    }

    // Backtrack path
    cv::Point2i currentNode = goal;
    std::list<cv::Point2i> path = { goal };
    while (currentNode != start) {
        currentNode = predecessor[currentNode];
        path.push_front(currentNode);
    }

    return path;
}

void PathPlanner::removeRedundantNodes(std::list<cv::Point2i> & path) {
    if (path.size() < 2)
        return;

    /*
     *   p0                p0
     *     \                \
     *      p1      --->     \
     *       \                \
     *        p2               p2
     */


    std::list<cv::Point2i>::iterator iter = path.begin();

    cv::Point2i p0 = *iter;
    iter++;
    cv::Point2i p1 = *iter;

    while (iter != path.end()) {
        iter++;
        cv::Point2i p2 = *iter;

        if (p1 - p0 == p2 - p1) {
            path.erase(std::prev(iter));
        }

        p0 = p1;
        p1 = p2;
    }
}

// optimize the path with shortcuts
void PathPlanner::optimizePath(std::list<cv::Point2i> &path, cv::Mat1b &om) {

    // check if the path is long enough to be optimized
    if (path.size() < 3) {
        return;
    }

    std::list<cv::Point2i>::iterator p0 = path.begin();
    std::list<cv::Point2i>::iterator p1 = std::next(p0);
    while (p1 != path.end()) {
        // check if there is a shortcut from p0 onwards
        while (std::next(p1) != path.end() && checkLine(*p0, *std::next(p1), om)) {
            ++p1;
        }
        // remove skipped points
        path.erase(std::next(p0), p1);

        p0++;
        p1 = std::next(p0);
    }
}

// check if there is no obstacle on the line between p0 and p1
// true means there is no obstacle
bool PathPlanner::checkLine(cv::Point2i &p0, cv::Point2i &p1, cv::Mat1b &om) {

    cv::Point2i difference = p1 - p0;
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
