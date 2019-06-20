#ifndef LIKELIHOODFIELDMODEL_H
#define LIKELIHOODFIELDMODEL_H

#include <MSG.h>

#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "sensormodel.h"
#include "map.h"

#include "importancetype/importancetype.h"

#include <rsc/misc/langutils.h>

template<class T>
class LikelihoodFieldModel : public SensorModel
{
public:
    LikelihoodFieldModel(Map *map)
    {
        static_assert(std::is_base_of<ImportanceType, T>::value, "Derived not derived from ImportanceType");

        this->map = map;

        distanceToObstacle = new float[map->rows * map->cols];

        INFO_MSG("starting computing distances to obstacles...");
        time_t start = time(NULL);
        computeLikelihoodFieldTree();
        INFO_MSG("done. Took " << (time(NULL) - start) << " seconds");

        showDistanceMap();
    }

    ~LikelihoodFieldModel()
    {
        delete[] distanceToObstacle;
    }

    void computeWeight(sample_t &sample, laserscan_t &scan)
    {
        int idxx = map->poseToIndex(sample.pose.x);
        int idxy = map->poseToIndex(sample.pose.y);

        if (!map->isValid(idxx, idxy) || map->isOccupied(idxx, idxy)) {
            sample.importance = 0;
            return;
        }
//        boost::uint64_t totalStartTime = rsc::misc::currentTimeMicros();

        float pose_cos = cos(sample.pose.theta);
        float pose_sin = sin(sample.pose.theta);

//        boost::uint64_t skippingTime = 0;
//        boost::uint64_t indexTime = 0;
//        boost::uint64_t distanceTime = 0;
//        boost::uint64_t importanceTime = 0;

        for (size_t i = 0; i < scan.size; ++i) {

            /*
             * This is the same as:
             *
             * float angle = scan.scan_angle_start() + i * scan.scan_angle_increment(); (or - depending wether increment is positive or negative)
             * float globalAngle = sample.pose.theta + angle;
             * int idxx = map->poseToIndex( sample.pose.x + scan.scan_values(i) * cos(globalAngle) );
             * int idxy = map->poseToIndex( sample.pose.y + scan.scan_values(i) * sin(globalAngle) );
             *
             * since:
             * cos(a + b) = cos(a) * cos(b) - sin(a) * sin(b)
             * sin(a + b) = sin(a) * cos(b) + cos(a) * sin(b)
             *
             * where a = sample.pose.theta
             * and b = angle
             */
//            boost::uint64_t start = rsc::misc::currentTimeMicros();
            int idxx = map->poseToIndex( sample.pose.x + scan.values[i] * (pose_cos * scan.cos[i] - pose_sin * scan.sin[i]) );
            int idxy = map->poseToIndex( sample.pose.y + scan.values[i] * (pose_sin * scan.cos[i] + pose_cos * scan.sin[i]) );
//            indexTime += (rsc::misc::currentTimeMicros() - start);

//            start = rsc::misc::currentTimeMicros();
            float d;
            if (map->isValid(idxx, idxy)) {
                d = distanceToObstacle[idxy * map->cols + idxx];
            } else {
                // find a point on map by clamping
                int onMapX = std::max(0, std::min(idxx, map->cols - 1));
                int onMapY = std::max(0, std::min(idxy, map->rows - 1));
                // distance to next obstacle is distance from on-map point + on-map points distance to next obstacle
                d = distanceToObstacle[onMapY * map->cols + onMapX];
                d += map->meterPerCell * sqrt( pow(onMapX - idxx, 2) + pow(onMapY - idxy, 2) );
            }
//            distanceTime += (rsc::misc::currentTimeMicros() - start);

//            start = rsc::misc::currentTimeMicros();
            // map the distance to importance
            importanceType.addBeamDistance(d);
//            importanceTime += (rsc::misc::currentTimeMicros() - start);
        }

        sample.importance = importanceType.aggregateScanImportance();

//        boost::uint64_t totalTime = rsc::misc::currentTimeMicros() - totalStartTime;

//        INFO_MSG("==============");
//        INFO_MSG("skipping time  : " << skippingTime);
//        INFO_MSG("index time     : " << indexTime);
//        INFO_MSG("distance time  : " << distanceTime);
//        INFO_MSG("importance time: " << importanceTime);
//        INFO_MSG("total time: " << totalTime);
//        INFO_MSG("==============");

    }

    void normalizeWeights(sample_set_t *sampleSet)
    {
        importanceType.normalizeImportance(sampleSet);
    }

    T importanceType;

private:

    Map *map;
    float *distanceToObstacle;

    void computeLikelihoodFieldTree()
    {
        namespace bg = boost::geometry;
        namespace bgi = boost::geometry::index;

        typedef bg::model::point<int, 2, bg::cs::cartesian> point;

        std::vector<point> occupiedCells;

        // find all occupied cells and insert them in the index
        for (int row = 0; row < map->rows; ++row) {
            for (int col = 0; col < map->cols; ++col) {
                if (map->isOccupied(col, row)) {
                    point p = point(col, row);
                    occupiedCells.push_back(p);
                }
            }
        }

        bgi::rtree<point, bgi::quadratic<16>> rtree(occupiedCells);

        DEBUG_MSG("Inserted all occupied cells in the index (" << rtree.size() << ")");

        for (int row = 0; row < map->rows; ++row) {
            //DEBUG_MSG("progress: " << (row) / (float)(map->rows));
            for (int col = 0; col < map->cols; ++col) {
                //DEBUG_MSG("row: " << row << " col: " <<

                if (map->isOccupied(col, row)) {
                    //DEBUG_MSG("is occupied -> distance = 0");
                    distanceToObstacle[row * map->cols + col] = 0.0f;
                } else {
                    std::vector<point> result_n;
                    //DEBUG_MSG("Searching nearest neighbor...");
                    rtree.query(bgi::nearest(point(col, row), 1), std::back_inserter(result_n));
                    //DEBUG_MSG("found nearest neighbor");

                    float distance = sqrt( pow(col - result_n.at(0).get<0>(), 2) + pow(row - result_n.at(0).get<1>(), 2) );
                    //DEBUG_MSG("distance: " << distance);
                    distanceToObstacle[row * map->cols + col] = distance * map->meterPerCell;
                }
            }
        }

        DEBUG_MSG("Computed all distances");
    }

    void showDistanceMap()
    {
        float maxValue = 0;
        for (int row = 0; row < map->rows; row++) {
            for (int col = 0; col < map->cols; col++) {
                if (distanceToObstacle[row * map->cols + col] > maxValue) {
                    maxValue = distanceToObstacle[row * map->cols + col];
                }
            }
        }

        DEBUG_MSG("maxValue: " << maxValue);

        cv::Mat1b dbgImg(map->size());

        for (int row = 0; row < map->rows; row++) {
            for (int col = 0; col < map->cols; col++) {
                dbgImg.at<uchar>(row, col) = 255.0f * distanceToObstacle[row * map->cols + col] / maxValue;
            }
        }

#ifndef __arm__
        cv::imshow("distance map", dbgImg);
        cv::waitKey(0);
#else
        cv::imwrite("distances.png", dbgImg);
#endif
    }
};

#endif // LIKELIHOODFIELDMODEL_H
