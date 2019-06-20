#ifndef MAP_H
#define MAP_H

#include <opencv2/core/core.hpp>

class Map : public cv::Mat1b
{
public:

    std::vector<cv::Point2i> freeCells;

    Map(const cv::Mat &m) : cv::Mat1b(m)
    {
        // collect all free cells
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                if (isFree(col, row)) {
                    freeCells.push_back(cv::Point2i(col, row));
                }
            }
        }
    }

    float meterPerCell = 0;

    inline bool isValid(const int x, const int y)
    {
        return (x >= 0 && x < cols && y >= 0 && y < rows);
    }

    inline bool isOccupied(const int x, const int y)
    {
        return (at<uchar>(y,x) < occupied);
    }

    inline bool isFree(const int x, const int y)
    {
        return (at<uchar>(y,x) > occupied);
    }

    inline size_t poseToIndex(const float &x)
    {
        return round(x / meterPerCell);
    }

private:
    const uchar occupied = (numeric_limits<uchar>::max() - numeric_limits<uchar>::min()) / 2;
};

#endif // MAP_H
