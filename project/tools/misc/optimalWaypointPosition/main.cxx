// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// program options
#include <boost/program_options.hpp>

// debug, info, error messages etc.
#include <MSG.h>

cv::Mat1b computeSafePositions(const cv::Mat1b &map, int distanceFromWall, const cv::Mat1b &room) {
    cv::Mat1b safePositions;

    // create structuring element
    cv::Size size(2 * distanceFromWall, 2 * distanceFromWall);
    cv::Point anchor(distanceFromWall, distanceFromWall);
    cv::Mat1b kernelOuter = cv::getStructuringElement(cv::MORPH_ELLIPSE, size, anchor);

    size = cv::Size(2 * (distanceFromWall - 1), 2 * (distanceFromWall - 1));
    anchor = cv::Point(distanceFromWall - 1, distanceFromWall - 1);
    cv::Mat1b kernelInner = cv::getStructuringElement(cv::MORPH_ELLIPSE, size, anchor);

    // erode and show differences from original
    cv::Mat1b outer;
    cv::erode(map, outer, kernelOuter);
    cv::Mat1b inner;
    cv::erode(map, inner, kernelInner);

    cv::bitwise_xor(inner, outer, safePositions);

    // Only consider positions in room
    //cv::Mat1b innerRoom;
    // part of inner room will 255
    //cv::threshold(room, innerRoom, 129, 255, CV_THRESH_BINARY_INV);

    cv::bitwise_and(safePositions, room, safePositions);

    return safePositions;
}

void computeRayCoverage(const cv::Mat1b &map, const cv::Point &robotPosition, const cv::Point &rayEnd,
                        cv::Mat1b &coverage) {
    cv::LineIterator tracer(map, robotPosition, rayEnd);

    // while no obstacle found
    uchar *current = *tracer;
    while (*current > 130) {
        coverage.at<uchar>(tracer.pos()) = 255;

        // don't wrap around
        if (tracer.pos().x == 0 || tracer.pos().x == map.cols - 1
                || tracer.pos().y == 0 || tracer.pos().y == map.rows - 1) {
            break;
        }

        // advance tracer
        ++tracer;
        current = *tracer;
    }
}

cv::Mat1b computeScanCoverage(cv::Mat1b &map, const cv::Point &robotPosition) {
    float angleStep = 2 * M_PI / 1024;
    float rayLength = 100;

    cv::Mat1b covered(map.size());
    covered.setTo(0);

    for (float currentAngle = 0; currentAngle < 2 * M_PI; currentAngle += angleStep) {
        cv::Point rayEnd(robotPosition);
        rayEnd += cv::Point(cos(currentAngle) * rayLength, sin(currentAngle) * rayLength);

        computeRayCoverage(map, robotPosition, rayEnd, covered);
    }

    return covered;
}

// select all pixels [126;128] (including)
void extractRoom(const cv::Mat1b &room, cv::Mat1b &dst) {
    cv::threshold(room, dst, 125, 0 /* ignored */, CV_THRESH_TOZERO);
    cv::threshold(dst, dst, 128, 0 /* ignored */, CV_THRESH_TOZERO_INV);
    cv::threshold(dst, dst, 124, 255, CV_THRESH_BINARY);
}

int main(int argc, char *argv[]) {
    // file paths to maps
    std::string mapImagePath = "";
    std::vector<std::string> roomImagePaths;
    int distanceFromWall = 0;

    /*
     * Setup program options
     */
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("debug", "enables additional debug output")
            ("map", po::value<std::string>(&mapImagePath), "file path to the map")
            ("rooms", po::value< std::vector<std::string> >(&roomImagePaths)->multitoken(), "file paths to rooms, multiple allowed")
            ("distanceFromWall", po::value<int>(&distanceFromWall)->default_value(5), "distance from wall in pixel")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    /*
     * Verify program options
     */
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (!vm.count("map")) {
        ERROR_MSG("No file path to map given!");
        return 1;
    }

    if (!vm.count("rooms")) {
        ERROR_MSG("No file paths to rooms given!");
        return 1;
    }

    bool debug = vm.count("debug") > 0;

    /*
     * Load map and rooms
     */
    // load map
    cv::Mat1b map = cv::imread(mapImagePath, CV_LOAD_IMAGE_GRAYSCALE);
    if (!map.data) {
        ERROR_MSG("Loading map file failed!");
        return 1;
    }

    if (debug) {
        cv::imshow("map", map);
    }

    for (std::string imagePath : roomImagePaths) {
        // load room
        cv::Mat1b room = cv::imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
        if (!room.data) {
            ERROR_MSG("Loading room file \"" << imagePath << "\" failed");
            return 1;
        }

        if (debug) {
            cv::imshow("room", room);
        }

        // extract the room (by color)
        cv::Mat1b roomOnly;
        extractRoom(room, roomOnly);

        if (debug) {
            cv::imshow("roomOnly-init", roomOnly);
            cv::waitKey(10);
        }

        // compute safe positions
        cv::Mat1b safePositions = computeSafePositions(map, distanceFromWall, roomOnly);

        if (debug) {
            cv::imshow("safePositions", safePositions);
            cv::waitKey(10);
        }

        // save best position in this room
        int highestCoverage = 0;
        cv::Point bestPosition;

        // for each safe position
        for (int row = 0; row < safePositions.rows; ++row) {
            for (int col = 0; col < safePositions.cols; ++col) {
                // is safe position?
                if (safePositions.at<uchar>(row, col) > 0) {
                    cv::Point position(col, row);
                    cv::Mat1b covered = computeScanCoverage(map, position);

                    // only consider pixels in this room
                    cv::bitwise_and(covered, roomOnly, covered);

                    int coverage = cv::countNonZero(covered);
                    if (debug) {
                        DEBUG_MSG("coverage: " << coverage);
                    }

                    if (coverage > highestCoverage) {
                        highestCoverage = coverage;
                        bestPosition = position;
                    }
                }
            }
        }

        // output best position
        float coverageRelative = (float)highestCoverage / cv::countNonZero(roomOnly);
        std::cout << "bestPosition: " << bestPosition << " with " << coverageRelative << std::endl;

        // display the best position
        cv::Mat3b colorMap;
        cv::cvtColor(map, colorMap, CV_GRAY2BGR);
        cv::circle(colorMap, bestPosition, 5, cv::Scalar(255,0,0));
        cv::imshow("bestPosition", colorMap);
        cv::waitKey();
    }

    return 0;
}
