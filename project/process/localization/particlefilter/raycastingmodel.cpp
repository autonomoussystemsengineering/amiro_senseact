#include "raycastingmodel.h"

RayCastingModel::RayCastingModel(Map *map)
{
    this->map = map;
}

void
RayCastingModel::computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan)
{
    vector<float> sampleScan = simulateScan(sample.pose, scan);

    // TODO: set sigma
    float sigma = 0.2f;

    // compare scans
    float importance = 0;
    for (int i = 0; i < scan.scan_values_size(); ++i) {
        float measuredRay = scan.scan_values(i);
        float expectedRay = sampleScan[i];

        if (measuredRay > scan.scan_values_max()) {
            measuredRay = scan.scan_values_max();
        }

        if (expectedRay > scan.scan_values_max()) {
            expectedRay = scan.scan_values_max();
        }

//        /*
//         * Instead of simple gaussian, we use the logarithmic of it. This way
//         * the importance factors don't become too small.
//         */
//        importance -= log(sigma * sqrt(2 * M_PI)) - 0.5 * pow((measuredRay - expectedRay) / sigma, 2);
        //importance += pdf_gaussian(measuredRay, expectedRay, sigma);
        // simplified version of gaussian, neglecting constant factor
        importance += exp( -0.5 * pow( (measuredRay - expectedRay)/sigma, 2.0 ) );
    }

    sample.importance = importance;
    normFactor += importance;
}

vector<float>
RayCastingModel::simulateScan(const pose_t &pose, const rst::vision::LocatedLaserScan &scanConfig)
{
    vector<float> scan(scanConfig.scan_values_size());

    size_t i = 0;
    for (float angle = scanConfig.scan_angle_start();
        angle < scanConfig.scan_angle_end();
        angle += scanConfig.scan_angle_increment()) {

        float globalAngle = /*normalizeAngle*/(pose.theta + angle); // no normalization necessary cause it is passed to cos/sin

        // cast ray
        float rayLength = simulateRay(pose, globalAngle, scanConfig);

        scan[i++] = rayLength;
    }

    return scan;
}

float
RayCastingModel::simulateRay(const pose_t &pose, float globalAngle, const rst::vision::LocatedLaserScan &scanConfig)
{
    // start point
    int x0 = map->poseToIndex(pose.x);
    int y0 = map->poseToIndex(pose.y);

    // target point
    int x1 = map->poseToIndex( pose.x + scanConfig.scan_values_max() * cos(globalAngle) );
    int y1 = map->poseToIndex( pose.y + scanConfig.scan_values_max() * sin(globalAngle) );

    // determine the fast growing direction
    bool steep;
    if(abs(y1 - y0) > abs(x1 - x0)) {
        steep = true;
    } else {
        steep = false;
    }

    // swap x and y, so that x is the fast growing direction
    if (steep) {
        int tmp = x0;
        x0 = y0;
        y0 = tmp;

        tmp = x1;
        x1 = y1;
        y1 = tmp;
    }

    // differences
    int deltax = abs(x1 - x0);
    int deltay = abs(y1 - y0);

    int error = 0;
    int deltaerr = deltay;

    int x = x0;
    int y = y0;

    // determine direction
    int xstep;
    if (x0 < x1) {
        xstep = 1;
    } else {
        xstep = -1;
    }

    int ystep;
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    // first check if start positions are valid (in map and not occupied)
    if (steep) {
        if (!map->isValid(y,x) || map->isOccupied(y,x)) {
            return 0;
        }
    } else {
        if (!map->isValid(x,y) || map->isOccupied(x,y)) {
            return 0;
        }
    }

    // until target point reached
    while (x != (x1 + xstep * 1)) {
        x += xstep;
        error += deltaerr;

        if (2 * error >= deltax) {
            y += ystep;
            error -= deltax;
        }

        if (steep) {
            if (!map->isValid(y,x) || map->isOccupied(y,x)) {
                return sqrt( (x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->meterPerCell;
            }
        } else {
            if (!map->isValid(x,y) || map->isOccupied(x,y)) {
                return sqrt( (x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->meterPerCell;
            }
        }
    }

    return scanConfig.scan_values_max();
}

void
RayCastingModel::normalizeWeights(sample_set_t *sampleSet)
{
    float sum = 0;
    for (size_t i = 0; i < sampleSet->size; ++i) {
        sampleSet->samples[i].importance /= normFactor;

        assert(sampleSet->samples[i].importance >= 0.0f);
        assert(sampleSet->samples[i].importance <= 1.0f);

        sum += sampleSet->samples[i].importance;
    }

    // reset normalization factor
    normFactor = 0;
}
