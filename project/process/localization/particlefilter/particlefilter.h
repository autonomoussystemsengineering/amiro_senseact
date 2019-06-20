#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

using namespace std;

#include <cstddef>
#include <Eigen/Core>
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <opencv2/core/core.hpp>
#include <random>
#include "map.h"
#include "sampleset.h"
#include "sensormodel.h"
#include "laserscan.h"

class ParticleFilter
{
public:

    /**
     * @brief ParticleFilter
     * @param sampleCount Number of particles used in this filter.
     * @param meterPerCell Resolution of the map in m/pixel.
     * @param scanConfig Used to determine number of rays, minimum and maximum values of scans etc...
     * @param odom The initial odometry data used to form odometry deltas.
     * @param map A openCV mat to represent to map.
     */
    ParticleFilter(size_t maxSampleCount, rst::geometry::Pose odom, const rst::vision::LocatedLaserScan &scanConfig, Map *map, SensorModel *sensorModel, float newSampleProb, bool doKLDSampling = false, int beamskip = 1, sample_set_t *sampleSet = NULL, float samplingStdDev = 0.01f);
    ~ParticleFilter();

    /**
     * @brief update updates the believe of the particle filter.
     * @param scan currently measured laser scan used for importance sampling.
     * @param odom currently measured odometry to compute odometry delta in the sampling step.
     * @param forceUpdate forces the particle filter to update even when odometry increment is small.
     */
    bool update(const rst::vision::LocatedLaserScan &scan, const rst::geometry::Pose &odom, bool forceUpdate = false);

    sample_set_t *getSamplesSet() {
        return sampleSet;
    }

    pose_t getWeightedMeanPose() {
        float meanX = 0.0f, meanY = 0.0f;
        float sumThetaX = 0.0f, sumThetaY = 0.0f;

        for (size_t i = 0; i < sampleSet->size; ++i) {
            sample_t sample = sampleSet->samples[i];
            float importance = sample.importance / sampleSet->totalWeight;
            // accumulate data for mean
            meanX += importance * sample.pose.x;
            meanY += importance * sample.pose.y;
            sumThetaX += importance * cos(sample.pose.theta);
            sumThetaY += importance * sin(sample.pose.theta);
        }

        pose_t meanPose;
        meanPose.x = meanX;
        meanPose.y = meanY;
        meanPose.theta = atan2(sumThetaY, sumThetaX);

        return meanPose;
    }

private:
    size_t maxSampleCount;
    size_t width, height;

    sample_set_t *sampleSet;
    sample_set_t *newSampleSet;

    pose_t prevOdom = {0,0,0};
    pose_t odometryDelta;

    laserscan_t laserscan;
    float *scan_cos;
    float *scan_sin;
    int beamskip = 1;

    Map *map;

    //std::default_random_engine rng;
    std::subtract_with_carry_engine<uint_fast32_t, 24, 10, 24> rng; // std::min_stdrand
    std::normal_distribution<float> samplingDistribution;

    void initSamples();
    sample_t randomSample();

    float *cumulative;

    sample_t rouletteWheelSelection(float t);

    /*
     *        p_t
     *
     *         O->
     *        /__/  phi2
     *       /
     *      /
     *     /\
     *  <-O |
     *  \___/ phi1
     *
     *    p_(t-1)
     *
     */

    // angle from old odometry theta to driving direction
    float phi1;
    // angle from driving direction to new new odometry theta
    float phi2;
    // distance between old and new odometry
    float odometryIncrement;

    // pre-computes above variables
    bool preparePoseUpdate(pose_t &newOdom);
    void updatePose(sample_t &sample);

    float newSampleProb = 0.0f;
    void sampling(sample_t &sample);


    SensorModel *sensorModel;

    pose_t convertPose(const rst::geometry::Pose &odom);
    void convertScan(const rst::vision::LocatedLaserScan &scan);

    // normalizes an angle to [-PI;+PI)
    inline float normalizeAngle(float theta)
    {
        if (theta < -M_PI || theta >= M_PI) {
            theta = fmod(theta + M_PI, 2*M_PI);
            if (theta < 0)
                theta += 2*M_PI;
            theta = theta - M_PI;
        }

        return theta;
    }

    inline float pdf_gaussian(float x, float mean, float sigma)
    {
        return ( 1 / ( sigma * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x - mean)/sigma, 2.0 ) );
    }

    inline float clamp(float x, float min, float max)
    {
        return std::max(min, std::min(x, max));
    }

    /*
     * KLD-Sampling related
     */

    bool doKLDSampling = true;

    float binSizeX = 0.50f;
    float binSizeY = 0.50f;
    float binSizeRot = (10 * M_PI / 180);

    int nbBinsX = 0;
    int nbBinsY = 0;
    int nbBinsRot = 0;

    bool ***bins;
    int binsWithSupport = 0;

    void updateBins(sample_t &sample);
};

#endif // PARTICLEFILTER_H
