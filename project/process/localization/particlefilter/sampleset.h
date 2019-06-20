#ifndef SAMPLESET_H
#define SAMPLESET_H

using namespace std;

#include <cstddef>

/**
 * @brief The pose_t struct holds a pose in the map in meters and radians.
 */
struct pose_t {
    /**
     * @brief x position along the x-axis in meter.
     */
    float x;
    /**
     * @brief y position along the y-axis in meter.
     */
    float y;
    /**
     * @brief theta rotation in radians.
     */
    float theta;
};

/**
 * @brief The sample_t struct represents a particle with its pose and importance weight.
 */
struct sample_t {
    pose_t pose;
    float importance;
};

struct sample_set_t {
    size_t size;
    sample_t *samples;
    float totalWeight;
};

#endif // SAMPLESET_H
