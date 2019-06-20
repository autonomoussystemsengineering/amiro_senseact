#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include "sampleset.h"
#include "laserscan.h"

class SensorModel
{
public:
    virtual void computeWeight(sample_t &sample, laserscan_t &scan) = 0;
    virtual void normalizeWeights(sample_set_t *sampleSet) = 0;
};

#endif // SENSORMODEL_H
