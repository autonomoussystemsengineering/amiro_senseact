#ifndef RAYCASTINGMODEL_H
#define RAYCASTINGMODEL_H

#include "sensormodel.h"
#include "map.h"

class RayCastingModel : public SensorModel
{
public:
    RayCastingModel(Map *map);
    void computeWeight(sample_t &sample, const rst::vision::LocatedLaserScan &scan);
    void normalizeWeights(sample_set_t *sampleSet);

private:
    Map *map;
    float normFactor = 0;

    vector<float> simulateScan(const pose_t &pose, const rst::vision::LocatedLaserScan &scanConfig);
    float simulateRay(const pose_t &pose, float globalAngle, const rst::vision::LocatedLaserScan &scanConfig);
};

#endif // RAYCASTINGMODEL_H
