#ifndef ADDGAUSSIANS_H
#define ADDGAUSSIANS_H

#include "importancetype.h"

/**
 * @brief The AddGaussians class adds gaussians like the player/ROS AMCL implementation does.
 */
class AddGaussians : public ImportanceType
{
private:
    /**
     * @brief importance aggregation helper variable for importance of a scan
     */
    float importance = 1.0f;

public:

    inline void addBeamDistance(float distance) final
    {
        importance += exp( -pow(distance / sigma, 2));
    }

    inline float aggregateScanImportance() final
    {
        float ret = importance;
        // reset importance
        importance = 1.0f;

        return ret;
    }
};

#endif // ADDGAUSSIANS_H
