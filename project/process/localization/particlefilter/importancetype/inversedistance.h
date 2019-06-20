#ifndef INVERSEDISTANCE_H
#define INVERSEDISTANCE_H

#include "importancetype.h"

class InverseDistance : public ImportanceType
{
private:
    /**
     * @brief numberOfBeams the number of beams in the scan
     */
    unsigned int numberOfBeams = 0;

    /**
     * @brief totalDistance sum of all distances of current scan
     */
    float totalDistance = 0.0f;

    float maximumImportance = std::numeric_limits<float>::max() / 2;

public:
    void addBeamDistance(float distance)
    {
        totalDistance += distance;
        numberOfBeams++;
    }

    float aggregateScanImportance()
    {
        if (!numberOfBeams) {
            // not a single valid beam
            totalDistance = 0.0f;
            return 0.0f;
        }

        if (totalDistance <= 0.0f) {
            // must be a perfect scan
            numberOfBeams = 0;
            totalDistance = 0.0f;
            return maximumImportance;
        }

        // use the average distance instead of sum, so the importance does not become too small
        float averageDistance = totalDistance / numberOfBeams;
        numberOfBeams = 0;
        totalDistance = 0.0f;
        return 1.0f / averageDistance;
    }
};

#endif // INVERSEDISTANCE_H
