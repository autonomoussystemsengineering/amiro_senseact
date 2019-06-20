#ifndef IMPORTANCETYPE_H
#define IMPORTANCETYPE_H

#include "../sampleset.h"
#include <MSG.h>

class ImportanceType
{
public:
    /**
     * @brief initialImportance the initial importance for scan
     */
    float initialImportance;

    /**
     * @brief addBeamDistance adds a new single beam's distance (and maybe maps it to an importance).
     * @param distance
     */
    inline virtual void addBeamDistance(float distance) = 0;

    /**
     * @brief aggregateScanImportance after all beam distances were collected, aggregate them to a single importance for the whole scan.
     * @return the importance
     */
    inline virtual float aggregateScanImportance() = 0;

    /**
     * @brief sigma for the gaussian:
     * https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html
     * "Accuracy 60 to 1,000mm : ±10mm, 1,000 to 4,095mm : 1% of measurement"
     *
     * 4m * 0.01 = 0.04m
     */
    float sigma = 1.0f;

    /**
     * @brief normalizeImportance to [0; 1]
     * @param sample_set
     */
    inline virtual void normalizeImportance(sample_set_t *sampleSet)
    {
        float normFactor = 0.0f;

        // calculate norm factor
        for (size_t i = 0; i < sampleSet->size; ++i) {
            DEBUG_MSG(i << ": " << sampleSet->samples[i].importance);
            normFactor += sampleSet->samples[i].importance;
        }

#ifdef _DEBUG_MSG
        // debug sum (should be ~1.0f)
        float sum = 0.0f;
#endif

        // normalize each sample
        for (size_t i = 0; i < sampleSet->size; ++i) {
            sampleSet->samples[i].importance /= normFactor;

            DEBUG_MSG(i << ": " << sampleSet->samples[i].importance);
            assert(sampleSet->samples[i].importance >= 0.0f);
            assert(sampleSet->samples[i].importance <= 1.0f);

#ifdef _DEBUG_MSG
            sum += sampleSet->samples[i].importance;
#endif
        }

#ifdef _DEBUG_MSG
        DEBUG_MSG("sum: " << sum);
#endif
    }
};

#endif // IMPORTANCETYPE_H
