#ifndef MULTIPLYGAUSSIANSLOG_H
#define MULTIPLYGAUSSIANSLOG_H

#include "importancetype.h"

/**
 * @brief The MultiplyGaussiansLog class mutliplies gaussians in log-space. Therefore too small floats are avoided. The transformation to "normal" space takes place when normalizing.
 */
class MultiplyGaussiansLog : public ImportanceType
{
private:
    /**
     * @brief exponent helper variable
     */
    float exponent = 0.0f;

    /**
     * @brief sigma for the gaussian:
     * https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html
     * "Accuracy 60 to 1,000mm : ±10mm, 1,000 to 4,095mm : 1% of measurement"
     *
     * 4m * 0.01 = 0.04m
     */
    const float sigma = 2.0f;

    inline float logAdd(float a, float b)
    {
        // swap variables so that b is greater than a
        if (a > b) {
            float tmp = a;
            a = b;
            b = tmp;
        }

        if (a == std::numeric_limits<float>::lowest()) {
            return b;
        }

        /*
         * log( exp(a) + exp(b) )
         *   = log( exp(b) * ( exp(a) / exp(b) + 1 ) )
         *   = b + log( exp(a) / exp(b) + 1 )
         *   = b + log( exp(a - b) + 1 )
         */
        return b + log( exp(a - b) + 1.0f );
    }

public:
    inline void addBeamDistance(float distance) final
    {
        // importance *= exp( - pow(d / sigma, 2) );
        // in log space we only save the exponent
        exponent += - pow(distance / sigma, 2);
    }

    inline float aggregateScanImportance() final
    {
        // the exponent will be transformed to "normal" space in the normalization step instead
        float ret = exponent;
        // reset it
        exponent = 0.0f;

        return ret;
    }

    inline void normalizeImportance(sample_set_t *sampleSet) final
    {
        float normFactor = std::numeric_limits<float>::lowest();

        // calculate norm factor
        for (size_t i = 0; i < sampleSet->size; ++i) {
            DEBUG_MSG(i << ": " << sampleSet->samples[i].importance);
            normFactor = logAdd(normFactor, sampleSet->samples[i].importance);
        }

#ifdef _DEBUG_MSG
        // debug sum (should be ~1.0f)
        float sum = 0.0f;
#endif

        // normalize each sample
        for (size_t i = 0; i < sampleSet->size; ++i) {
            // division in log space is substraction
            sampleSet->samples[i].importance -= normFactor;
            // now convert back to "normal" space
            sampleSet->samples[i].importance = exp(sampleSet->samples[i].importance);

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

#endif // MULTIPLYGAUSSIANSLOG_H
