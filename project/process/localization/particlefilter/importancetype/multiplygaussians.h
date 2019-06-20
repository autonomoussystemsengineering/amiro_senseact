#ifndef MULTIPLYGAUSSIANS_H
#define MULTIPLYGAUSSIANS_H

#include "importancetype.h"

/**
 * @brief The MultiplyGaussians class naively multiplies gaussians. This leads to very small numbers that can't be represented by float.
 */
class MultiplyGaussians : public ImportanceType
{
private:
    /**
     * @brief exponent helper variable
     */
    float exponent = 0.0f;

public:
    /**
     * @brief sigma for the gaussian:
     * https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx.html
     * "Accuracy 60 to 1,000mm : ±10mm, 1,000 to 4,095mm : 1% of measurement"
     *
     * 4m * 0.01 = 0.04m
     */
    float sigma = 2.0f;

    inline void addBeamDistance(float distance) final
    {
        // importance *= exp( - pow(d / sigma, 2) );
        // just save the exponent, since exp(a) * exp(b) = exp(a + b), exp needs only to be applied once
        exponent += - pow(distance / sigma, 2);
    }

    inline float aggregateScanImportance() final
    {
        // now apply the exponential function to the exponent
        float ret = exp(exponent);
        // and reset it
        exponent = 0.0f;

        return ret;
    }
};

#endif // MULTIPLYGAUSSIANS_H
