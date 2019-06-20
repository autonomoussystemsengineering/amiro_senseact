#ifndef LASERSCAN_H
#define LASERSCAN_H

using namespace std;

#include <cstddef>

struct laserscan_t {
    float *values;
    float *sin;
    float *cos;
    size_t size;
};

#endif // LASERSCAN_H
