#pragma once

#include <rsb/converter/Converter.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace boost;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;

namespace muroxConverter {

/**
 * A simple converter for the Mat object.
 */
class MatConverter: public rsb::converter::Converter<std::string> {
public:
    MatConverter();

    std::string serialize(const rsb::AnnotatedData& data,
            std::string& wire);

    rsb::AnnotatedData deserialize(const std::string& wireSchema,
            const std::string& wire);
    
    /**
     * Get the depth of the pixels
     */
    int getDepth(uchar p_ucDepth);
};

}
