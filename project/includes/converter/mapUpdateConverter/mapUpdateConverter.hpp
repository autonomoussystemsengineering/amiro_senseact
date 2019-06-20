#pragma once

#include <rsb/converter/Converter.h>
#include <opencv2/core/core.hpp>
#include "mapUpdate.hpp"
// #include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace boost;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;

namespace muroxConverter {

/**
 * A simple converter for the Mat object.
 */
class MapUpdateConverter: public rsb::converter::Converter<std::string> {
public:
	MapUpdateConverter();

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
