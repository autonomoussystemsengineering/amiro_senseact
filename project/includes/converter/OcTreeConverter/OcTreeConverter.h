#pragma once

#include <rsb/converter/Converter.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <iostream>

using namespace std;
using namespace boost;
using namespace octomap;
using namespace rsb;
using namespace rsb::converter;

namespace converter_OcTree {

/**
 * A simple converter for the OcTree object.
 */
class OcTreeConverter: public rsb::converter::Converter<std::string> {
public:
    OcTreeConverter();

    std::string serialize(const rsb::AnnotatedData& data,
            std::string& wire);

    rsb::AnnotatedData deserialize(const std::string& wireSchema,
            const std::string& wire);
};

}
