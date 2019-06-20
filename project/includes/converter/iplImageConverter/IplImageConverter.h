/* ============================================================
 *
 * This file is part of the rst-converters project.
 *
 * Copyright (C) 2010 by Sebastian Wrede <swrede at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <string>

#include <boost/shared_ptr.hpp>

#include <rsb/converter/Converter.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rsc/logging/Logger.h>

#include <rst/vision/Image.pb.h>

namespace rst {
namespace converters {
namespace opencv {

/**
 * Converter for IplImages to rst.vision.Image type.
 *
 * @author jwienke
 * @author swrede
 */
class IplImageConverter: public rsb::converter::Converter<std::string> {
public:

    IplImageConverter();

    /**
     * @param grayScaleOutput if @c true, then it produces grayscale images on
     *                        deserialization.
     */
    IplImageConverter(bool grayScaleOutput);
    virtual ~IplImageConverter();

    std::string getWireSchema() const;

    std::string serialize(const rsb::AnnotatedData &data, std::string &wire);
    rsb::AnnotatedData deserialize(const std::string &wireType,
            const std::string &wire);

    /**
     * Deletes IplImages properly if used in a boost::shared_ptr.
     *
     * @author jwienke
     */
    class IplImageDeleter {
    public:
        void operator()(void *image);
    };

private:

    rsb::converter::ProtocolBufferConverter<rst::vision::Image> converter;
    rsc::logging::LoggerPtr log;
    bool grayScaleOutput;

};

}
}
}
