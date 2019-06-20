/* ============================================================
 *
 * This file is part of the rst-converters project.
 *
 * Copyright (C) 2010 by Sebastian Wrede <swrede at techfak dot uni-bielefeld dot de>
 * Copyright (C) 2010 by Johannes Wienke <jwienke at techfak dot uni-bielefeld dot de>
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

#include "IplImageConverter.h"

#include "rst/vision/Image.pb.h"

#include <rsb/converter/Converter.h>
#include <rsb/converter/SerializationException.h>
#include <rsc/logging/Logger.h>
#include <rsc/logging/LoggerFactory.h>

#include <opencv/cv.h>

using namespace std;
using namespace rsc::logging;

namespace rst {
namespace converters {
namespace opencv {

void IplImageConverter::IplImageDeleter::operator()(void *image) {
    IplImage *i = (IplImage *) image;
    cvReleaseImage(&i);
}

// we use a macro here to have inlining even in debug mode because otherwise the
// converter is horribly slow.
// TODO there is more chance for optimization. Right now the passed expression
// probably needs to be evaluated 3 times.
#define clip(val)(((val) > 255) ? 255 : ((val) < 0 ? 0 : (val)));

static void convertYUV422ToBGR(int width, int height,
        const unsigned char* imageDataIn, unsigned char* imageDataOut) {

    // integer implementation (4.9 times faster) - PEV
    unsigned char * ptuc_imageDataOut = (unsigned char *) imageDataOut;
    const unsigned char * ptuc_imageDataIn = (const unsigned char *) imageDataIn;
    int widthMax = width * 2;
    int widthStep = 4;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < widthMax; i += widthStep) {
            int C = ptuc_imageDataIn[i] - 16;
            int D = ptuc_imageDataIn[i + 1] - 128;
            int C2 = ptuc_imageDataIn[i + 2] - 16;
            int E = ptuc_imageDataIn[i + 3] - 128;
            register int valC1 = 298 * C;
            register int valC2 = 298 * C2;
            register int valB = 517 * D;
            register int valG = -208 * D - 100 * E;
            register int valR = 409 * E;
            *ptuc_imageDataOut++ = clip((valC1 + valB + 128) >> 8);
            *ptuc_imageDataOut++ = clip((valC1 + valG + 128) >> 8);
            *ptuc_imageDataOut++ = clip((valC1 + valR + 128) >> 8);
            *ptuc_imageDataOut++ = clip((valC2 + valB + 128) >> 8);
            *ptuc_imageDataOut++ = clip((valC2 + valG + 128) >> 8);
            *ptuc_imageDataOut++ = clip((valC2 + valR + 128) >> 8);
        }

        ptuc_imageDataIn += widthMax;

    }

}

static void logIplImageMetadata(LoggerPtr log, IplImage* image) {
    if (log->isTraceEnabled()) {
        RSCTRACE(log, "Serializing IplImage with the following properties:");
        RSCTRACE(log, "Size: " << image->imageSize);
        RSCTRACE(log, "Depth: " << image->depth);
        RSCTRACE(log, "Size (WxH): " << image->width << "x" << image->height);
        RSCTRACE(log, "#Channels: " << image->nChannels);
        RSCTRACE(log, "Colormodel: " << image->colorModel);
        RSCTRACE(log, "Data order (0==Interleaved): " << image->dataOrder);
        RSCTRACE(log, "Width-Step: " << image->widthStep);
    }
}

IplImageConverter::IplImageConverter() :
        rsb::converter::Converter<string>("unused", RSB_TYPE_TAG(IplImage)), grayScaleOutput(
                false) {
    log = Logger::getLogger("IplImageConverter");
}

IplImageConverter::IplImageConverter(bool grayScaleOutput) :
        rsb::converter::Converter<string>("unused", RSB_TYPE_TAG(IplImage)), grayScaleOutput(
                grayScaleOutput) {
    log = Logger::getLogger("IplImageConverter");
}

IplImageConverter::~IplImageConverter() {
}

string IplImageConverter::getWireSchema() const {
    return converter.getWireSchema();
}

string IplImageConverter::serialize(const rsb::AnnotatedData &data,
        string &wire) {

    assert(data.first == getDataType());

    boost::shared_ptr<IplImage> image = boost::static_pointer_cast<IplImage>(
            data.second);

    logIplImageMetadata(log, image.get());

    rst::vision::Image message;
    message.set_width(image->width);
    message.set_height(image->height);

    // assumptions about color modes based on channels
    switch (image->nChannels) {
    case 1:
        message.set_color_mode(rst::vision::Image::COLOR_GRAYSCALE);
        break;
    case 3:
        message.set_color_mode(rst::vision::Image::COLOR_BGR);
        break;
    default:
        throw rsb::converter::SerializationException(
                boost::str(
                        boost::format("Unsupported number of channels: %1%")
                                % image->nChannels));
        break;
    }

    message.set_channels(image->nChannels);

    // color space
    switch (image->depth) {
    case IPL_DEPTH_8U:
        message.set_depth(rst::vision::Image::DEPTH_8U);
        break;
    case IPL_DEPTH_16U:
        message.set_depth(rst::vision::Image::DEPTH_16U);
        break;
    case IPL_DEPTH_32F:
        message.set_depth(rst::vision::Image::DEPTH_32F);
        break;
    default:
        throw rsb::converter::SerializationException(
                boost::str(
                        boost::format("Unknown image depth for IplImage: %1%")
                                % image->depth));
    }

    // TODO depth check is a hack and we need a real way to determine alpha
    if (image->widthStep == (image->width * 4) && image->depth == IPL_DEPTH_8U) {
        RSCDEBUG(log,
                "Detected IplImage with alpha channel information which will be discarded upon serialization.");
        IplImage* tempImage = cvCreateImage(cvGetSize(image.get()),
                image->depth, image->nChannels);
        cvCvtColor(image.get(), tempImage, CV_BGRA2BGR);
        message.set_data((char *) tempImage->imageData, tempImage->imageSize);
        cvReleaseImage(&tempImage);
    } else {
        message.set_data((char *) image->imageData, image->imageSize);
    }

    // data order
    if (image->dataOrder == 0) {
        message.set_data_order(rst::vision::Image::DATA_INTERLEAVED);
        RSCDEBUG(log, "Interleaved data detected");
    } else {
        message.set_data_order(rst::vision::Image::DATA_SEPARATE);
        RSCDEBUG(log, "Separate data planes detected");
    }

    string buffer = converter.serialize(
            make_pair(rsc::runtime::typeName<rst::vision::Image>(),
                    boost::shared_ptr<void>(&message,
                            rsc::misc::NullDeleter())), wire);

    return buffer;

}

rsb::AnnotatedData IplImageConverter::deserialize(const std::string &wireType,
        const std::string &wire) {

    boost::shared_ptr<rst::vision::Image> message = boost::static_pointer_cast<
            rst::vision::Image>(converter.deserialize(wireType, wire).second);

    RSCDEBUG(log,
            "De-serializing rst.vision.Image. Color model: " << message->color_mode() << ", Width: " << message->width() << ", Height: " << message->height() << ", Ordering: " << message->data_order());

    // TODO support other types
    set<rst::vision::Image::ColorMode> supportedModes;
    supportedModes.insert(rst::vision::Image::COLOR_YUV422);
#if CV_MAJOR_VERSION >= 2
#if CV_MINOR_VERSION >= 3
    supportedModes.insert(rst::vision::Image::COLOR_YUV);
#endif
#endif
    supportedModes.insert(rst::vision::Image::COLOR_BGR);
    supportedModes.insert(rst::vision::Image::COLOR_RGB);
    supportedModes.insert(rst::vision::Image::COLOR_GRAYSCALE);
    if (supportedModes.count(message->color_mode()) != 1) {
        stringstream s;
        s << "Cannot deserialize images of type ";
        s << rst::vision::Image::ColorMode_Name(message->color_mode());
        s << ". ";
        s << "Supported modes:";
        for (set<rst::vision::Image::ColorMode>::const_iterator modeIt =
                supportedModes.begin(); modeIt != supportedModes.end();
                ++modeIt) {
            s << " " << rst::vision::Image::ColorMode_Name(*modeIt);
        }
        throw rsb::converter::SerializationException(s.str());
    }

    // XXX we use the fact that depth is initialized like opencv's structures
    IplImage *image;

    // TODO jwienke: this whole optimization should be factored out to some
    // other class
    if (this->grayScaleOutput) {
        image = cvCreateImageHeader(cvSize(message->width(), message->height()),
                IPL_DEPTH_8U, 1);

        if (message->color_mode() == rst::vision::Image::COLOR_YUV422) {

            cvCreateData(image);

            unsigned char * dst = (unsigned char*) image->imageData;
            const unsigned char * src = (unsigned char*) message->data().c_str();
            const unsigned char * end = src
                    + message->width() * message->height() * 2;
            for (; src < end; ++dst, src += 2) {
                // FIXME once Aldebaran fixes wrong YUV output, the value (255-16)
                // should be replaced with 219
                *dst = (unsigned char) (((unsigned short) (*src - 16)) * 255
                        / (255 - 16));
            }

        } else if (message->color_mode() == rst::vision::Image::COLOR_BGR) {
            assert(message->channels() == 3);

            cvCreateData(image);

            IplImage* tempImage = cvCreateImageHeader(cvSize(message->width(), message->height()), IPL_DEPTH_8U, 3);
            cvSetData(tempImage, const_cast<char*> (message->data().c_str()), 3 * message->width());
            cvCvtColor(tempImage, image, CV_BGR2GRAY);
            cvReleaseImageHeader(&tempImage);

        } else {
            throw rsb::converter::SerializationException(
                    "Cannot deserialize (and convert to gray) images with color mode "
                            + rst::vision::Image::ColorMode_Name(
                                    message->color_mode()));
        }

    } else {

        int depth = 0;
        switch (message->depth()) {
        case rst::vision::Image::DEPTH_8U:
            depth = IPL_DEPTH_8U;
            break;
        case rst::vision::Image::DEPTH_16U:
            depth = IPL_DEPTH_16U;
            break;
        case rst::vision::Image::DEPTH_32F:
            depth = IPL_DEPTH_32F;
            break;
        default:
            throw rsb::converter::SerializationException(
                    boost::str(
                            boost::format("Unsupported image depth: %1%")
                                    % rst::vision::Image::Depth_Name(
                                            message->depth())));
            break;
        }
        int nChannels = message->channels();
        // in case of YUV422 we will do a conversion internally
        if (message->color_mode() == rst::vision::Image::COLOR_YUV422) {
            nChannels = 3;
        }

        image = cvCreateImage(cvSize(message->width(), message->height()),
                depth, nChannels);

        switch (message->color_mode()) {
        case rst::vision::Image::COLOR_RGB:
        case rst::vision::Image::COLOR_BGR:
        case rst::vision::Image::COLOR_GRAYSCALE:
            RSCDEBUG(log, "Deserializing BGR Image.");
            memcpy(image->imageData, const_cast<char*>(message->data().c_str()),
                    message->data().size());
            break;
        case rst::vision::Image::COLOR_YUV422:
            convertYUV422ToBGR(message->width(), message->height(),
                    (const unsigned char *) message->data().c_str(),
                    (unsigned char*) image->imageData);
            break;
#if CV_MAJOR_VERSION >= 2
#if CV_MINOR_VERSION >= 3
        case rst::vision::Image::COLOR_YUV: {
            IplImage *tempImage = cvCreateImageHeader(
                    cvSize(message->width(), message->height()),
                    message->depth(), 3);
            tempImage->imageData = const_cast<char*>(message->data().c_str());
            cvCvtColor(tempImage, image, CV_YUV2BGR);
            cvReleaseImageHeader(&tempImage);
            break;
        }
#endif
#endif
        default:
            // this should have already been checked on top of this function
            assert(false);
            throw rsb::converter::SerializationException(
                    "Cannot deserialize images with color mode "
                            + rst::vision::Image::ColorMode_Name(
                                    message->color_mode()));
        }

    }

    logIplImageMetadata(log, image);

    boost::shared_ptr<IplImage> s(image, IplImageDeleter());

    return rsb::AnnotatedData(getDataType(), s);

}

}
}
}
