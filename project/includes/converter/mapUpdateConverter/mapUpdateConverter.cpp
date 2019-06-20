/*
 * mapUpdateConverter.cpp
 *
 *  Created on: Dec 12, 2014 
 *      Author: fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
 */

#include "mapUpdateConverter.hpp"

namespace muroxConverter {

// We have to pass two arguments to the base-class constructor:
// 1. The data-type
// 2. The wire-schema
//
// Note: this could also be written as
// Converter<string>("matrix-image", RSB_TYPE_TAG(Mat))
// to infer the "string" name of the data-type using RTTI.
MapUpdateConverter::MapUpdateConverter() :
    Converter<string> ("MapUpdate", "mapUpdate", true) {
}

int MapUpdateConverter::getDepth(uchar p_ucDepth) {
  switch ( p_ucDepth ) {
      case CV_8U:  return(1); // "8U"; break;
      case CV_8S:  return(1); // "8S"; break;
      case CV_16U: return(2); // "16U"; break;
      case CV_16S: return(2); // "16S"; break;
      case CV_32S: return(4); // "32S"; break;
      case CV_32F: return(4); // "32F"; break;
      case CV_64F: return(8); // "64F"; break;
      default:     return(1); // "User"; break;
  }
}

string MapUpdateConverter::serialize(const AnnotatedData& data, string& wire) {
    // Ensure that DATA actually holds a datum of the data-type we
    // expect.
    assert(data.first == getDataType());

    // Force conversion to the expected data-type.
    boost::shared_ptr<const MapUpdate> matImage =
            static_pointer_cast<const MapUpdate> (data.second);

    // Prepare the sending data

    // Get the pose and store it
    float x = matImage->x;
    float y = matImage->y;
    float theta = matImage->theta;

    // Get the information of the matrix and store it
    int iCols = matImage->cols;
    int iRows = matImage->rows;
    int iType = matImage->type();

    // extract the information regarding image type
    int   iDepth = this->getDepth((uchar) (iType & CV_MAT_DEPTH_MASK));
    uchar ucChannels = 1 + (iType >> CV_CN_SHIFT);
    
    // Store the content to the selected binary layout.
    // Define the amount of sending data
    // Allocate pose (3 float -> 3*4), cols (int -> 4), rows (int -> 4), type (int -> 4) and the data in bytes
    ssize_t uiImageSize = iCols * iRows * iDepth * ucChannels;
    wire.resize( 4 + 4 + 4 + 4 + 4 + 4 + uiImageSize );

    // Write the pose
    copy(((char*) &x), ((char*) &x) + 4, wire.begin());
    copy(((char*) &y), ((char*) &y) + 4, wire.begin() + 4);
    copy(((char*) &theta), ((char*) &theta) + 4, wire.begin() + 8);
    // Write the cols
    copy(((char*) &iCols), ((char*) &iCols) + 4, wire.begin() + 12);
    // Write the rows
    copy(((char*) &iRows), ((char*) &iRows) + 4, wire.begin() + 16);
    // Write the type
    copy(((char*) &iType), ((char*) &iType) + 4, wire.begin() + 20);
    // Write the data
    copy(matImage->data, matImage->data + uiImageSize, wire.begin() + 24 );


    // Return the wire-schema of the serialized representation in
    // WIRE.
    return getWireSchema(); // this->getWireSchema() == "matrix-image"
}

AnnotatedData MapUpdateConverter::deserialize(const string& wireSchema,
        const string& wire) {
    // Ensure that WIRE uses the expected wire-schema.
    assert(wireSchema == getWireSchema()); // this->getWireSchema() == "matrix-image"

    // Allocate a new SimpleImage object and set its data members from
    // the content of WIRE.
    
    // Get the pose
    const float x = *((float*) &*wire.begin());
    const float y = *((float*) &*(wire.begin() + 4));
    const float theta = *((float*) &*(wire.begin() + 8));
    // Get the cols
    const int   iCols = *((int*)   &*(wire.begin() + 12));
    // Get the rows
    const int   iRows = *((int*)   &*(wire.begin() + 16));
    // Get the type
    const int   iType = *((int*)   &*(wire.begin() + 20));

    // Create the image and copy the pointer to the date
//     cv::Mat * matImage = new cv::Mat( iRows, iCols, iType);
//     matImage->data = ((uchar*)   &*(wire.begin() + 12));
//     cv::Mat * matImage = new cv::Mat( Size(iCols, iRows), iType, ((void*)   &*(wire.begin() + 12)) );
    MapUpdate matImage( Size(iCols, iRows), iType, ((void*)   &*(wire.begin() + 24)));
    
    boost::shared_ptr<MapUpdate> frame(new MapUpdate( matImage.clone() ));
    frame->x = x;
    frame->y = y;
    frame->theta = theta;

// Swapping the BGR to RGB if the color is not correct
// #ifndef __arm__
//     cv::cvtColor(*frame, *frame, CV_RGB2BGR);
// #endif
    
    

    // Return (a shared_ptr to) the constructed object along with its
    // data-type.
//     return make_pair(getDataType(), boost::shared_ptr<cv::Mat> (matImage));
    return make_pair(getDataType(), frame);
}

}



