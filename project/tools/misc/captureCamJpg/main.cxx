#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

// For checking character pressed in the console
#include <kbhit.hpp>

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
using namespace rsb::converter;


#include <jpeglib.h>
// #include <libv4l2.h>

// #include "sandan-v4l.h"

//int framerate = 25;
//int deliveredFrames = 0;

int v4l2_compress_jpeg(int width, int height, unsigned char *src, int src_size, unsigned char *dest, int dest_size) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *buf = dest;
    unsigned long buf_size = dest_size;
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;

    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 85, FALSE);

    jpeg_mem_dest(&cinfo, &buf, &buf_size);

    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;

    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &src[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);

    jpeg_destroy_compress(&cinfo);

    if (buf != dest) {
        printf("Destination memory to small (dest: %px%d, new: %px%ld)\n", dest,
                dest_size, buf, buf_size);
        return 0;
    }

    return buf_size;
}



int main() {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::getFactory();

  // Create the informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope("/image/jpg"));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video0
  if ( cam.open(0) ) {
    // Allocate a frame object to store the picture
    cv::Mat frame;
    // Buffer to store the compressed image
    
    
    // Process the cam forever
    for (; ;) {

      // Save the actual picture to the frame object
      cam >> frame;
      
      // Ask for any keystroke, to quit the capturing
      // Info: cv::waitKey() is useless here, because it wont work in a terminal
      if( kbhit() ) {
//         int KB_code = getchar();
//         cout << "KB_code = " << KB_code << endl;
        break;
      }

      // Compress image
//        v4l2_compress_jpeg(frame->cols, frame->rows, (unsigned char*) frame->data, frame->cols*frame->rows*3, (unsigned char*) &((*frameJpg)[0]), frameJpgSize);
      // Set compression parameters
      vector<uchar> buf;
      vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(20);
      
       imencode(".jpg", frame, buf, compression_params);
//        cv::imshow("Camera",*frame);
//        cv::waitKey(10);

      // Send the data.
      shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
      informer->publish(frameJpg);

    }
  }

  // Free the cam
  cam.release();

  return 0;

}
