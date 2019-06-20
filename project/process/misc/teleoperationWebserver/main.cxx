/*
 * Author: Markus Gaffke, Timo Korthals
 * File: main.cxx
 * Purpose: Providing the Steering Website for the BeBot/Amiro. 
 *          Furthermore sending the cameradata and infrareddata via Websocketprotocol
 *        to the website. Incoming commands are computed and send via RSB to the 
 *      respective scope. For light signals -> starting respective script.
 * Last Changed: 22.05.2015
 *
 */
#ifdef CMAKE_BUILD
#include "lws_config.h"
#endif

// For running system comands
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <pthread.h>
#include <chrono>

#ifdef _WIN32
#include <io.h>
#ifdef EXTERNAL_POLL
#define poll WSAPoll
#endif
#else
#include <syslog.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <iostream>
#include <libwebsockets.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Video 4 Linux
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <fcntl.h>

// Print nice messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// RSB
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>

// initialize CAN
#include <ControllerAreaNetwork.h>
ControllerAreaNetwork myCAN;

#include <Color.h>  // Color types

// maximum number of chars acepted to be read from ws client
#define MAX_READ 155

// polling intervall
#define SERVICE_TIMEOUT 50
// the macimum number of connections allowed to the lws server
#define SERVICE_MAXIMUM_SESSIONS 100

// Libwebsockets specific variables
int max_poll_elements;
static int close_testing;

struct pollfd *pollfds;
int *fd_lookup;
int count_pollfds;
static volatile int force_exit = 0;
static struct libwebsocket_context *context;

using namespace std;
static string textStringToSend;
static string lightMessage;

// Buffer for the camera picture. Is used during the whole time and only 
// freed when the server is closed. Used in Camera-Protocoll.
// maximum picture size is 57600
unsigned char bufferCam[LWS_SEND_BUFFER_PRE_PADDING + 57600
    + LWS_SEND_BUFFER_POST_PADDING];

// variables for picutre
cv::VideoCapture cam;
bool frameReceived = false;

//Speedvariable
int speed_for_robot = 50;
int speed_for_robot_X = 50;
int speed_for_robot_Y = 50;

//Number of users
int static clientcount = 0;
int static clientcount_beforereset = 0;
int static userIndex = 1000;

//Lights
bool white = false, red = false, green = false, blue = false;


// Some namespaces
using namespace boost;
using namespace std;

////////////////////////rsb/////////////////////
// Namespace which belong to rsb
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// Scopes
std::string g_sInScopeImage = "/IMAGES";
std::string g_sOutScopeImage = "/IMAGES";
std::string g_sInScopeIr = "/IR";
std::string g_sOutScopeSteering = "/STEERING";

// Get the RSB factory
rsb::Factory& factory = rsb::Factory::getInstance();

// Register new converter for std::vector<int>
boost::shared_ptr<vecIntConverter> converterVecInt;

// Register new converter for cv::Mat
boost::shared_ptr<MatConverter> converterMat;

// Prepare RSB informer
rsb::Informer<std::vector<int> >::Ptr informer_vec;

// Calculate the new steering (two elements with 0 initialized)
boost::shared_ptr<std::vector<int> > vecSteering;

// Listener for the IR data
rsb::ListenerPtr listener;

// Holds the IR data vector which is received by the listener
boost::shared_ptr<std::vector<int> > vecIrData;

// Listener for the camera data
rsb::ListenerPtr listenerCamera;

// Holds the compressed image of the camera listener
std::string receivedFrame;

// listener function for the frames
void listenCamera(rsb::EventPtr event) {
  if (frameReceived == false) {
    // Get the pointer
    boost::shared_ptr<std::string> receivedFramePointer =
        static_pointer_cast<std::string>(event->getData());
    // Copy the image
    receivedFrame = *receivedFramePointer;
    frameReceived = true;
  }
}

// Declaration of the thread which opens the camera
// Note: The camera will be closed on SYSINT in
// main, after the thread was killed
void * informerCamera(void * argument) {

  // Create an explicit inprocess config
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
  // Disable socket
  {
    // Get the options for socket transport, because we want to change them
    rsc::runtime::Properties tmpProp = tmpPartConf.mutableTransport(
        "socket").getOptions();

    // disable socket transport
    std::string enabled = "0";
    tmpProp["enabled"] = boost::any(enabled);

    // Write the socket tranport properties back to the participant config
    tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
  }
  // Disable spread
  {
    // Get the options for spread transport, because we want to change them
    rsc::runtime::Properties tmpProp = tmpPartConf.mutableTransport(
        "spread").getOptions();

    // disable socket transport
    std::string enabled = "0";
    tmpProp["enabled"] = boost::any(enabled);

    // Write the spread tranport properties back to the participant config
    tmpPartConf.mutableTransport("spread").setOptions(tmpProp);
  }
  {
    // Get the options for inprocess transport, because we want to change them
    rsc::runtime::Properties tmpProp = tmpPartConf.mutableTransport(
        "inprocess").getOptions();

    // disable socket transport
    std::string enabled = "1";
    tmpProp["enabled"] = boost::any(enabled);

    // Write the inprocess tranport properties back to the participant config
    tmpPartConf.mutableTransport("inprocess").setOptions(tmpProp);
  }

  // Informer for sending the compressed image
  rsb::Informer<std::string>::Ptr informer = factory.createInformer<
      std::string>(g_sOutScopeImage, tmpPartConf);

  // Open the camera device
  lwsl_info("Open Camera\n");
  int resultCamOpen;
  
#ifdef __arm__
  
        struct v4l2_format fmt;
        int rc, fd = -1;
        unsigned int i, length;
        const char *dev_name = std::string("/dev/video6").c_str();
        char out_name[256];
        FILE *fout;
        void *buffer;
        
        fd = v4l2_open(dev_name, O_RDWR, 0);
        if (fd < 0) {
                fprintf(stderr, "Cannot open device %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        
//         rc = v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt);
//         printf("Resolution: %d x %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
//         printf("format: %d \n", fmt.fmt.pix.pixelformat);
//         printf("field: %d \n", fmt.fmt.pix.field);
//         return 0;
        
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640;  // Maximum width
        fmt.fmt.pix.height      = 480;  // Maximum hight
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;//V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        rc = v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt);
        printf("Resolution: %d x %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
        printf("format: %d \n", fmt.fmt.pix.pixelformat);
        printf("field: %d \n", fmt.fmt.pix.field);
        if (rc == -1) {
                fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
//         if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
//                 fprintf(stderr, "Error: Libv4l did not accept format\n");
//                 exit(EXIT_FAILURE);
//         }
//      if ((fmt.fmt.pix.width != 320) || (fmt.fmt.pix.height != 240))
//              printf("Warning: driver is sending image at %dx%d\n",
//                      fmt.fmt.pix.width, fmt.fmt.pix.height);

        if (fmt.fmt.pix.sizeimage < (fmt.fmt.pix.width * fmt.fmt.pix.height)) {
                fprintf(stderr, "Error: Driver is sending image at %dx%d with size of %d\n",
                        fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
                resultCamOpen = false;
        }

        buffer = malloc(fmt.fmt.pix.sizeimage);
        if (buffer == NULL) {
                fprintf(stderr, "Cannot allocate buffer\n");
                resultCamOpen = false;
        }
        
        resultCamOpen = true;


  // Allocate a frame object to store the picture
   cv::Mat data(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC3);
   cv::Mat frameUYVY(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC2);
#else
  
  resultCamOpen = cam.open(0);
  INFO_MSG("Open Cam 0")
  // Holds the image from the camera
  cv::Mat data;
#endif
  
  if (resultCamOpen) {
    // Holds the compressed image
    vector<uchar> buf;
    // Define compression parameters
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(20); // that's percent, so 100 == no compression, 1 == full

    for (;;) {
      // Get the new frame
#ifdef __arm__
      length = v4l2_read(fd, (void*) frameUYVY.data , fmt.fmt.pix.sizeimage);
      if (length == -1) {
              fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
              exit(EXIT_FAILURE);
      }
      cvtColor(frameUYVY,data, CV_YUV2BGR_YUY2);
#else
      cam >> data;
#endif 
      if(!data.empty()){
      imencode(".jpg", data, buf, compression_params);
        
      //INFO_MSG("after imencode")
      // Send the data.
      boost::shared_ptr<std::string> frameJpg(
          new std::string(buf.begin(), buf.end()));
      // Publish data
      informer->publish(frameJpg);
      }
    }
  }

  pthread_exit(0);
}

// Listener for the IR data, which copies
// the received data of the IR information
void receiveIrData(rsb::EventPtr event) {
  vecIrData = static_pointer_cast<std::vector<int> >(event->getData());
  // We assume AMiRo proximity values 0x0 .. 0xFFFF
  for (unsigned int idx = 0; idx < vecIrData->size(); ++idx) {
    vecIrData->at(idx) = vecIrData->at(idx) / 655; // we assume 65535 but we need values from 0..100
  }
}

////////////////////////////////////////////////

char *resource_path; // = LOCAL_RESOURCE_PATH;

/*
 * We take a strict whitelist approach to stop ../ attacks
 */

struct serveable {
  const char *urlpath;
  const char *mimetype;
};

struct per_session_data__http {
  int fd;
};

//checking for special types of data like HTML-Site, PNG and CSS
const char * get_mimetype(const char *file) {
  int n = strlen(file);
  INFO_MSG(file)
  if (n < 5)

    if (!strcmp(&file[n - 4], ".ico"))
      return "image/x-icon";

  if (!strcmp(&file[n - 4], ".jpg"))
    return "image/jpg";
  if (!strcmp(&file[n - 4], ".png"))
    return "image/png";
  if (!strcmp(&file[n - 4], ".css"))
    return "text/css";
  if (!strcmp(&file[n - 5], ".html"))
    return "text/html";
  if (!strcmp(&file[n - 3], ".js"))
    return "text/javascript";

  return NULL;
}

/* this protocol server (always the first one) just knows how to do HTTP */

static int callback_http(struct libwebsocket_context *context,
    struct libwebsocket *wsi, enum libwebsocket_callback_reasons reason,
    void *user, void *in, size_t len) {
#if 0
  char client_name[128];
  char client_ip[128];
#endif
  char buf[256];
  int n, m;
  static unsigned char buffer[4096];
  struct per_session_data__http *pss = (struct per_session_data__http *) user;
  const char *mimetype;
#ifdef EXTERNAL_POLL
  struct libwebsocket_pollargs *pa = (struct libwebsocket_pollargs *)in;
#endif

  switch (reason) {
  case LWS_CALLBACK_HTTP:

    if (len < 1) {
      libwebsockets_return_http_status(context, wsi,
          HTTP_STATUS_BAD_REQUEST, NULL);
      return -1;
    }

    /* if a legal POST URL, let it continue and accept data */
    if (lws_hdr_total_length(wsi, WSI_TOKEN_POST_URI))
      return 0;

    /* if not, send a file the easy way */
    strcpy(buf, resource_path);
    if (strcmp((const char *) in, "/")) {
      if (*((const char *) in) != '/')
        strcat(buf, "/");
      strncat(buf, (const char *) in,
          sizeof(buf) - strlen(resource_path));
    } else
      /* default file to serve */
      strcat(buf, "/project.html");
    buf[sizeof(buf) - 1] = '\0';

    /* refuse to serve files we don't understand */
    mimetype = get_mimetype(buf);
    if (!mimetype) {
      lwsl_err("Unknown mimetype for %s\n", buf);
      libwebsockets_return_http_status(context, wsi,
          HTTP_STATUS_UNSUPPORTED_MEDIA_TYPE, NULL);
      return -1;
    }

    if (libwebsockets_serve_http_file(context, wsi, buf, mimetype,
        "text/html"))
      return -1; /* through completion or error, close the socket */

    /*
     * notice that the sending of the file completes asynchronously,
     * we'll get a LWS_CALLBACK_HTTP_FILE_COMPLETION callback when
     * it's done
     */

    break;

  case LWS_CALLBACK_HTTP_BODY:
    strncpy(buf, (const char *) in, 20);
    buf[20] = '\0';
    if (len < 20)
      buf[len] = '\0';

    lwsl_notice("LWS_CALLBACK_HTTP_BODY: %s... len %d\n",
        (const char * )buf, (int )len);

    break;

  case LWS_CALLBACK_HTTP_BODY_COMPLETION:
    lwsl_notice("LWS_CALLBACK_HTTP_BODY_COMPLETION\n");
    /* the whole of the sent body arried, close the connection */
    libwebsockets_return_http_status(context, wsi, HTTP_STATUS_OK, NULL);

    return -1;

  case LWS_CALLBACK_HTTP_FILE_COMPLETION:
//    lwsl_info("LWS_CALLBACK_HTTP_FILE_COMPLETION seen\n");
    /* kill the connection after we sent one file */
    return -1;

  case LWS_CALLBACK_HTTP_WRITEABLE:
    /*
     * we can send more of whatever it is we were sending
     */

    do {
      n = read(pss->fd, buffer, sizeof buffer);
      /* problem reading, close conn */
      if (n < 0)
        goto bail;
      /* sent it all, close conn */
      if (n == 0)
        goto flush_bail;
      /*
       * because it's HTTP and not websocket, don't need to take
       * care about pre and postamble
       */
      m = libwebsocket_write(wsi, buffer, n, LWS_WRITE_HTTP);
      if (m < 0)
        /* write failed, close conn */
        goto bail;
      if (m != n)
        /* partial write, adjust */
        lseek(pss->fd, m - n, SEEK_CUR);

      if (m) /* while still active, extend timeout */
        libwebsocket_set_timeout(wsi, PENDING_TIMEOUT_HTTP_CONTENT, 5);

    } while (!lws_send_pipe_choked(wsi));
    libwebsocket_callback_on_writable(context, wsi);
    break;
    flush_bail:
    /* true if still partial pending */
    if (lws_send_pipe_choked(wsi)) {
      libwebsocket_callback_on_writable(context, wsi);
      break;
    }

    bail: close(pss->fd);
    return -1;

    /*
     * callback for confirming to continue with client IP appear in
     * protocol 0 callback since no websocket protocol has been agreed
     * yet.  You can just ignore this if you won't filter on client IP
     * since the default uhandled callback return is 0 meaning let the
     * connection continue.
     */

  case LWS_CALLBACK_FILTER_NETWORK_CONNECTION:
#if 0
    libwebsockets_get_peer_addresses(context, wsi, (int)(long)in, client_name,
        sizeof(client_name), client_ip, sizeof(client_ip));

    fprintf(stderr, "Received network connect from %s (%s)\n",
        client_name, client_ip);
#endif
    /* if we returned non-zero from here, we kill the connection */
    break;

#ifdef EXTERNAL_POLL
    /*
     * callbacks for managing the external poll() array appear in
     * protocol 0 callback
     */

    case LWS_CALLBACK_LOCK_POLL:
    /*
     * lock mutex to protect pollfd state
     * called before any other POLL related callback
     */
    break;

    case LWS_CALLBACK_UNLOCK_POLL:
    /*
     * unlock mutex to protect pollfd state when
     * called after any other POLL related callback
     */
    break;

    case LWS_CALLBACK_ADD_POLL_FD:

    if (count_pollfds >= max_poll_elements) {
      lwsl_err("LWS_CALLBACK_ADD_POLL_FD: too many sockets to track\n");
      return 1;
    }

    fd_lookup[pa->fd] = count_pollfds;
    pollfds[count_pollfds].fd = pa->fd;
    pollfds[count_pollfds].events = pa->events;
    pollfds[count_pollfds++].revents = 0;
    break;

    case LWS_CALLBACK_DEL_POLL_FD:
    if (!--count_pollfds)
    break;
    m = fd_lookup[pa->fd];
    /* have the last guy take up the vacant slot */
    pollfds[m] = pollfds[count_pollfds];
    fd_lookup[pollfds[count_pollfds].fd] = m;
    break;

    case LWS_CALLBACK_CHANGE_MODE_POLL_FD:
    pollfds[fd_lookup[pa->fd]].events = pa->events;
    break;

#endif

  case LWS_CALLBACK_GET_THREAD_ID:
    /*
     * if you will call "libwebsocket_callback_on_writable"
     * from a different thread, return the caller thread ID
     * here so lws can use this information to work out if it
     * should signal the poll() loop to exit and restart early
     */

    /* return pthread_getthreadid_np(); */

    break;

  default:
    break;
  }

  return 0;
}

//========================================================CAMERA============================================================================
/*
 * This callback receives the pictures from an RSB-Scope and uses them to deliver of the
 * "videostream" by a Websocket connection. The Camera device is startet as soon as the 
 * Server starts due to the threading of the Camera. The Camera device will be shut down if
 * Server is closed. While a connection to a client is activ every second pictures will be
 * send to the client. 
 */
struct per_session_data__camera {

};

static int callback_camera(struct libwebsocket_context *context,
    struct libwebsocket *wsi, enum libwebsocket_callback_reasons reason,
    void *user, void *in, size_t len) {
  // Velocity for steering
  int speed;

//   cv::Mat frame;
//   std::vector<unsigned char> outbuff;
//   std::vector<int> compression_params;

  lwsl_info("V4l callback \n");

  switch (reason) {

  case LWS_CALLBACK_ESTABLISHED:
    //if a new CLient connects count up
    clientcount++;
    clientcount_beforereset++;
    // normaly the cameradevice would be started at this point if a
    // client connects but since the camera runs in a thread, this is outdated
    break;

    // callback from libwebsocket_callback_on_writable_all_protocol
  case LWS_CALLBACK_SERVER_WRITEABLE:

    // Send the steering commands to keep the steering alive while
    // controlling the robot via on-screen navigation
    informer_vec->publish(vecSteering);

    //getting lenght of picture
    len = receivedFrame.size();

    unsigned char *buffpic;
    buffpic = bufferCam + LWS_SEND_BUFFER_PRE_PADDING;
    // Copy the image to the sendbuffer
    memcpy(buffpic, &receivedFrame[0], len);
    // sending the picture via websockets
    if (libwebsocket_write(wsi, buffpic, len, LWS_WRITE_BINARY) < 0) {
      lwsl_notice("Video write to client failed \n");
    }
    break;
  case LWS_CALLBACK_RECEIVE:
//======================================================================COMPUTE STEERING====================
    // Receiving steering comands from client
    // SpeedSignal in mm/s
    speed = atoi((const char*) in);
    if (speed <= 100) {
      speed_for_robot = speed;
      //INFO_MSG("Geschwindigkeit erhalten" << speed_for_robot)
    }
    // getting the forward speed out of the camera_canvas move commands. 
    //200 are added from the clients side to reach a range of numbers that is not used
    // afterwards 200 are substracted    
    if (speed <= 300 && speed >= 200) {
      speed_for_robot_X = speed - 200;
      //INFO_MSG("Geschwindigkeit erhalten für X: " << speed_for_robot_X)

    }
    // getting the turn speed out of the camera_canvas move commands. 
    // 500 are added from the clients side to reach a range of numbers that is not used
    // afterwards 500 are substracted    
    if (speed <= 600 && speed >= 400) {
      speed_for_robot_Y = speed - 500;
      //INFO_MSG("Geschwindigkeit erhalten für Y: " << speed_for_robot_Y)

    }
    // Convert to µm/s
    speed = speed * 1000;
    //for Steering with camera_canvas
    if (speed < 1000 && speed >= 200) {
      vecSteering->at(0) = speed_for_robot_X;
      vecSteering->at(1) = speed_for_robot_Y;
    }
    //forward left
    else if (strcmp((const char *) in, "1001") == 0) {
      INFO_MSG("forward left")
      vecSteering->at(0) = speed_for_robot;
      vecSteering->at(1) = speed_for_robot;
    }
    //forward 
    else if (strcmp((const char *) in, "1002") == 0) {
      INFO_MSG("forward")
      vecSteering->at(0) = speed_for_robot;
      vecSteering->at(1) = 0;
    }
    //forward right
    else if (strcmp((const char *) in, "1003") == 0) {  
      INFO_MSG("forward right")
      vecSteering->at(0) = speed_for_robot;
      vecSteering->at(1) = -speed_for_robot;
    }
    //left turn
    else if (strcmp((const char *) in, "1004") == 0) {  
      INFO_MSG("left turn")
      vecSteering->at(0) = 0;
      vecSteering->at(1) = speed_for_robot;
    }
    //right turn
    else if (strcmp((const char *) in, "1005") == 0) {  
      INFO_MSG("right turn")
      vecSteering->at(0) = 0;
      vecSteering->at(1) = -speed_for_robot;
    }
    //backward left
    else if (strcmp((const char *) in, "1006") == 0) {    
      INFO_MSG("backward left")
      vecSteering->at(0) = -speed_for_robot;
      vecSteering->at(1) = -speed_for_robot;
    }
    //backward
    else if (strcmp((const char *) in, "1007") == 0) {  
      INFO_MSG("backward")
      vecSteering->at(0) = -speed_for_robot;
      vecSteering->at(1) = 0;
    }
    //backward right
    else if (strcmp((const char *) in, "1008") == 0) {
      INFO_MSG("backward right")
      vecSteering->at(0) = -speed_for_robot;
      vecSteering->at(1) = speed_for_robot;
    }
    //Stop Signal
    else if (strcmp((const char *) in, "1099") == 0) {
      INFO_MSG("stop")
      vecSteering->at(0) = 0;
      vecSteering->at(1) = 0;
    }
    // Backup
    else {
      INFO_MSG("stop")
      vecSteering->at(0) = 0;
      vecSteering->at(1) = 0;
    }
//======================================================================END STEERING============================================
//======================================================================COMPUTE LIGHTSIGNALS====================
    //whitelight Signal
    if (strcmp((const char *) in, "1101") == 0) {
      white = 1;
      INFO_MSG("light on")

#ifdef __arm__
//       ::system("./leds_trigger_none.sh");
//       ::system("./leds_brightness.sh 512");
      myCAN.setLightBrightness(200);
      for (unsigned int idx = 0; idx < 8; ++idx)
        myCAN.setLightColor(idx, amiro::Color(amiro::Color::GlobalColor::WHITE));
#endif
    }

    //redlight Signal
    if (strcmp((const char *) in, "1102") == 0) {
      red = 1;
      INFO_MSG("red light on")
#ifdef __arm__
//       ::system("./leds_trigger_none.sh");
//       ::system("./leds_red_brightness.sh 512");
      myCAN.setLightBrightness(200);
      for (unsigned int idx = 0; idx < 8; ++idx)
        myCAN.setLightColor(idx, amiro::Color(amiro::Color::GlobalColor::RED));
#endif
    }

    //greenlight Signal
    if (strcmp((const char *) in, "1103") == 0) {
      green = 1;
      INFO_MSG("green light on")
#ifdef __arm__
//       ::system("./leds_trigger_none.sh");
//       ::system("./leds_green_brightness.sh 512");
      myCAN.setLightBrightness(200);
      for (unsigned int idx = 0; idx < 8; ++idx)
        myCAN.setLightColor(idx, amiro::Color(amiro::Color::GlobalColor::GREEN));
#endif
    }

    //bluelight Signal
    if (strcmp((const char *) in, "1104") == 0) {
      blue = 1;
      INFO_MSG("blue light on")
#ifdef __arm__
//       ::system("./leds_trigger_none.sh");
//       ::system("./leds_blue_brightness.sh 512");
      myCAN.setLightBrightness(200);
      for (unsigned int idx = 0; idx < 8; ++idx)
        myCAN.setLightColor(idx, amiro::Color(amiro::Color::GlobalColor::BLUE));
#endif
    }

    //lights off Signal
    if (strcmp((const char *) in, "1105") == 0) {
      white = 0;
      red = 0;
      green = 0;      
      blue = 0;
      INFO_MSG("all lights off")
#ifdef __arm__
//       ::system("./leds_trigger_none.sh");
//       ::system("./leds_brightness.sh 0");
      myCAN.setLightBrightness(0);
#endif
    }



    // Point to reenter the queue to take control over robot
    if (strcmp((const char *) in, "1108") == 0){
      clientcount_beforereset++;
      userIndex = 1000;
    } 
//======================================================================END LIGHTSIGNALS============================================
    break;

  case LWS_CALLBACK_CLOSED:
    // each time a client closes the connection the clientcount is decremented
    clientcount--;
    INFO_MSG("Lowest counter: " << clientcount)
    // if no Clients are connected reset clientcount_beforeresert
    if (clientcount == 0) {
      clientcount_beforereset = 0;
    }
    /*
    * each time a client leaves the userIndex is set to 100 and the server 
    * gives the client with the lowest userNumber the permission to send steering
    * commands.
    */
    userIndex = 1000;
    break;
  default:
    break;
  }

  return 0;
}
//========================================================INFRAREDWEBSOCKET=================================================================
struct per_session_data__infrared {
  const char *message;
};

static int callback_infrared(struct libwebsocket_context *context,
    struct libwebsocket *wsi, enum libwebsocket_callback_reasons reason,
    void *user, void *in, size_t len) {
  unsigned char buf[LWS_SEND_BUFFER_PRE_PADDING + 512 +
  LWS_SEND_BUFFER_POST_PADDING];
  unsigned char *p = &buf[LWS_SEND_BUFFER_PRE_PADDING];
  int user_number;
  struct per_session_data__infrared *pss =
      (struct per_session_data__infrared *) user;

  switch (reason) {

  case LWS_CALLBACK_SERVER_WRITEABLE:

    // Copy the IR data to a text string, which will be send to the client
    textStringToSend.clear();
    
    // For future developement. If u can deterime via code which robot is in use, 
    // just change the first char of the string to the apropriate number
    // if(Roboter==Bebot) => send 1 else 0 for Amiro    
    textStringToSend = "0";  //1 == Bebot
    unsigned int idxIrData;
    for (idxIrData = 0; idxIrData < vecIrData->size(); idxIrData++) {
      textStringToSend += ":" + to_string(vecIrData->at(idxIrData));
    }
    /* Information about who is in control of the steering and how many users have logged on are send 
     * to each Client here. 
     * clientcount holds the current number of connected clients
     * clientcount_beforereset counts up if another clients connects and is only reset when the last client left
     * userIndex tells the clients who is in control of the steering commands
     */
    textStringToSend += ":;" + to_string(userIndex) + ";"
        + to_string(clientcount_beforereset) + ";"
        + to_string(clientcount);
    
    //preparing the light status
    lightMessage.clear();
    lightMessage = ";" + to_string(white) + ";" + to_string(red) + ";"
        + to_string(green) + ";" + to_string(blue);

    textStringToSend += lightMessage;

    //sending Infrared // light status // Client status    
    pss->message = textStringToSend.c_str();
    {
      int n = sprintf((char *) p, "%s", pss->message);
      int m = libwebsocket_write(wsi, p, n, LWS_WRITE_TEXT);
      if (m < n) {
        lwsl_err("ERROR %d writing to socket\n", n);
        return -1;
      }
    }

    break;

  case LWS_CALLBACK_RECEIVE:
    user_number = atoi((const char*) in);
    //lowest number received gets control
    if (user_number < userIndex) {
      userIndex = user_number;
    }
    break;

  default:
    break;
  }

  return 0;
}

//========================================================PROTOCOLS============


enum protocols {
  /* always first */
  PROTOCOL_HTTP = 0, PROTOCOL_CAMERA, PROTOCOL_INFRARED,
  /* always last */
  PROTOCOL_COUNT
};

/* list of supported protocols and callbacks */

static struct libwebsocket_protocols protocols[] = {
/* first protocol must always be HTTP handler */

{ "http-only",             /* name */
  callback_http,           /* callback */
  sizeof(struct per_session_data__http),     /* per_session_data_size */
  0,             /* max frame size / rx buffer */
},

{ "camera", callback_camera, sizeof(struct per_session_data__camera), 57600, },
    { "infrared", callback_infrared,
        sizeof(struct per_session_data__infrared), 10, },

    { NULL, NULL, 0, 0 } /* terminator */
};

void sighandler(int sig) {
  force_exit = 1;
  libwebsocket_cancel_service(context);
}

//options
static struct option options[] = { 
  { "help", no_argument, NULL, 'h' }, 
  { "debug", required_argument, NULL, 'd' }, 
  { "port", required_argument, NULL, 'p' }, 
  { "thread_camera", no_argument, NULL, 't' }, 
  { "interface", required_argument, NULL, 'i' }, 
  { "closetest", no_argument, NULL, 'c' }, 
  { "libev", no_argument, NULL, 'e' },
#ifndef LWS_NO_DAEMONIZE
    { "daemonize", no_argument, NULL, 'D' },
#endif
    { "resource_path", required_argument, NULL, 'r' }, { NULL, 0, 0, 0 } };
 
int main(int argc, char **argv) {
  int n = 0;
  int opts = 0;
  bool startCameraThread = false;
  char interface_name[128] = "";
  const char *iface = NULL;
#ifndef WIN32
  int syslog_options = LOG_PID | LOG_PERROR;
#endif
  struct lws_context_creation_info info;

  int debug_level = 7;
#ifndef LWS_NO_DAEMONIZE
  int daemonize = 0;
#endif

  memset(&info, 0, sizeof info);
  info.port = 7681;

  while (n >= 0) {
    n = getopt_long(argc, argv, "eci:hsapt:d:Dr:", options, NULL);
    if (n < 0)
      continue;
    switch (n) {
    case 'e':
      opts |= LWS_SERVER_OPTION_LIBEV;
      break;
#ifndef LWS_NO_DAEMONIZE
    case 'D':
      daemonize = 1;
#ifndef WIN32
      syslog_options &= ~LOG_PERROR;
#endif
      break;
#endif
    case 'd':
      debug_level = atoi(optarg);
      break;
    case 'p':
      info.port = atoi(optarg);
      break;
    case 'i':
      strncpy(interface_name, optarg, sizeof interface_name);
      interface_name[(sizeof interface_name) - 1] = '\0';
      iface = interface_name;
      break;
    case 'c':
      close_testing = 1;
      fprintf(stderr, " Close testing mode -- closes on "
          "client after 50 dumb increments"
          "and suppresses lws_mirror spam\n");
      break;
    case 'r':
      resource_path = optarg;
      printf("Setting resource path to \"%s\"\n", resource_path);
      break;
    case 't':
      startCameraThread = true;
      printf("Using internal thread for processing the camera");
      break;
    case 'h':
      fprintf(stderr, "Usage: murox-server "
          "[--port=<p>] "
          "[-d <log bitfield>] "
          "[-t] Use internal thread for processing the camera"
          "[--resource_path <path>]\n");
      exit(1);
    }
  }

#if !defined(LWS_NO_DAEMONIZE) && !defined(WIN32)
  /* 
   * normally lock path would be /var/lock/lwsts or similar, to
   * simplify getting started without having to take care about
   * permissions or running as root, set to /tmp/.lwsts-lock
   */
  if (daemonize && lws_daemonize("/tmp/.lwsts-lock")) {
    fprintf(stderr, "Failed to daemonize\n");
    return 1;
  }
#endif

  ::signal(SIGINT, sighandler);

#ifndef WIN32
  /* we will only try to log things according to our debug_level */
  setlogmask(LOG_UPTO(LOG_DEBUG));
  openlog("lwsts", syslog_options, LOG_DAEMON);
#endif

  /* tell the library what debug level to emit and to send it to syslog */
  lws_set_log_level(debug_level, lwsl_emit_syslog);

  lwsl_notice("libwebsockets test server - "
      "(C) Copyright 2010-2013 Andy Green <andy@warmcat.com> - "
      "licensed under LGPL2.1\n");
#ifdef EXTERNAL_POLL
  max_poll_elements = getdtablesize();
  pollfds = malloc(max_poll_elements * sizeof (struct pollfd));
  fd_lookup = malloc(max_poll_elements * sizeof (int));
  if (pollfds == NULL || fd_lookup == NULL) {
    lwsl_err("Out of memory pollfds=%d\n", max_poll_elements);
    return -1;
  }
#endif

  info.iface = iface;
  info.protocols = protocols;
  info.gid = -1;
  info.uid = -1;
  info.options = opts;

  context = libwebsocket_create_context(&info);
  if (context == NULL) {
    lwsl_err("libwebsocket init failed\n");
    return -1;
  }

  ////////////////////////rsb/////////////////////
  INFO_MSG("Create rsb environment")
  // Register new converter for std::vector<int>
  converterVecInt.reset(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Register new converter for cv::Mat
  converterMat.reset(new MatConverter());
  converterRepository<std::string>()->registerConverter(converterMat);

  // Prepare RSB informer
  informer_vec = factory.createInformer<std::vector<int> >(
      g_sOutScopeSteering);

  // Holds the new steering comand (two elements with 0 initialized)
  vecSteering.reset(new std::vector<int>(2, 0));

  // Add the scope to the listener
  listener = factory.createListener(g_sInScopeIr);

  // Assigne the function to the listener
  listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&receiveIrData)));

  // Add the scope to the camera listener
        // BUG: Inprocess transport is manualy set to enable, because the rsb.conf has a bug,
        // so that you cannot enable socket and inprocess for a reader
        if (startCameraThread) {
          // Create an explicit inprocess config
          rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
          // Disable socket
          {
                  // Get the options for socket transport, because we want to change them
                  rsc::runtime::Properties tmpProp = tmpPartConf.mutableTransport("socket").getOptions();

                  // disable socket transport
                  std::string enabled = "0";
                  tmpProp["enabled"] = boost::any(enabled);

                  // Write the socket tranport properties back to the participant config
                  tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
          }
          // Enable inprocess
          {
                  // Get the options for socket transport, because we want to change them
                  rsc::runtime::Properties tmpProp = tmpPartConf.mutableTransport("inprocess").getOptions();

                  // disable socket transport
                  std::string enabled = "1";
                  tmpProp["enabled"] = boost::any(enabled);

                  // Write the socket tranport properties back to the participant config
                  tmpPartConf.mutableTransport("inprocess").setOptions(tmpProp);
          }
          listenerCamera = factory.createListener(g_sInScopeImage,tmpPartConf);
        } else {
          listenerCamera = factory.createListener(g_sInScopeImage);
        }

  // Assigne the function to the camera listener
  listenerCamera->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&listenCamera)));

  // Holds the IR data (12 elements with 0 initialized)
  vecIrData.reset(new std::vector<int>(12, 0));

  // Holds the received Frame
//      receivedFrame.reset(new cv::Mat);

  // Init the thread which handles the camera
  pthread_t threadCamera = 0;
  // Start the thread which reads the camera if the argument says so
  if (startCameraThread) {
    pthread_create(&threadCamera, NULL, informerCamera, argv[1]);
  }
  ////////////////////////////////////////////////

  n = 0;
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  while (n >= 0 && !force_exit) {

    /*
     * This provokes the LWS_CALLBACK_SERVER_WRITEABLE for every
     * live websocket connection using the DUMB_INCREMENT protocol,
     * as soon as it can take more packets (usually immediately)
     */

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time_span = std::chrono::duration_cast<
        std::chrono::duration<float>>(t2 - t1);

    // Check if the time difference is bigger than the value in seconds, then execute the callbacks
    if (time_span.count() > 0.05) {  // 0.05 = 20x in a second
      if (frameReceived == true) {
        libwebsocket_callback_on_writable_all_protocol(
            &protocols[PROTOCOL_CAMERA]);
        frameReceived = false;
      }
      libwebsocket_callback_on_writable_all_protocol(
          &protocols[PROTOCOL_INFRARED]);
      t1 = std::chrono::high_resolution_clock::now();
    }

#ifdef EXTERNAL_POLL

    /*
     * this represents an existing server's single poll action
     * which also includes libwebsocket sockets
     */

    n = poll(pollfds, count_pollfds, 50);
    if (n < 0)
    continue;

    if (n)
    for (n = 0; n < count_pollfds; n++)
    if (pollfds[n].revents)
    /*
     * returns immediately if the fd does not
     * match anything under libwebsockets
     * control
     */
    if (libwebsocket_service_fd(context,
            &pollfds[n]) < 0)
    goto done;
#else
    /*
     * If libwebsockets sockets are all we care about,
     * you can use this api which takes care of the poll()
     * and looping through finding who needed service.
     *
     * If no socket needs service, it'll return anyway after
     * the number of ms in the second argument.
     */

    n = libwebsocket_service(context, 50);
#endif
  }

#ifdef EXTERNAL_POLL
  done:
#endif

  libwebsocket_context_destroy(context);
  lwsl_notice("libwebsockets-murox-server exited cleanly\n");

  // Kill the thread
  if (threadCamera != 0) {
    pthread_cancel(threadCamera);
    // Close the camera device
    cam.release();
  }

  // Clean up the RSB listener and informer
  listener.reset();
  listenerCamera.reset();
  informer_vec.reset();
  vecIrData.reset();
  vecSteering.reset();
  converterVecInt.reset();
  converterMat.reset();
//        receivedFrame.reset();

#ifndef WIN32
  closelog();
#endif

  return 0;
}
