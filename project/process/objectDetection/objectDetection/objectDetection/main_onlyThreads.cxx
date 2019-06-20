#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// For checking character pressed in the console
#include <kbhit.hpp>


// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_SAVE = "SAVE";
std::string COMMAND_NEW = "NEW";
std::string COMMAND_DELETE = "DEL";
std::string COMMAND_COMPARE = "COMP";
std::string COMMAND_LOAD = "LOAD";

#define maxObjects 10
#define maxObjectSites 8


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
using namespace rsb::converter;
using namespace cv;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

static std::string g_sImageScope = "/objectDetection/image";
static std::string g_sOutScope = "/objectDetection/detected";
static std::string g_sInScope = "/objectDetection/command";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;

std::string directory = "objectPics/";
std::string siftDirectory = "siftResults/";
std::string savingDir = "object_";
std::string fileend = ".jpg";

// init case values
bool saving = false;
bool sendingPic = false;
bool reversedSearchOrder = false;
bool debugging = false;
bool checkAllObjects = false;

// init values for loading and comparing
int objectCount = 0;
int siteId = 0;
int comparisonCount = 0;
Mat objectPics[maxObjects][maxObjectSites];
std::vector<KeyPoint> objectKeyPoints[maxObjects][maxObjectSites];
Mat objectDescriptors[maxObjects][maxObjectSites];
bool detected[maxObjectSites];
bool threadFinished[maxObjectSites];
boost::mutex threadLocks[maxObjectSites];

// init SURF class
int minHessian = 1000;
SurfFeatureDetector detector(minHessian);
SurfDescriptorExtractor extractor;

// init margins
float minDist = 100;
float minDia = 100;
float maxDist = 500;



// saving objects procedure
void savingObject(Mat img_object, int objectNum, int objectSite, bool saveImg) {
  // Save histogramms
  img_object.copyTo(objectPics[objectNum][objectSite]);
  detector.detect(img_object, objectKeyPoints[objectNum][objectSite]);
  extractor.compute(img_object, objectKeyPoints[objectNum][objectSite], objectDescriptors[objectNum][objectSite]);

  if (saveImg) {
    string filename = directory + savingDir + to_string(objectNum) + "_" + to_string(objectSite) + fileend;
    bool ok = imwrite(filename, img_object);
    if (!ok) {
      std::cout << "Couldn't save object " << (objectNum+1) << " with site " << (objectSite+1) << " into file '" << directory << savingDir << objectNum << "_" << objectSite << fileend << "'!\n";
    } 
  }
}


// load objects
void loadObjects() {
  for (objectCount=0; objectCount<maxObjects; objectCount++) {
    printf("Try to load %i. object\n", objectCount+1);
    int siteId;
    for (siteId=0; siteId<maxObjectSites; siteId++) {
      string filename = directory + savingDir + to_string(objectCount) + "_" + to_string(siteId) + fileend;
      Mat frame = imread(filename, 1);
      if (frame.rows > 0 && frame.cols > 0) {
        savingObject(frame, objectCount, siteId, false);
      } else {
        //printf("Couldn't load at object %i site %i.\n", objectCount+1, siteId+1);
        break;
      }
    }
    if (siteId == 0) {
      break;
    }
  }
  if (objectCount > 0) {
    printf("%i objects loaded.\n", objectCount);
  } else {
    std::cout << "No objects could be loaded in relative directory '" << directory << "' with filename type '" << savingDir << "x_x" << fileend << "'!\n";
  }
}


void threadedDetection(int objId, int siteId, Mat img_scene, std::vector<KeyPoint> keypoints_scene, Mat descriptors_scene) {
  clock_t tStart, tEnd;
  tStart = clock();
  // load object keypoints and descriptor
  Mat img_object;
  objectPics[objId][siteId].copyTo(img_object);
  std::vector<KeyPoint> keypoints_object = objectKeyPoints[objId][siteId];
  Mat descriptors_object = objectDescriptors[objId][siteId];

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match(descriptors_object, descriptors_scene, matches);

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors_object.rows; i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for (int i = 0; i < descriptors_object.rows; i++) {
    if (matches[i].distance < 3*min_dist) {
      good_matches.push_back(matches[i]);
    }
  }

      
  Mat img_matches;
  if (debugging) {
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  }

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;
  for( int i = 0; i < good_matches.size(); i++ ) {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  bool detectedFeatures = obj.size() >= 4 && scene.size() >= 4;
  bool detectedDist = true;
  bool detectedPos = true;
  bool detectedDia = true;
  bool detectedLarge = true;
  std::vector<float> finalEdges(4);
  std::vector<Point2f> obj_corners(4);
  std::vector<Point2f> scene_corners(4);
  if (detectedFeatures) {

    // TODO make threaded! Break thread after ca. a second.
    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_scene.cols, 0 );
    obj_corners[2] = cvPoint( img_scene.cols, img_scene.rows ); obj_corners[3] = cvPoint( 0, img_scene.rows );

    perspectiveTransform( obj_corners, scene_corners, H);

    int countLessMinDist = 0;
    for (int i=0; i<4; i++) {
      int j=i+1;
      int k=i+2;
      if (j == 4) j=0;
      if (k >= 4) k-=4;
      for (int i_ref=0; i_ref<4; i_ref++) {
        if (i != i_ref) {
          Point2f diff = scene_corners[i] - scene_corners[j];
          float dist = sqrt(diff.x*diff.x + diff.y*diff.y);
          //finalEdges[i] = dist;
          if (dist > maxDist) {
            detectedLarge = false;
          }
          if (i_ref == j && dist < minDist) {
            countLessMinDist++;
          }
          if (i_ref == k && dist < minDia) {
            detectedDia = false;
          }
        }
      }
      switch (i) {
        case 0: if (scene_corners[j].x - scene_corners[i].x <= 0) detectedPos=false; break;
        case 1: if (scene_corners[j].y - scene_corners[i].y <= 0) detectedPos=false; break;
        case 2: if (scene_corners[i].x - scene_corners[j].x <= 0) detectedPos=false; break;
        default: if (scene_corners[i].y - scene_corners[j].y <= 0) detectedPos=false; break;
      }
    }
    if (countLessMinDist > 1) {
      detectedDist = false;
    }

    if (debugging) {
      Scalar color;
      if (detectedDist && detectedPos && detectedDia && detectedLarge) { // && (tObjEnd-tObjStart >= 1000000)) {
        color = Scalar(255, 0, 0);
      } else if (detectedDist && detectedPos && detectedDia && detectedLarge) {
        color = Scalar(0, 255, 0);
      } else {
        color = Scalar(0, 0, 255);
      }

      //-- Draw lines between the corners (the mapped object in the scene - image_2 )
      line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), color, 4 );
      line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), color, 4 );
      line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), color, 4 );
      line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), color, 4 );
    }
  }

  threadLocks[siteId].lock();
  detected[siteId] = detectedFeatures && detectedDist && detectedLarge && detectedDia && detectedPos;
  threadLocks[siteId].unlock();

  tEnd = clock();

  printf("Object %i site %i ", objId+1, siteId+1);
  if (detected[siteId]) {
    printf("detected");
  } else {
    printf("could not be detected by");
    if (!detectedFeatures) {
      printf(" missing features");
    } else if (!detectedDist && !detectedPos && !detectedLarge && !detectedDia) {
      printf(" square");
    } else if (!detectedDist) {
      printf(" too small square");
    } else if (!detectedPos) {
      printf(" too ugly square");
    } else if (!detectedDia) {
      printf(" too short dias");
    } else {
      printf(" too large square");
    }
  }
  if (debugging) {
    printf(" - %i us", int(tEnd-tStart));
  }
  printf("\n");

  if (debugging) {
    string filename = siftDirectory + savingDir + to_string(comparisonCount) + "_" + to_string(objId) + "_" + to_string(siteId) + fileend;
    bool ok = imwrite(filename, img_matches);
    if (!ok) {
      printf("[Couldn't save comparison!]\n");
    }
  }

  threadLocks[siteId].lock();
  threadFinished[siteId] = true;
  threadLocks[siteId].unlock();
}



int searchObjectsInScene(Mat img_scene) {
  clock_t tStart, tFV, tEnd, tObjStart, tObjEnd, tBeforeMatch, tAfterMatch, tBeforeHomography, tAfterHomography;
  tStart = clock();

  // create feature vector of scene
  printf("Create feature vector of scene.\n");
  std::vector<KeyPoint> keypoints_scene;
  Mat descriptors_scene;
  detector.detect(img_scene, keypoints_scene);
  extractor.compute(img_scene, keypoints_scene, descriptors_scene);

  tFV = clock();
  if (debugging) {
    printf("Time for creating feature vector: %i us\n", int(tFV-tStart));
  }

  bool finalDetection = false;
  for (int siteIdx=0; siteIdx<maxObjectSites; siteIdx++) {
    threadFinished[siteIdx] = true;
  }

  // compare scene features with all object features
  int objId, detectObjId;
  for (int objIdFor=0; objIdFor<objectCount; objIdFor++) {
    if (reversedSearchOrder) {
      objId = objectCount-1-objIdFor;
    } else {
      objId = objIdFor;
    }

    // start all threads
    boost::thread threads[maxObjectSites];
    for (int siteId=0; siteId<maxObjectSites; siteId++) {
      bool firstTime = true;
      threadLocks[siteId].lock();
      while(!threadFinished[siteId]) {
        threadLocks[siteId].unlock();
        if (firstTime) {
          printf("Thread for site %i still running.\n", siteId+1);
          firstTime = false;
        }
        usleep(100*1000);
        threadLocks[siteId].lock();
      }
      threadLocks[siteId].unlock();
      if (!firstTime) {
        printf("Starting thread for site %i now.\n", siteId+1);
      }

      tObjStart = clock();

      threadFinished[siteId] = false;
      threads[siteId] = boost::thread(threadedDetection, objId, siteId, img_scene, keypoints_scene, descriptors_scene);

      int maxTime = 1100; // ms
      int spTime = 0; // ms
      bool threadDetected = false;

      threadLocks[siteId].lock();
      while(!threadFinished[siteId] && spTime <= maxTime) {
        threadLocks[siteId].unlock();
        spTime += 100;
        usleep(100*1000);
        threadLocks[siteId].lock();
      }
      if (threadFinished[siteId]) {
        threadDetected = detected[siteId];
      }
      threadLocks[siteId].unlock();

      // TODO join or cancel thread here!
//      if (threadFinished[siteId]) {
//        threads[siteId].join();
//      } else {
//        threads[siteId].~thread();
//      }

      tObjEnd = clock();
      if (debugging) {
        printf("Time for object %i site %i: %i", objId+1, siteId+1, int(tObjEnd-tObjStart));
        if (threadDetected) {
          printf(" - detected!");
        }
        printf("\n");
      }

      if (threadDetected) {
        detectObjId = objId;
      }

      if (!finalDetection) {
        finalDetection = threadDetected;
      }
    }


    if (finalDetection && !checkAllObjects) {
      break;
    }
  }

  tEnd = clock();
  if (debugging) {
    printf("Time for whole objectDetection: %i us\n", int(tEnd-tStart));
  }

  comparisonCount++;

  if (finalDetection) {
    printf("\nObject %i detected!\n\n", detectObjId+1);
    return detectObjId+1;
  } else {
    printf("\nNo object detected. Are you sure that there was a known object in view sight?\n\n");
    return 0;
  }
}



// main function
int main(int argc, char **argv) {  
  
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving commands")
            ("loadingDirectory,l", po::value < std::string > (&directory),"Directory from where the data can be loaded.")
            ("device,d", po::value < int > (&g_iDevice),"Number of device")
            ("quality,q", po::value < unsigned int > (&g_uiQuality),"Quality of JPEG compression [0 .. 100]")
            ("sending,s", "Sends the taken snapshot over RSB.")
            ("reverseSearch,r", "Reverses search order of objects.")
            ("debug", "Activates debugging which includes generated pictures and additional console information.")
            ("checkAll", "All objects will be checked.")
            ("printPic", "Prints a notice if a new picture has been taken.");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

  // first, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }

  sendingPic = vm.count("sending");
  reversedSearchOrder = vm.count("reverseSearch");
  debugging = vm.count("debug");
  checkAllObjects = vm.count("checkAll");
    
  // afterwards, let program options handle argument errors
  po::notify(vm);
    
  INFO_MSG("Output scope: " << g_sOutScope);
  INFO_MSG("Command scope: " << g_sInScope);
  if (sendingPic) {
    INFO_MSG("Picture scope: " << g_sImageScope);
  }
  INFO_MSG("Device: " << g_iDevice);
  INFO_MSG("JPEG Quality: " << g_uiQuality);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::getFactory();

  // Create the informer
  Informer<std::string>::Ptr imageInformer = getFactory().createInformer<std::string> (Scope(g_sImageScope));
  Informer<std::string>::Ptr detectedInformer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

  // Create and start the command listener
  rsb::ListenerPtr listener = factory.createListener(g_sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video<g_iDevice>
  if ( cam.open(g_iDevice) ) {

    Mat frame;

    // Process the cam forever
    for (; ;) {

      // Save the actual picture to the frame object
      cam >> frame;
      if (vm.count("printPic")) {
        printf("New image taken.\n");
      }
      
      if (sendingPic) {
        // Compress image
        vector<uchar> buf;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(g_uiQuality);
      
        imencode(".jpg", frame, buf, compression_params);

        // Send the data.
        shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
        imageInformer->publish(frameJpg);
      }

      // Get command
      if (!commandQueue->empty()) {
        std::string command = *commandQueue->pop().get();
        // check for quit command
        if (command == COMMAND_QUIT) {
          INFO_MSG("Quit application.");
          break;
        // check for save site command
        } else if (command == COMMAND_SAVE) {
          savingObject(frame, objectCount, siteId, true);
          siteId++;
          saving = true;
          printf("Saved %i. object %i. site.\n", objectCount+1, siteId);
          if (siteId >= maxObjectSites) {
            siteId = 0;
            objectCount++;
            saving = false;
          }
        // check for delete command
        } else if (command == COMMAND_DELETE) {
          objectCount = 0;
          siteId = 0;
          printf("All objects unlearned (old pictures will be overwritten with next learning)!\n");
        // check for compare commandCOMMAND_STORE
        } else if (command == COMMAND_COMPARE) {
          if (objectCount > 0 && !saving) {
            printf("Comparing:\n");
            int detObj = searchObjectsInScene(frame);

            std::string output;
            if (detObj > 0) {
              output = to_string(detObj);
            } else {
              output = "null";
            }
            shared_ptr<std::string> StringPtr(new std::string(output));
            detectedInformer->publish(StringPtr);

          } else if (siteId != 0) {
            printf("Please save the rest %i sites for object %i to continue with comparing.\n", maxObjectSites-siteId, objectCount+1);
          } else {
            printf("No objects saved for comparison.\n");
          }
        // check for storing command
        } else if (command == COMMAND_LOAD) {
          loadObjects();
        // otherwise it is an unknown command
        } else {
          INFO_MSG("Unknown command.");
        }
      }

    }
  }

  // Free the cam
  cam.release();

  return 0;

}
