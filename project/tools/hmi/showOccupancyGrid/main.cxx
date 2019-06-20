//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Request an occupancy grid map from a mapping server as cv::mat image
//============================================================================




#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <math.h>
#define PI 3.14159265


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rst/navigation/OccupancyGrid2DInt.pb.h>
using namespace rst::navigation;
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;
      
using namespace std;



// Prototypes
void showMap(rsb::EventPtr event);


std::string g_sInScope_OGM = "/maps/ogm";



int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&g_sInScope_OGM), "Scope for receiving the grid maps");

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
  
  // afterwards, let program options handle argument errors
  po::notify(vm);
  
  INFO_MSG( "Inscope: " << g_sInScope_OGM)

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register new converter for occupancy grid maps
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<OccupancyGrid2DInt> >
      converter(new rsb::converter::ProtocolBufferConverter<OccupancyGrid2DInt>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader for the IR data
  rsb::ReaderPtr reader = factory.createReader(g_sInScope_OGM);

  // Receive IR values and publish the map
  while (true) {
    showMap(reader->read());
  }

  return EXIT_SUCCESS;
}

void showMap(rsb::EventPtr event) {

  // Get the message with the IR data
  boost::shared_ptr<OccupancyGrid2DInt > map = boost::static_pointer_cast<OccupancyGrid2DInt >(event->getData());

  // Copy map to an image
  cv::Mat mapImage(map->height(), map->width(), CV_8UC1);
  for(int i = 0; i < map->height(); i++)
    for(int j = 0; j < map->width(); j++)
      mapImage.at<uchar>(i,j)=map->map()[j+i*map->width()];

  // Show the map
  #ifndef __arm__
    cv::flip(mapImage,mapImage,0); // Flip the image, to display the correct coordinates bottom-left
    cv::imshow("Map",mapImage);
    cv::waitKey(1);
  #endif

}
