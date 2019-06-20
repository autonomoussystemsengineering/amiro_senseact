#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

//RSC
#include <rsc/misc/SignalWaiter.h>
// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <converter/matConverter/matConverter.hpp>

#include <Constants.h>

// Arguments
static std::string mapserverScope("/mapServer");
static std::string requestString("mapImageReq");
static std::size_t rpcDelay = 10;

int main(int argc, char* argv[]) {

  // Handle program options
  namespace po = boost::program_options;
  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
  ("mapserverScope,m", po::value<std::string>(&mapserverScope), "scope for the mapserver, default = /mapServer")
  ("requestString,r", po::value<std::string>(&requestString), "Request String, default = mapImageReq")
  ("rpcDelay,d", po::value<std::size_t>(&rpcDelay), "Seconds to wait for a reply");

  // allow to give the value as a positional argument
  po::positional_options_description p_description;
  p_description.add("value", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(options).positional(p_description).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  rsb::Factory& factory = rsb::getFactory();
  boost::shared_ptr<muroxConverter::MatConverter> matConverter(new muroxConverter::MatConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(matConverter);
  rsb::patterns::RemoteServerPtr remoteServer = factory.createRemoteServer(mapserverScope);

  // Request the map as an image
  try {
    boost::shared_ptr<cv::Mat> result;
    result = remoteServer->call<cv::Mat>(requestString, rpcDelay /*second*/);
    cv::flip(*result, *result, 0);  // Horizontal flip to make the y-axis points upwards
    cv::imshow("DisplayRoi", *result);
    cv::waitKey(0);
  } catch (...) {
    WARNING_MSG("Map server error")
  }

  return 0;
}
