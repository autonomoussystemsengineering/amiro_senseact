// RSB
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
// #include <rst0.11/stable/rst/vision/LaserScan.pb.h>
// #include <types/LocatedLaserScan.pb.h>
// #include <rst/geometry/Pose.pb.h>
#include <rst/vision/LaserScan.pb.h>

// For program options
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <Eigen/Geometry>

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace rsb::converter;
static std::string outScope = "/lidar";
static std::size_t publishNthScan = 1;
#include "HokuyoReader.hh"
static std::string hokuyoDevice = "/dev/ttyACM0";
//SCAN_NAME is used to determine the scanType and scanSkip values
//See below for details
static std::string scanName = "range"; // ["top_urg_range+intensity" | "range+intensity1+AGC1"]
static unsigned int data[HOKUYO_MAX_NUM_POINTS] = {};
static int n_data;          //number of returned points
const int scanStartMin=44;      //Min start of the scan
static int scanStart=44;      // Start of the scan
static int scanEnd=725;      // End of the scan
const  int scanEndMax=725;      //Max end of the scan
static int scanSkip=1;       //this is so-called "cluster count", however special values
static int encoding=HOKUYO_3DIGITS; //HOKUYO_2DIGITS
static int baud=115200;            //communication baud rate (does not matter for USB connection)

static int scanType;               //scan type specifies whether a special scan is required,
                                   //such as HOKUYO_SCAN_SPECIAL_ME - for URG-30LX intensity mode
                                   //otherwise use HOKUYO_SCAN_REGULAR. This will be automatically
                                   //acquired by the getScanTypeAndSkipFromName() function below

static bool doTest = false;

int setup(HokuyoReader &hokuyoReader);
int test(HokuyoReader &hokuyoReader);

int main(int argc, char **argv) {

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&outScope),"Scope for sending scans")
            ("device,d", po::value < std::string > (&hokuyoDevice),"Device name (e.g. /dev/ttyACM0)")
            ("publishNthScan", po::value < std::size_t > (&publishNthScan),"Publish only every n'th scan via RSB")
            ("baudrate,b", po::value < int > (&baud),"Communication baud rate (does not matter for USB connection)")
            ("scanName,n", po::value < std::string > (&scanName),"[range | top_urg_range+intensity | range+intensity1+AGC1]")
            ("scanStart,s", po::value < int > (&scanStart),"Index where to start of the scan (e.g. Min 44)")
            ("scanEnd,e", po::value < int > (&scanEnd),"Index where to end of the scan (e.g. Max 725)")
            ("test", po::value < bool > (&doTest),"Performing test without any RSB interaction")
            ("encoding", po::value < int > (&encoding),"[2 | 3] 2 or 3 char encoding. 04LX supports both, but 30LX only 3-char 2-char encoding reduces range to 4meters, but improves data transfer rates over standard serial port");
            // ("scanSkip,k", po::value < int > (&scanSkip),"this is so-called 'cluster count', however special values of this variable also let request Intensity and AGC values from URG-04LX. (see the intensity mode manual and Hokuyo.hh). If the scanner is 04LX and the scan name is not 'range', then this skip value will be overwritten with the appropriate value in order to request the special scan. See call to getScanTypeAndSkipFromName() below")

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

    // Configure the lidar
    HokuyoReader hokuyoReader;
    if(setup(hokuyoReader) != 0) {
      return -1;
    }

    if(doTest) {
      int ret = test(hokuyoReader);
      hokuyoReader.Disconnect();
      return ret;
    }

    INFO_MSG("")

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  // rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

  rsb::Factory& factory = rsb::getFactory();
  // Create the informer
  rsb::Informer<rst::vision::LaserScan>::Ptr informer = factory.createInformer<rst::vision::LaserScan> (outScope);
  boost::shared_ptr<rst::vision::LaserScan> laserScan(new rst::vision::LaserScan());
  // rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScan(new rst::vision::LocatedLaserScan);


  // const double radPerStep = (360.0 /*°*/ / 1024.0 /*Steps*/) * (M_PI /*rad*/ / 180.0f /*°*/);
  // const double radPerSkipStep = (360.0 /*°*/ / 1024.0 /*Steps*/) * (M_PI /*rad*/ / 180.0f /*°*/) * scanSkip;
  // const double startAngle =  120.0 * M_PI / 180.0;
//   const double endAngle   = -120 * M_PI / 180;
  // if (scanSkip == 1) {
  //   laserScan->set_scan_angle(radPerStep * (scanEnd - scanStart + 1));
  //   laserScan->set_scan_angle_start(startAngle - radPerStep * (scanStart - scanStartMin));
  //   laserScan->set_scan_angle_end(startAngle - radPerStep * (scanEnd - scanStartMin));
  // } else {
  //   laserScan->set_scan_angle(radPerStep * (scanEnd - scanStart + 1) - radPerSkipStep);
  //   laserScan->set_scan_angle_start(startAngle - radPerStep * (scanStart - scanStartMin) - radPerSkipStep / 2.0f);
  //   laserScan->set_scan_angle_end(startAngle - radPerStep * (scanEnd - scanStartMin) + radPerSkipStep / 2.0f);
  // }
  // laserScan->set_scan_values_min(0.02); // From Hokuyo URG04 manual
  // laserScan->set_scan_values_max(4.0); // From Hokuyo URG04 manual
  // laserScan->set_scan_angle_increment(radPerSkipStep);
  laserScan->set_scan_angle(3.98); // From Hokuyo URG04 manual, min=0.02 and max=4.0

  int n_data_expected = ceil((static_cast<float>(scanEnd - scanStart + 1) / static_cast<float>(scanSkip)));

  // Reserve data
  for(int idx = 1; idx <= n_data_expected; ++idx)
    laserScan->mutable_scan_values()->Add(0.0f);

  // Process the lidar forever
  size_t publishCounter = 0;
  for (; ;) {
    // Fetch the data
    hokuyoReader.GetScan(data, n_data);
    // process it
    if (n_data == n_data_expected ) {
      if (publishCounter % publishNthScan == 0) {
        for(int idx = 0; idx < n_data; ++idx) {
          // Convert the data from millimeter to meter
          laserScan->mutable_scan_values()->Set(idx, static_cast<float>(data[idx]) / 1000);
        }
        // Send the data.
        informer->publish(laserScan);
        publishCounter = 0;
      }
      ++publishCounter;
    } else {
      ERROR_MSG("n_data: " << n_data << " vs. n_data_expected: " << n_data_expected)
    }

  }

  // Free the hokuyoReader
  hokuyoReader.Disconnect();

  return 0;

}

int setup(HokuyoReader &hokuyoReader) {
  if (hokuyoReader.Connect(hokuyoDevice.c_str(),baud)){
    std::cerr<<"testHokuyoReader: could not connect to the sensor."<<std::endl;
    exit(-1);
  }

  std::cout<<hokuyoReader.GetVendor()<<std::endl<<hokuyoReader.GetProduct()<<std::endl<<hokuyoReader.GetFirmware()
           <<std::endl<<hokuyoReader.GetProtocol()<<std::endl<<hokuyoReader.GetSerial()<<std::endl;


  std::cout<<"---------------"<<std::endl;
  std::cout<<hokuyoReader.GetModel()<<std::endl
           <<hokuyoReader.GetDistMin()<<std::endl
           <<hokuyoReader.GetDistMax()<<std::endl
           <<hokuyoReader.GetAngleRes()<<std::endl
           <<hokuyoReader.GetAngleMin()<<std::endl
           <<hokuyoReader.GetAngleMax()<<std::endl
           <<hokuyoReader.GetScanRate()<<std::endl<<std::endl;


  int sensorType= hokuyoReader._type;
  int newSkip;
  char *scanNameLocal = const_cast<char*>(scanName.data());

//    char sscanName[128] = scanName.c_str();
  //get the special skip value (if needed) and scan type, depending on the scanName and sensor type
  if (hokuyoReader.GetScanTypeAndSkipFromName(sensorType, scanNameLocal, &newSkip, &scanType)){
    std::cerr<<"testHokuyoReader: getScanTypeAndSkipFromName: Error getting the scan parameters."<<std::endl;
    exit(-1);
  }

  if (newSkip!=1){            //this means that a special value for skip must be used in order to request
    scanSkip=newSkip;         //a special scan from 04LX. Otherwise, just keep the above specified value of
  }                           //skip

  //set the scan parameters
  if (hokuyoReader.SetScanParams(scanNameLocal,scanStart, scanEnd, scanSkip, encoding, scanType)){
    std::cerr<<"testHokuyoReader: setScanSettings: Error setting the scan parameters."<<std::endl;
    exit(-1);
  }


  //get the number of output arguments (number of types of data that will be returned)
  //Since there is only one pointer provided to Getscan function, all the data will be written to that array.
  //So, if there is range and intensity data, it will be alternating and will have to be separated from
  //the array that looks like this: 1212121212 (where 1 represents first data type, 2 - second).
  //For 04LX it is possible to have 3 types of data, so they will come like this : 123123123123..
  int numOutputArgs=hokuyoReader.GetNumOutputArgs(sensorType,scanNameLocal);
  if (numOutputArgs < 0){
    std::cerr<<"testHokuyoReader: bad number of output args.."<<std::endl;
    return -1;
  }
  std::cout << "Number of output arguments = " << numOutputArgs <<std::endl;

  return 0;
}
int test(HokuyoReader &hokuyoReader) {

  for (int i=0;i<10;i++){
    if (hokuyoReader.GetScan(data, n_data)){
      std::cerr<<"testHokuyoReader: scan failed"<<std::endl;
      return -1;
    }
    else
      std::cerr<<"testHokuyoReader: iteration #"<<i<<" : scan succeeded! got " <<n_data<<" points"<<std::endl;
  }
  return 0;
}
