#include <sick_tim/sick_tim_common.h>

#include <cstdio>
#include <cstring>

namespace sick_tim
{

SickTimCommon::SickTimCommon(AbstractParser* parser, std::string rsboutscope) :
    expectedFrequency_(15.0), parser_(parser)
    // FIXME All Tims have 15Hz?
{

  rsb::Factory &nh_ = rsb::getFactory();

  // Register converter
  // boost::shared_ptr< rsb::converter::ProtocolBufferConverter< rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  // rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter< rst::vision::LaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

  // scan publisher
  pub_ = nh_.createInformer<rst::vision::LaserScan>(rsboutscope);
  // pub_ = nh_.createInformer<rst::vision::LocatedLaserScan>(rsboutscope);

  // Allocate memory for the message
  // msg_ptr = boost::shared_ptr<rst::vision::LocatedLaserScan>(new rst::vision::LocatedLaserScan);
  msg_ptr = boost::shared_ptr<rst::vision::LaserScan>(new rst::vision::LaserScan);
}

int SickTimCommon::stop_scanner()
{
  /*
   * Stop streaming measurements
   */
  const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
  int result = sendSOPASCommand(requestScanData0, NULL);
  if (result != 0)
    // use printf because we cannot use printf from the destructor
    printf("\nSOPAS - Error stopping streaming scan data!\n");
  else
    printf("\nSOPAS - Stopped streaming scan data.\n");

  return result;
}

SickTimCommon::~SickTimCommon()
{

  printf("sick_tim driver exiting.\n");
}


int SickTimCommon::init()
{
  int result = init_device();
  if(result != 0) {
      printf("Failed to init device: %d", result);
      return result;
  }
  result = init_scanner();
  if(result != 0) {
      printf("Failed to init scanner: %d", result);
  }
  return result;
}

int SickTimCommon::init_scanner()
{
  /*
   * Read the SOPAS variable 'DeviceIdent' by index.
   */
  const char requestDeviceIdent[] = "\x02sRI0\x03\0";
  std::vector<unsigned char> identReply;
  int result = sendSOPASCommand(requestDeviceIdent, &identReply);
  if (result != 0)
  {
    printf("SOPAS - Error reading variable 'DeviceIdent'.");
  }

  /*
   * Read the SOPAS variable 'SerialNumber' by name.
   */
  const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
  std::vector<unsigned char> serialReply;
  result = sendSOPASCommand(requestSerialNumber, &serialReply);
  if (result != 0)
  {
    printf("SOPAS - Error reading variable 'SerialNumber'.");
  }

  // set hardware ID based on DeviceIdent and SerialNumber
  identReply.push_back(0);  // add \0 to convert to string
  serialReply.push_back(0);
  std::string identStr;
  for (std::vector<unsigned char>::iterator it = identReply.begin(); it != identReply.end(); it++)
  {
    if (*it > 13) // filter control characters for display
      identStr.push_back(*it);
  }
  std::string serialStr;
  for (std::vector<unsigned char>::iterator it = serialReply.begin(); it != serialReply.end(); it++)
  {
    if (*it > 13)
      serialStr.push_back(*it);
  }

  if (!isCompatibleDevice(identStr))
    return ExitFatal;

  /*
   * Read the SOPAS variable 'FirmwareVersion' by name.
   */
  const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
  result = sendSOPASCommand(requestFirmwareVersion, NULL);
  if (result != 0)
  {
    printf("SOPAS - Error reading variable 'FirmwareVersion'.");
  }

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(requestScanData, NULL);
  if (result != 0)
  {
    printf("SOPAS - Error starting to stream 'LMDscandata'.");
    return ExitError;
  }

  return ExitSuccess;
}

bool sick_tim::SickTimCommon::isCompatibleDevice(const std::string identStr) const
{
  char device_string[7];
  int version_major = -1;
  int version_minor = -1;

  if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
             &version_major, &version_minor) == 3
      && strncmp("TiM3", device_string, 4) == 0
      && version_major >= 2 && version_minor >= 50)
  {
    printf("This scanner model/firmware combination does not support ranging output!");
    printf("Supported scanners: TiM5xx: all firmware versions; TiM3xx: firmware versions < V2.50.");
    printf("This is a %s, firmware version %d.%d", device_string, version_major, version_minor);

    return false;
  }
  return true;
}

int SickTimCommon::loopOnce()
{
  printf("START LOOPING\n");
  unsigned char receiveBuffer[65536];
  int actual_length = 0;
  static unsigned int iteration_count = 0;

  int result = get_datagram(receiveBuffer, 65536, &actual_length);
  if (result != 0)
  {
      printf("Read Error when getting datagram: %i.\n", result);
      return ExitError; // return failure to exit node
  }
  if(actual_length <= 0)
      return ExitSuccess; // return success to continue looping

  printf("START 1\n");
  // ----- if requested, skip frames
//  if (iteration_count++ % (config_.skip + 1) != 0)
//    return ExitSuccess;
  printf("START 2\n");

  /*
   * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
   */
  char* buffer_pos = (char*)receiveBuffer;
  char *dstart, *dend;
  while( (dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)) )
  {
    size_t dlength = dend - dstart;
    *dend = '\0';
    dstart++;
    int success = parser_->parse_datagram(dstart, dlength, config_, *msg_ptr);
    if (success == ExitSuccess) {
      printf("SEND DATA\n");
      pub_->publish(msg_ptr);
    }
    buffer_pos = dend + 1;
  }

  return ExitSuccess; // return success to continue looping
}

void SickTimCommon::check_angle_range(SickTimConfig &conf)
{
  if (conf.min_ang > conf.max_ang)
  {
    printf("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
    conf.min_ang = conf.max_ang;
  }
}

void SickTimCommon::update_config(sick_tim::SickTimConfig &new_config, uint32_t level)
{
  check_angle_range(new_config);
  config_ = new_config;
}

} /* namespace sick_tim */
