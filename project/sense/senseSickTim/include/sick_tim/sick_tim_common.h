#ifndef SICK_TIM3XX_COMMON_H_
#define SICK_TIM3XX_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#include <rsb/Factory.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>

#include <sick_tim/SickTimConfig.h>

#include "abstract_parser.h"

namespace sick_tim
{

class SickTimCommon
{
public:
  SickTimCommon(AbstractParser* parser, std::string rsboutscope);
  virtual ~SickTimCommon();
  virtual int init();
  int loopOnce();
  void check_angle_range(SickTimConfig &conf);
  void update_config(sick_tim::SickTimConfig &new_config, uint32_t level = 0);
  double get_expected_frequency() const { return expectedFrequency_; }

protected:
  virtual int init_device() = 0;
  virtual int init_scanner();
  virtual int stop_scanner();
  virtual int close_device() = 0;

  /// Send a SOPAS command to the device and print out the response to the console.
  /**
   * \param [in] request the command to send.
   * \param [out] reply if not NULL, will be filled with the reply package to the command.
   */
  virtual int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply) = 0;

  /// Read a datagram from the device.
  /**
   * \param [in] receiveBuffer data buffer to fill
   * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
   * \param [out] actual_length the actual amount of data written
   */
  virtual int get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length) = 0;

  bool isCompatibleDevice(const std::string identStr) const;

private:
  // RSB
  // rsb::Informer<rst::vision::LocatedLaserScan>::Ptr pub_;
  rsb::Informer<rst::vision::LaserScan>::Ptr pub_;

  // Diagnostics
  double expectedFrequency_;

  // Dynamic Reconfigure
  SickTimConfig config_;

  // Parser
  AbstractParser* parser_;

  // Message
  // boost::shared_ptr<rst::vision::LocatedLaserScan> msg_ptr;
  boost::shared_ptr<rst::vision::LaserScan> msg_ptr;
};

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_COMMON_H_ */
