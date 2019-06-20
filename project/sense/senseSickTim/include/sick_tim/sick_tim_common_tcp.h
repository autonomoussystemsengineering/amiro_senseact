#ifndef SICK_TIM3XX_COMMON_TCP_H
#define SICK_TIM3XX_COMMON_TCP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <boost/asio.hpp>

#include "sick_tim_common.h"

namespace sick_tim
{

class SickTimCommonTcp : public SickTimCommon
{
public:
  SickTimCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, AbstractParser* parser, std::string rsboutscope);
  virtual ~SickTimCommonTcp();

protected:
  virtual int init_device();
  virtual int close_device();

  /// Send a SOPAS command to the device and print out the response to the console.
  virtual int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply);

  /// Read a datagram from the device.
  /**
   * \param [in] receiveBuffer data buffer to fill
   * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
   * \param [out] actual_length the actual amount of data written
   */
  virtual int get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length);
 
  // Helpers for boost asio
  int readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read = 0, bool *exception_occured = 0);
  void handleRead(boost::system::error_code error, size_t bytes_transfered);
  void checkDeadline();

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::deadline_timer deadline_;
  boost::asio::streambuf input_buffer_;
  boost::system::error_code ec_;
  size_t bytes_transfered_;

  std::string hostname_;
  std::string port_;
  int timelimit_;
};

inline void SickTimCommonTcp::handleRead(boost::system::error_code error, size_t bytes_transfered)
{
    ec_ = error;
    bytes_transfered_ += bytes_transfered;
}

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_COMMON_TCP_H */

