#ifndef SICK_TIM3XX_COMMON_USB_H_
#define SICK_TIM3XX_COMMON_USB_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include "sick_tim_common.h"

namespace sick_tim
{

class SickTimCommonUsb : public SickTimCommon
{
public:
  SickTimCommonUsb(AbstractParser* parser, std::string rsboutscope);
  virtual ~SickTimCommonUsb();

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

private:
  static const unsigned int USB_TIMEOUT = 1000; // milliseconds

  ssize_t getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID, libusb_device ***list);
  void freeSOPASDeviceList(libusb_device **list);

  void printUSBDeviceDetails(struct libusb_device_descriptor desc);
  void printUSBInterfaceDetails(libusb_device* device);
  void printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices);

  // libusb
  libusb_context *ctx_;
  ssize_t numberOfDevices_;
  libusb_device **devices_;
  libusb_device_handle *device_handle_;
};

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_COMMON_USB_H_ */
