#include <sick_tim/sick_tim_common_usb.h>

namespace sick_tim
{

SickTimCommonUsb::SickTimCommonUsb(AbstractParser* parser, std::string rsboutscope) : SickTimCommon(parser, rsboutscope),
    ctx_(NULL), numberOfDevices_(0), devices_(NULL), device_handle_(NULL)
{
}

SickTimCommonUsb::~SickTimCommonUsb()
{
  stop_scanner();
  close_device();
}

int SickTimCommonUsb::close_device()
{
  int result = 0;
  if (device_handle_ != NULL)
  {
    /*
     * Release the interface
     */
    result = libusb_release_interface(device_handle_, 0);
    if (result != 0)
      printf("LIBUSB - Cannot Release Interface!\n");
    else
      printf("LIBUSB - Released Interface.\n");

    /*
     * Close the device handle.
     */
    libusb_close(device_handle_);
  }

  /*
   * Free the list of the USB devices.
   */
  freeSOPASDeviceList(devices_);

  /*
   * Close the LIBUSB session.
   */
  libusb_exit(ctx_);
  return result;
}

/**
 * Returns a list of USB devices currently attached to the system and matching the given vendorID and productID.
 */
ssize_t SickTimCommonUsb::getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID,
                                             libusb_device ***list)
{
  libusb_device **resultDevices = NULL;
  ssize_t numberOfResultDevices = 0;
  libusb_device **devices;

  /*
   * Get a list of all USB devices connected.
   */
  ssize_t numberOfDevices = libusb_get_device_list(ctx, &devices);

  /*
   * Iterate through the list of the connected USB devices and search for devices with the given vendorID and productID.
   */
  for (ssize_t i = 0; i < numberOfDevices; i++)
  {
    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      printf("LIBUSB - Failed to get device descriptor");
      continue;
    }

    if (desc.idVendor == vendorID && desc.idProduct == 0x5001)
    {
      /*
       * Add the matching device to the function result list and increase the device reference count.
       */
      resultDevices = (libusb_device **)realloc(resultDevices, sizeof(libusb_device *) * (numberOfResultDevices + 2));
      if (resultDevices == NULL)
      {
        printf("LIBUSB - Failed to allocate memory for the device result list.");
      }
      else
      {
        resultDevices[numberOfResultDevices] = devices[i];
        resultDevices[numberOfResultDevices + 1] = NULL;
        libusb_ref_device(devices[i]);
        numberOfResultDevices++;
      }
    }
  }

  /*
   * Free the list of the connected USB devices and decrease the device reference count.
   */
  libusb_free_device_list(devices, 1);

  /*
   * Prepare the return values of the function.
   */
  *list = resultDevices;
  return numberOfResultDevices;
}

/*
 * Free the list of devices obtained from the function 'getSOPASDeviceList'.
 */
void SickTimCommonUsb::freeSOPASDeviceList(libusb_device **list)
{
  if (!list)
    return;

  int i = 0;
  struct libusb_device *dev;
  while ((dev = list[i++]) != NULL)
    libusb_unref_device(dev);

  free(list);
}

/*
 * Print the device details such as USB device class, vendor id and product id to the console.
 */
void SickTimCommonUsb::printUSBDeviceDetails(struct libusb_device_descriptor desc)
{
  printf("Device Class: 0x%x", desc.bDeviceClass);
  printf("VendorID:     0x%x", desc.idVendor);
  printf("ProductID:    0x%x", desc.idProduct);
}

/*
 * Iterate through the the interfaces of the USB device and print out the interface details to the console.
 */
void SickTimCommonUsb::printUSBInterfaceDetails(libusb_device* device)
{
  struct libusb_config_descriptor *config;

  /*
   * Get a USB configuration descriptor based on its index.
   */
  libusb_get_config_descriptor(device, 0, &config);

  printf("Interfaces: %i", (int)config->bNumInterfaces);
  printf("----------------------------------------");

  const struct libusb_interface *interface;
  const struct libusb_interface_descriptor *interface_descriptor;
  const struct libusb_endpoint_descriptor *endpoint_descriptor;

  int i, j, k;
  for (i = 0; i < config->bNumInterfaces; i++)
  {
    interface = &config->interface[i];
    printf("Number of alternate settings: %i", interface->num_altsetting);

    for (j = 0; j < interface->num_altsetting; j++)
    {
      interface_descriptor = &interface->altsetting[j];

      printf("Interface number: %i", (int)interface_descriptor->bInterfaceNumber);
      printf("Number of endpoints: %i", (int)interface_descriptor->bNumEndpoints);

      for (k = 0; k < interface_descriptor->bNumEndpoints; k++)
      {
        endpoint_descriptor = &interface_descriptor->endpoint[k];
        printf("Descriptor Type: %i", endpoint_descriptor->bDescriptorType);
        printf("EP Address: %i", endpoint_descriptor->bEndpointAddress);
      }
    }

    if (i < config->bNumInterfaces - 1)
    {
      printf("----------------------------------------");
    }
  }

  /*
   * Free the configuration descriptor obtained from 'libusb_get_config_descriptor'
   */
  libusb_free_config_descriptor(config);
}

/**
 * Print the USB device information of the connected TIM3xx devices to the console.
 */
void SickTimCommonUsb::printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices)
{
  ssize_t i;
  for (i = 0; i < numberOfDevices; i++)
  {
    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      printf("LIBUSB - Failed to get device descriptor");
      continue;
    }
    if (result == 0)
    {
      printf("SICK AG - TIM3XX - [%zu]", (i + 1));
      printf("----------------------------------------");
      printUSBDeviceDetails(desc);
      printf("----------------------------------------");
      printUSBInterfaceDetails(devices[i]);
      printf("----------------------------------------");
    }
  }

  if (numberOfDevices == 0)
  {
    printf("LIBUSB - No SICK TIM device connected.");
  }
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTimCommonUsb::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply)
{
  if (device_handle_ == NULL) {
    printf("LIBUSB - device not open");
    return ExitError;
  }

  int result = 0;
  unsigned char receiveBuffer[65536];

  /*
   * Write a SOPAS variable read request to the device.
   */
  printf("LIBUSB - Write data... %s", request);

  int actual_length = 0;
  int requestLength = strlen(request);
  result = libusb_bulk_transfer(device_handle_, (2 | LIBUSB_ENDPOINT_OUT), (unsigned char*)request, requestLength,
                                &actual_length, 0);
  if (result != 0 || actual_length != requestLength)
  {
    printf("LIBUSB - Write Error: %i.", result);
    return result;
  }

  /*
   * Read the SOPAS device response with the given timeout.
   */
  result = libusb_bulk_transfer(device_handle_, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual_length, USB_TIMEOUT);
  if (result != 0)
  {
    printf("LIBUSB - Read Error: %i.", result);
    return result;
  }

  receiveBuffer[actual_length] = 0;
  printf("LIBUSB - Read data...  %s", receiveBuffer);
  if(reply) {
      reply->clear();
      for(int i = 0; i < actual_length; i++) {
          reply->push_back(receiveBuffer[i]);
      }
  }

  return result;
}

/*
 * provided as a separate method (not inside constructor) so we can return error codes
 */
int SickTimCommonUsb::init_device()
{
  /*
   * Create and initialize a new LIBUSB session.
   */
  int result = libusb_init(&ctx_);
  if (result != 0)
  {
    printf("LIBUSB - Initialization failed with the following error code: %i.", result);
    return ExitError;
  }

  /*
   * Set the verbosity level to 3 as suggested in the documentation.
   */
  libusb_set_debug(ctx_, 3);

  /*
   * Get a list of all SICK TIM3xx devices connected to the USB bus.
   *
   * As a shortcut, you can also use the LIBUSB function:
   * libusb_open_device_with_vid_pid(ctx, 0x19A2, 0x5001).
   */
  int vendorID = 0x19A2; // SICK AG
  int deviceID = 0x5001; // TIM3XX
  numberOfDevices_ = getSOPASDeviceList(ctx_, vendorID, deviceID, &devices_);

  /*
   * If available, open the first SICK TIM3xx device.
   */
  if (numberOfDevices_ == 0)
  {
    printf("No SICK TiM devices connected!");
    return ExitError;
  }
  else if (numberOfDevices_ > 1)
  {
    printf("%zu TiM3xx scanners connected, using the first one", numberOfDevices_);
  }

  /*
   * Print out the SOPAS device information to the console.
   */
  printSOPASDeviceInformation(numberOfDevices_, devices_);

  /*
   * Open the device handle and detach all kernel drivers.
   */
  libusb_open(devices_[0], &device_handle_);
  if (device_handle_ == NULL)
  {
    printf("LIBUSB - Cannot open device; please read sick_tim/udev/README");
    return ExitError;
  }
  else
  {
    printf("LIBUSB - Device opened");
  }

  if (libusb_kernel_driver_active(device_handle_, 0) == 1)
  {
    printf("LIBUSB - Kernel driver active");
    if (libusb_detach_kernel_driver(device_handle_, 0) == 0)
    {
      printf("LIBUSB - Kernel driver detached!");
    }
  }

  /*
   * Claim the interface 0
   */
  result = libusb_claim_interface(device_handle_, 0);
  if (result < 0)
  {
    printf("LIBUSB - Cannot claim interface");
    return ExitError;
  }
  else
  {
    printf("LIBUSB - Claimed interface");
  }

  return ExitSuccess;
}

int SickTimCommonUsb::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
{
  int result = libusb_bulk_transfer(device_handle_, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, bufferSize - 1, actual_length,
                                USB_TIMEOUT);   // read up to bufferSize - 1 to leave space for \0
  if (result != 0)
  {
    if (result == LIBUSB_ERROR_TIMEOUT)
    {
      printf("LIBUSB - Read Error: LIBUSB_ERROR_TIMEOUT.");
      *actual_length = 0;
      return ExitSuccess; // return success with size 0 to continue looping
    }
    else
    {
      printf("LIBUSB - Read Error: %i.", result);
      return result; // return failure to exit node
    }
  }

  receiveBuffer[*actual_length] = 0;
  return ExitSuccess;
}

} /* namespace sick_tim */
