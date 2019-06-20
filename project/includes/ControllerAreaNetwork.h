//============================================================================
// Name        : ControllerAreaNetwork.h
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Communication via CAN
//============================================================================

#ifndef AMIRO_CAN_H_
#define AMIRO_CAN_H_

// The CAN::* values are defined in the Constants.h, which
// is copied from the amiro-os repository
#include <Constants.h>

#include <linux/can.h>
#include <linux/can/raw.h>  // CAN_RAW_FILTER
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>  // int32

#include <string.h>
#include <stdio.h>
#include <unistd.h>  // sleep

#include <Color.h>  // Color types
#include <Types.h>  // types::position

#include <vector>


using namespace std;
using namespace amiro;

class ControllerAreaNetwork {
 public:
  ControllerAreaNetwork() {

    memset(&this->ifr, 0x0, sizeof(this->ifr));
    memset(&this->addr, 0x0, sizeof(this->addr));
    memset(&this->frame, 0x0, sizeof(this->frame));

    /* open CAN_RAW socket */
    this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    /* convert interface sting "can0" into interface index */
    strcpy(this->ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &this->ifr);

    /* setup address for bind */
    this->addr.can_ifindex = this->ifr.ifr_ifindex;
    this->addr.can_family = PF_CAN;

    /* bind socket to the can0 interface */
    ::bind(s, (struct sockaddr *)&this->addr, sizeof(addr));

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    /* Listen to own sending */
//    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
  }
  ~ControllerAreaNetwork() {
    close(s);
  }

 public:
  void setTargetSpeed(int v, int w) {
    /* first fill, then send the CAN frame */
    this->frame.can_id = 0;
    this->encodeDeviceId(&this->frame, CAN::TARGET_SPEED_ID);
    memcpy(&(this->frame.data[0]), &v, 4);
    memcpy(&(this->frame.data[4]), &w, 4);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void getAnyFrame() {
    const int nbytes = read(s, &frame, sizeof(frame));
     if (nbytes > 0) {
       printf("ID=0x%X DLC=%d data[0]=0x%X\n ",frame.can_id,frame.can_dlc,frame.data[0]);
     }
  }
  
  types::position getOdometry() {
    
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = ((CAN::ODOMETRY_ID & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | CAN::DI_WHEEL_DRIVE_ID;
//    rfilter[0].can_id   = 0x383u;
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    const int nbytes = read(s, &frame, sizeof(frame));
    types::position robotPosition;
    // TODO Make proper error handling, so that the calling function knows that the data is not correct
    if (nbytes > 0) {
      /* Process the data */
      robotPosition.x = (frame.data[0] << 8 | frame.data[1] << 16 | frame.data[2] << 24);
      robotPosition.y = (frame.data[3] << 8 | frame.data[4] << 16 | frame.data[5] << 24);
      robotPosition.f_z = (frame.data[6] << 8 | frame.data[7] << 16);

    } else {
      robotPosition.x = 0;
      robotPosition.y = 0;
      robotPosition.f_z = 0;
    }
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    return robotPosition;
  }

  void setOdometry(types::position robotPosition) {

    // NOTE: for robotPosition only values between [0, 2 * pi * 1e6] are allowed

    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::SET_ODOMETRY_ID);
    // Cut of the first byte, which precission is not needed
    int32_t x_mm = (robotPosition.x >> 8);
    int32_t y_mm = (robotPosition.y >> 8);
    int16_t f_z_mrad = int16_t(robotPosition.f_z >> 8 );
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&x_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[3]), (uint8_t *)&y_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[6]), (uint8_t *)&f_z_mrad, 2);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void setKinematicConstants(float Ed, float Eb) {

    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::SET_KINEMATIC_CONST_ID);
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&Ed, 4);
    memcpy((uint8_t *)&(this->frame.data[4]), (uint8_t *)&Eb, 4);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void setTargetPosition(types::position &robotPosition, uint16_t targetTimeMilliSeconds) {

    this->frame.can_id = 0;
    this->encodeDeviceId(&this->frame, CAN::TARGET_POSITION_ID);
    // Cut of the first byte, which precission is not needed
    int32_t x_mm = (robotPosition.x >> 8);
    int32_t f_z_mrad = int32_t(robotPosition.f_z >> 8 );
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&x_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[3]), (uint8_t *)&f_z_mrad, 3);
    memcpy((uint8_t *)&(this->frame.data[6]), (uint8_t *)&targetTimeMilliSeconds, 2);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  int getActualSpeed(int32_t &v, int32_t &w) {

    int returnValue = 0;
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = ((CAN::ACTUAL_SPEED_ID & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | CAN::DI_WHEEL_DRIVE_ID;
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    const int nbytes = read(s, &frame, sizeof(frame));
    /* Process the data */
     if (nbytes > 0) {
       memcpy(&v, &(frame.data[0]), 4);
       memcpy(&w, &(frame.data[4]), 4);
     } else {
       returnValue = -1;
     }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;

  }
  
  int getProximityFloorValue(std::vector<uint16_t> &proximityValues) {

    int returnValue = 0;
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = getCanFilter(CAN::PROXIMITY_FLOOR_ID, CAN::DI_WHEEL_DRIVE_ID);
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    const int nbytes = read(s, &frame, sizeof(frame));
    //std::cout << "nbytes Floor Value: " << nbytes <<  std::endl;
    /* Process the data */
     if (nbytes > 0) {
       memcpy(&proximityValues[0], &(frame.data[0]), 2);  // Front right
       memcpy(&proximityValues[1], &(frame.data[2]), 2);  // Wheel right
       memcpy(&proximityValues[2], &(frame.data[4]), 2);  // Wheel left
       memcpy(&proximityValues[3], &(frame.data[6]), 2);  // Front left
     } else {
       returnValue = -1;
     }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;

  }

  int getProximityRingValue(std::vector<uint16_t> &proximityValues) {

    int returnValue = 0;
    struct can_filter rfilter[8];

    /* Set the filter for the message */
    for (int idx = 0; idx < 8; ++idx ) {
      rfilter[idx].can_id   = ((CAN::PROXIMITY_RING_ID(idx) & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | CAN::POWER_MANAGEMENT_ID;
      rfilter[idx].can_mask = CAN_SFF_MASK;
    }

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    for (int sensorIdx = 0; sensorIdx < 8; ++sensorIdx) {
      const int nbytes = read(s, &frame, sizeof(frame));
       if (nbytes > 0) {
         int index = decodeDeviceId(&frame) & 0x7;
         memcpy(&proximityValues[index], &(frame.data[0]), 2);
       } else {
         returnValue = -1;
       }
    }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  int getMagnetometerValue(std::vector<int32_t> &magnetometerValues) {

    int returnValue = 0;
    struct can_filter rfilter[3];

    /* Set the filter for the message */
    for (int axis = 0; axis < 3; ++axis ) {
      rfilter[axis].can_id   = ((CAN::MAGNETOMETER_ID(axis) & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | CAN::DI_WHEEL_DRIVE_ID;
      rfilter[axis].can_mask = CAN_SFF_MASK;
    }

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    for (int axis = 0; axis < 3; ++axis) {
      const int nbytes = read(s, &frame, sizeof(frame));
       if (nbytes > 0) {
         int index = decodeDeviceId(&frame) & 0x3;
         memcpy(&magnetometerValues[index], &(frame.data[0]), 4);
       } else {
         returnValue = -1;
       }
    }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;
  }

  int getGyroscopeValue(std::vector<int16_t> &gyroscopeValues) {

    int returnValue = 0;
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    //rfilter[0].can_id   = getCanFilter(CAN::GYROSCOPE_ID, CAN::DI_WHEEL_DRIVE_ID);
    rfilter[0].can_id   = ((CAN::GYROSCOPE_ID & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | CAN::DI_WHEEL_DRIVE_ID;
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    //std::cout << "Hasen sind toll" <<  std::endl;
    const int nbytes = read(s, &frame, sizeof(frame));
    //std::cout << "nbytes Gyro Value: " << nbytes <<  std::endl;
    /* Process the data */
     if (nbytes > 0) {
       memcpy(&gyroscopeValues[0], &(frame.data[0]), 2); 
       memcpy(&gyroscopeValues[1], &(frame.data[2]), 2);  
       memcpy(&gyroscopeValues[2], &(frame.data[4]), 2);  
     } else {
       returnValue = -1;
     }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  void sendMotorVector(float x, float y, uint32_t id = CAN::DIRECTION_VECTOR_TWB_ID){
      this->frame.can_id = 0;
      this->encodeDeviceId(&frame, id);
      memcpy(&(this->frame.data[0]), &x, 4);
      memcpy(&(this->frame.data[4]), &y, 4);
      this->frame.can_dlc = 8;
      this->transmitMessage(&frame);
  }

  void sendTwbVector(float x, float y){
      sendMotorVector(x, y, CAN::DIRECTION_VECTOR_TWB_ID);
  }
  
  void sendProxVector(float x, float y){
      sendMotorVector(x, y, CAN::DIRECTION_VECTOR_PROX_ID);
  }

  void broadcastShutdown() {
      this->frame.can_id = 0;
      this->encodeDeviceId(&frame, CAN::BROADCAST_SHUTDOWN);
      const uint16_t data = CAN::SHUTDOWN_MAGIC;
      memcpy(&(this->frame.data[0]),&data,2);
      this->frame.can_dlc = 2;
      this->transmitMessage(&frame);
  }

  void setLightBrightness(uint8_t brightness) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::BRIGHTNESS_ID);
    memcpy(&(this->frame.data[0]),&brightness,1);
    this->frame.can_dlc = 1;
    this->transmitMessage(&frame);
  }

  void setLightColor(int index, Color color) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::COLOR_ID(index));
    uint8_t redColor = color.getRed();
    uint8_t greenColor = color.getGreen();
    uint8_t blueColor = color.getBlue();
    memcpy(&(this->frame.data[0]), &redColor,1);
    memcpy(&(this->frame.data[1]), &greenColor,1);
    memcpy(&(this->frame.data[2]), &blueColor,1);
    this->frame.can_dlc = 3;
    this->transmitMessage(&frame);
  }

  void calibrateRingProximitySensors() {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::CALIBRATE_PROXIMITY_RING);
    this->frame.can_dlc = 0;
    this->transmitMessage(&frame);
  }

  void calibrateFloorProximitySensors() {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::CALIBRATE_PROXIMITY_FLOOR);
    this->frame.can_dlc = 0;
    this->transmitMessage(&frame);
  }

 private:
   
  int getCanFilter(int deviceID, int boardID) {
    return ((deviceID & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | boardID;
  }
  
  void encodeDeviceId(struct can_frame *frame, int device) {
    frame->can_id |= (device & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT;
  }

  int decodeDeviceId(struct can_frame *frame) {
    return (frame->can_id >> CAN::DEVICE_ID_SHIFT) & CAN::DEVICE_ID_MASK;
  }

  ssize_t transmitMessage(struct can_frame *frame) {
    this->encodeBoardId(frame, CAN::COGNITION);
    return write(this->s, &this->frame, sizeof(this->frame));
  }

  void encodeBoardId(struct can_frame *frame, int board) {
    frame->can_id |= (board & CAN::BOARD_ID_MASK) << CAN::BOARD_ID_SHIFT;
  }

  struct ifreq ifr;
  struct sockaddr_can addr;
  struct can_frame frame;
  int s;
};

#endif /* AMIRO_CAN_H_ */
