//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads messages on the vcan0 interface
//============================================================================


#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <stdio.h>
#include <string.h>

#include <stdio.h>
#include <unistd.h> 

#include <iostream>

int main(int argc, char **argv)
{
struct ifreq ifr;
struct sockaddr_can addr;
struct can_frame frame;
int s;

memset(&ifr, 0x0, sizeof(ifr));
memset(&addr, 0x0, sizeof(addr));
memset(&frame, 0x0, sizeof(frame));

/* open CAN_RAW socket */
s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

/* convert interface sting "can0" into interface index */
strcpy(ifr.ifr_name, "vcan0");
ioctl(s, SIOCGIFINDEX, &ifr);

/* setup address for bind */
addr.can_ifindex = ifr.ifr_ifindex;
addr.can_family = PF_CAN;

/* bind socket to the can0 interface */
bind(s, (struct sockaddr *)&addr, sizeof(addr));

while (true) {
  int nbytes = read(s, &frame, sizeof(frame));
  if (nbytes > 0) {
    printf("ID=0x%X DLC=%d data[0]=0x%X\n ",frame.can_id,frame.can_dlc,frame.data[0]);
  }
}

close(s);
return 0;

}
