#ifndef SICK_TIM3XX_DATAGRAM_TEST_H_
#define SICK_TIM3XX_DATAGRAM_TEST_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sick_tim/SickTimConfig.h>

#include "abstract_parser.h"

namespace sick_tim
{

class SickTimDatagramTest
{
public:
  SickTimDatagramTest(AbstractParser* parser);
  virtual ~SickTimDatagramTest();
  void check_angle_range(SickTimConfig &conf);
  void update_config(sick_tim::SickTimConfig &new_config, uint32_t level = 0);

private:
  ros::NodeHandle nh_;

  // publisher to "scan" topic
  ros::Publisher pub_;

  // subscriber to "datagram" topic
  ros::Subscriber sub_;
  void datagramCB(const std_msgs::String::ConstPtr &msg);

  // Dynamic Reconfigure
  SickTimConfig config_;
  dynamic_reconfigure::Server<sick_tim::SickTimConfig> dynamic_reconfigure_server_;

  AbstractParser* parser_;
};

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_DATAGRAM_TEST_H_ */
