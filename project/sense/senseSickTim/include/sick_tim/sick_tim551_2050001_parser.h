#ifndef SICK_TIM551_2050001_PARSER_H_
#define SICK_TIM551_2050001_PARSER_H_

#include "abstract_parser.h"

namespace sick_tim
{

class SickTim5512050001Parser : public AbstractParser
{
public:
  SickTim5512050001Parser();
  virtual ~SickTim5512050001Parser();

  virtual int parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                             rst::vision::LaserScan &msg);

  void set_range_min(float min);
  void set_range_max(float max);
  void set_time_increment(float time);

private:
  float override_range_min_, override_range_max_;
  float override_time_increment_;
};

} /* namespace sick_tim */
#endif /* SICK_TIM551_2050001_PARSER_H_ */
