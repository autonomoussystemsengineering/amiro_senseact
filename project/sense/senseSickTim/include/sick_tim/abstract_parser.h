#ifndef ABSTRACT_PARSER_H_
#define ABSTRACT_PARSER_H_

#include <sick_tim/SickTimConfig.h>
// RST Proto types
#include <rst/vision/LaserScan.pb.h>
// #include <types/LocatedLaserScan.pb.h>
// #include <rst/geometry/Pose.pb.h>

namespace sick_tim
{

enum ExitCode
{
    ExitSuccess = 0
    , ExitError = 1    // non-fatal error, retry
    , ExitFatal = 2    // fatal error, exit
};

class AbstractParser
{
public:
  AbstractParser();
  virtual ~AbstractParser();

  virtual int parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                             rst::vision::LaserScan &msg) = 0;
};

} /* namespace sick_tim */
#endif /* ABSTRACT_PARSER_H_ */
