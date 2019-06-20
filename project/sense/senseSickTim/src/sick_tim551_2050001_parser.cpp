#include <sick_tim/sick_tim551_2050001_parser.h>

// RST Proto types
// #include <types/LocatedLaserScan.pb.h>
// #include <rst/geometry/Pose.pb.h>
#include <rst/vision/LaserScan.pb.h>


namespace sick_tim
{

SickTim5512050001Parser::SickTim5512050001Parser() :
    AbstractParser(),
    override_range_min_(0.05),
    override_range_max_(10.0),
    override_time_increment_(-1.0)
{
}

SickTim5512050001Parser::~SickTim5512050001Parser()
{
}

int SickTim5512050001Parser::parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                                            rst::vision::LaserScan &msg)
{
  static const size_t HEADER_FIELDS = 33;
  char* cur_field;
  size_t count;

  // Reserve sufficient space
  std::vector<char *> fields;
  fields.reserve(std::size_t(datagram_length / 2));

  // ----- only for debug output
  char datagram_copy[datagram_length + 1];
  strncpy(datagram_copy, datagram, datagram_length); // datagram will be changed by strtok
  datagram_copy[datagram_length] = 0;

  // ----- tokenize
  count = 0;
  cur_field = strtok(datagram, " ");
  fields.push_back(cur_field);

  while (cur_field != NULL)
  {
    cur_field = strtok(NULL, " ");
    fields.push_back(cur_field);
  }

  count = fields.size();

  // Validate header. Total number of tokens is highly unreliable as this may
  // change when you change the scanning range or the device name using SOPAS ET
  // tool. The header remains stable, however.
  if (count < HEADER_FIELDS)
  {
    printf(
        "received less fields than minimum fields (actual: %zu, minimum: %zu), ignoring scan", count, HEADER_FIELDS);
    printf("are you using the correct node? (124 --> sick_tim310_1130000m01, > 33 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
    // printf("received message was: %s", datagram_copy);
    return ExitError;
  }
  if (strcmp(fields[15], "0"))
  {
    printf("Field 15 of received data is not equal to 0 (%s). Unexpected data, ignoring scan", fields[15]);
    return ExitError;
  }
  if (strcmp(fields[20], "DIST1"))
  {
    printf("Field 20 of received data is not equal to DIST1i (%s). Unexpected data, ignoring scan", fields[20]);
    return ExitError;
  }

  // More in depth checks: check data length and RSSI availability
  // 25: Number of data (<= 10F)
  unsigned short int number_of_data = 0;
  sscanf(fields[25], "%hx", &number_of_data);

  if (number_of_data < 1 || number_of_data > 811)
  {
    printf("Data length is outside acceptable range 1-811 (%d). Ignoring scan", number_of_data);
    return ExitError;
  }
  if (count < HEADER_FIELDS + number_of_data)
  {
    printf("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
    return ExitError;
  }
  printf("Number of data: %d", number_of_data);

  // Calculate offset of field that contains indicator of whether or not RSSI data is included
  size_t rssi_idx = 26 + number_of_data;
  int tmp;
  sscanf(fields[rssi_idx], "%d", &tmp);
  bool rssi = tmp > 0;
  unsigned short int number_of_rssi_data = 0;
  if (rssi)
  {
    sscanf(fields[rssi_idx + 6], "%hx", &number_of_rssi_data);

    // Number of RSSI data should be equal to number of data
    if (number_of_rssi_data != number_of_data)
    {
      printf("Number of RSSI data is lower than number of range data (%d vs %d", number_of_data, number_of_rssi_data);
      return ExitError;
    }

    // Check if the total length is still appropriate.
    // RSSI data size = number of RSSI readings + 6 fields describing the data
    if (count < HEADER_FIELDS + number_of_data + number_of_rssi_data + 6)
    {
      printf("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
      return ExitError;
    }

    if (strcmp(fields[rssi_idx + 1], "RSSI1"))
    {
      printf("Field %zu of received data is not equal to RSSI1 (%s). Unexpected data, ignoring scan", rssi_idx + 1, fields[rssi_idx + 1]);
    }
  }

  // ----- read fields into msg
//  printf("publishing with frame_id %s", config.frame_id.c_str());

  // <STX> (\x02)
  // 0: Type of command (SN)
  // 1: Command (LMDscandata)
  // 2: Firmware version number (1)
  // 3: Device number (1)
  // 4: Serial number (eg. B96518)
  // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
  // 7: Telegram counter (eg. 99)
  // 8: Scan counter (eg. 9A)
  // 9: Time since startup (eg. 13C8E59)
  // 10: Time of transmission (eg. 13C9CBE)
  // 11 + 12: Input status (0 0)
  // 13 + 14: Output status (8 0)
  // 15: Reserved Byte A (0)

  // 16: Scanning Frequency (5DC)
  unsigned short scanning_freq = -1;
  sscanf(fields[16], "%hx", &scanning_freq);
//  msg.scan_time = 1.0 / (scanning_freq / 100.0);
  // printf("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, msg.scan_time);

  // 17: Measurement Frequency (36)
  unsigned short measurement_freq = -1;
  sscanf(fields[17], "%hx", &measurement_freq);
    // Some lasers may report incorrect measurement frequency
  // printf("measurement_freq: %d, time_increment: %f", measurement_freq, msg.time_increment);

  // 18: Number of encoders (0)
  // 19: Number of 16 bit channels (1)
  // 20: Measured data contents (DIST1)

  // 21: Scaling factor (3F800000)
  // ignored for now (is always 1.0):
//      unsigned int scaling_factor_int = -1;
//      sscanf(fields[21], "%x", &scaling_factor_int);
//
//      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
//      // printf("hex: %s, scaling_factor_int: %d, scaling_factor: %f", fields[21], scaling_factor_int, scaling_factor);

  // 22: Scaling offset (00000000) -- always 0
  // 23: Starting angle (FFF92230)
  int starting_angle = -1;
  sscanf(fields[23], "%x", &starting_angle);
  // msg.set_scan_angle_start((starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2);
  // printf("starting_angle: %d, angle_min: %f\n", starting_angle, msg.scan_angle_start());

  // 24: Angular step width (2710)
  unsigned short angular_step_width = -1;
  sscanf(fields[24], "%hx", &angular_step_width);
  float angular_step_width_radiant = (angular_step_width / 10000.0) / 180.0 * M_PI;
  // msg.set_scan_angle_increment((angular_step_width / 10000.0) / 180.0 * M_PI);
  // msg.set_scan_angle_end(msg.scan_angle_start() + (number_of_data - 1) * msg.scan_angle_increment());

  // 25: Number of data (<= 10F)
  // This is already determined above in number_of_data

  // adjust angle_min to min_ang config param
  int index_min = 0;
  // while (msg.scan_angle_increment() + msg.scan_angle_start() < config.min_ang)
  // {
  //   msg.set_scan_angle_start(msg.scan_angle_start() + msg.scan_angle_increment());
  //   index_min++;
  // }
  //
  // // adjust angle_max to max_ang config param
  int index_max = number_of_data - 1;
  // while (msg.scan_angle_end() - msg.scan_angle_increment() > config.max_ang)
  // {
  //   msg.set_scan_angle_end(msg.scan_angle_end() - msg.scan_angle_increment());
  //   index_max--;
  // }
  //
  // printf("index_min: %d, index_max: %d\n", index_min, index_max);
  // printf("angular_step_width: %d, angle_increment: %f, angle_max: %f\n", angular_step_width, msg.scan_angle_increment(), msg.scan_angle_start());

  // 26..26 + n - 1: Data_1 .. Data_n
  const int num_samples = index_max - index_min + 1;
  printf("num_samples: %d, msg.scan_values_size(): %d\n", num_samples, msg.scan_values_size());
  while (num_samples > msg.scan_values_size()) {
    msg.mutable_scan_values()->Add();
  }
  while (num_samples < msg.scan_values_size()) {
    msg.mutable_scan_values()->RemoveLast();
  }
   printf("Alloc: num_samples: %d, msg.scan_values_size(): %d\n", num_samples, msg.scan_values_size());
  for (int j = index_min; j <= index_max; ++j)
  {
    unsigned short range;
    sscanf(fields[j + 26], "%hx", &range);
    msg.set_scan_values(j - index_min, range / 1000.0);
    // msg.mutable_scan_values()->Set(j - index_min, range / 1000.0);
  }

  // 26 + n: RSSI data included
  // IF RSSI not included:
  //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
  //   26 + n + 4 .. count - 4 = device label
  //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
  //   <ETX> (\x03)

  // msg.set_scan_values_min(override_range_min_);
  // msg.set_scan_values_max(override_range_max_);
  msg.set_scan_angle((msg.scan_values_size()-1) * angular_step_width_radiant);

  // ----- adjust start time
  // - last scan point = now  ==>  first scan point = now - number_of_data * time increment
//  msg.header.stamp = start_time - ros::Duration().fromSec(number_of_data * msg.time_increment);

  // - shift forward to time of first published scan point
//  msg.header.stamp += ros::Duration().fromSec((double)index_min * msg.time_increment);

  // - add time offset (to account for USB latency etc.)
//  msg.header.stamp += ros::Duration().fromSec(config.time_offset);

  // ----- consistency check
// float expected_time_increment = msg.scan_time * msg.angle_increment / (2.0 * M_PI);
// if (fabs(expected_time_increment - msg.time_increment) > 0.00001)
// {
//   printf("The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
//       "Expected time_increment: %.9f, reported time_increment: %.9f. "
//       "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
//       expected_time_increment, msg.time_increment);
// }

  return ExitSuccess;
}

void SickTim5512050001Parser::set_range_min(float min)
{
  override_range_min_ = min;
}

void SickTim5512050001Parser::set_range_max(float max)
{
  override_range_max_ = max;
}

void SickTim5512050001Parser::set_time_increment(float time)
{
  override_time_increment_ = time;
}

} /* namespace sick_tim */
