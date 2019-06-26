#ifndef __VELODYNE_RAW_POS_DATA_H
#define __VELODYNE_RAW_POS_DATA_H

#include <velodyne_msgs/VelodynePosPacket.h>

namespace velodyne_rawPosData
{
  class RawPosData
  {
  public:
    RawPosData();
    ~RawPosData();
    
    bool unpack(const velodyne_msgs::VelodynePosPacket::ConstPtr &ppkt,
		double &sec_past_hour,
		std::string &nmea_gprmc);

  };

} // namespace velodyne_rawPosData

#endif // __VELODYNE_RAW_POS_DATA_H
