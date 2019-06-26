#include <velodyne_driver/rawPosData.h>
#include <string> 
#include <iostream>

namespace velodyne_rawPosData
{
  // Position and length specs for interpreting position packet
  // Refer to Velodyne VLP-16 manual
  uint const IDX_TIME = 198;
  uint const LEN_TIME = 4;  // number of bytes representing timestamp
  uint const IDX_NMEA = IDX_TIME + LEN_TIME + 4; // 4 unused bytes 
  uint const LEN_NMEA = 72 + 234; // number of bytes (max) for nmea message 

  RawPosData::RawPosData(){}

  RawPosData::~RawPosData(){}

  bool RawPosData::unpack(const velodyne_msgs::VelodynePosPacket::ConstPtr &ppkt,
			  double &sec_past_hour,
			  std::string &nmea_gprmc){

    // timestamp (past the hour) on the position packet
    uint* timestamp_packet_microsec = (uint*)&(ppkt->data[IDX_TIME]);
    sec_past_hour = (double)(*timestamp_packet_microsec) / 1e6;

    nmea_gprmc = std::string( (const char *)&(ppkt->data[IDX_NMEA]), LEN_NMEA );
    std::size_t const checksum_idx = nmea_gprmc.find_first_of("*");
    if(checksum_idx == std::string::npos){
      return false; // checksum * not found, unpack failed
    }
    nmea_gprmc.resize(checksum_idx + 3); // checksum is 2 bytes long

    // verify checksum
    std::string checksum_str = nmea_gprmc.substr( checksum_idx + 1, 2 );
    char checksum = 0;
    try{
      checksum = std::stoi( checksum_str, 0, 16 ); // c++11 req'd
    }catch (const std::invalid_argument& e){
      std::cout << "Invalid checksum argument for string to int conversion\n"
                << "NMEA: " << nmea_gprmc
                << "Checksum: " << checksum_str.c_str() << std::endl;
      return false;
    }
    char checksum_calc = 0;
    for(std::size_t i = 1; i < checksum_idx; i++){
      checksum_calc = checksum_calc ^ nmea_gprmc[i];
    }
    if(checksum != checksum_calc){
      return false;
    }
    return true;
  }

}
