/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing Group
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Velodyne LiDAR, Algorithms and Signal Processing Group
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <velodyne_pointcloud/time_conv.hpp>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  double const SEC_IN_HOUR      = 3600;
  double const SEC_IN_DAY       = 24 * SEC_IN_HOUR;
  double const SEC_IN_WEEK      = 7 * SEC_IN_DAY;
  double const SEC_IN_GPS_CYCLE = 1024 * SEC_IN_WEEK;

  RawData::RawData() {
    gps_h_past_week_ = 0;
    prev_packet_time_sec_past_hour_ = -1;

//    // Initialize OpenCV images with designated width and height
    int num_x_pix = (int) ceil(2 * M_PI / azi_res_rad);
//    int num_x_pix = (int) ceil(2.0 / 3 * M_PI / azi_res_rad);
    // TODO Leaving a 0.5 degree buffer on top and bottom
    int num_y_pix = (int) ceil((40.0 + (0.5 * 2)) / 180 * M_PI / elev_res_rad);
    ROS_INFO("Initializing CV Windows");
    intensity_img = cv::Mat(num_y_pix, num_x_pix, CV_8U, 0.0);
    depth_img = cv::Mat(num_y_pix, num_x_pix, CV_32F, 0.0);
    valid_img = cv::Mat(num_y_pix, num_x_pix, CV_8U, 0.0);
    fire_img = cv::Mat(128, 2000, CV_8U, 0.0);
    fire_id = std::vector<int>(2000, 0);
    img_timestamp = 0.0;

    // Hard-coded laser map
    laser_map = std::vector<int>(128, 0);
    laser_map.at( 127 ) =  36;

    laser_map.at(59) = 0;
    laser_map.at(74) = 1;
    laser_map.at(41) = 2;
    laser_map.at(120) = 3;
    laser_map.at(31) = 4;
    laser_map.at(110) = 5;
    laser_map.at(13) = 6;
    laser_map.at(92) = 7;
    laser_map.at(51) = 8;
    laser_map.at(66) = 9;
    laser_map.at(33) = 10;
    laser_map.at(112) = 11;
    laser_map.at(23) = 12;
    laser_map.at(102) = 13;
    laser_map.at(5) = 14;
    laser_map.at(84) = 15;
    laser_map.at(107) = 16;
    laser_map.at(10) = 17;
    laser_map.at(89) = 18;
    laser_map.at(56) = 19;
    laser_map.at(79) = 20;
    laser_map.at(46) = 21;
    laser_map.at(125) = 22;
    laser_map.at(28) = 23;
    laser_map.at(99) = 24;
    laser_map.at(2) = 25;
    laser_map.at(81) = 26;
    laser_map.at(48) = 27;
    laser_map.at(71) = 28;
    laser_map.at(38) = 29;
    laser_map.at(117) = 30;
    laser_map.at(20) = 31;
    laser_map.at(43) = 32;
    laser_map.at(122) = 33;
    laser_map.at(25) = 34;
    laser_map.at(104) = 35;
    laser_map.at(15) = 36;
    laser_map.at(94) = 37;
    laser_map.at(61) = 38;
    laser_map.at(76) = 39;
    laser_map.at(35) = 40;
    laser_map.at(114) = 41;
    laser_map.at(17) = 42;
    laser_map.at(96) = 43;
    laser_map.at(7) = 44;
    laser_map.at(86) = 45;
    laser_map.at(53) = 46;
    laser_map.at(68) = 47;
    laser_map.at(91) = 48;
    laser_map.at(58) = 49;
    laser_map.at(73) = 50;
    laser_map.at(40) = 51;
    laser_map.at(127) = 52;
    laser_map.at(30) = 53;
    laser_map.at(109) = 54;
    laser_map.at(12) = 55;
    laser_map.at(83) = 56;
    laser_map.at(50) = 57;
    laser_map.at(65) = 58;
    laser_map.at(32) = 59;
    laser_map.at(119) = 60;
    laser_map.at(22) = 61;
    laser_map.at(101) = 62;
    laser_map.at(4) = 63;
    laser_map.at(27) = 64;
    laser_map.at(106) = 65;
    laser_map.at(9) = 66;
    laser_map.at(88) = 67;
    laser_map.at(63) = 68;
    laser_map.at(78) = 69;
    laser_map.at(45) = 70;
    laser_map.at(124) = 71;
    laser_map.at(19) = 72;
    laser_map.at(98) = 73;
    laser_map.at(1) = 74;
    laser_map.at(80) = 75;
    laser_map.at(55) = 76;
    laser_map.at(70) = 77;
    laser_map.at(37) = 78;
    laser_map.at(116) = 79;
    laser_map.at(75) = 80;
    laser_map.at(42) = 81;
    laser_map.at(121) = 82;
    laser_map.at(24) = 83;
    laser_map.at(111) = 84;
    laser_map.at(14) = 85;
    laser_map.at(93) = 86;
    laser_map.at(60) = 87;
    laser_map.at(67) = 88;
    laser_map.at(34) = 89;
    laser_map.at(113) = 90;
    laser_map.at(16) = 91;
    laser_map.at(103) = 92;
    laser_map.at(6) = 93;
    laser_map.at(85) = 94;
    laser_map.at(52) = 95;
    laser_map.at(11) = 96;
    laser_map.at(90) = 97;
    laser_map.at(57) = 98;
    laser_map.at(72) = 99;
    laser_map.at(47) = 100;
    laser_map.at(126) = 101;
    laser_map.at(29) = 102;
    laser_map.at(108) = 103;
    laser_map.at(3) = 104;
    laser_map.at(82) = 105;
    laser_map.at(49) = 106;
    laser_map.at(64) = 107;
    laser_map.at(39) = 108;
    laser_map.at(118) = 109;
    laser_map.at(21) = 110;
    laser_map.at(100) = 111;
    laser_map.at(123) = 112;
    laser_map.at(26) = 113;
    laser_map.at(105) = 114;
    laser_map.at(8) = 115;
    laser_map.at(95) = 116;
    laser_map.at(62) = 117;
    laser_map.at(77) = 118;
    laser_map.at(44) = 119;
    laser_map.at(115) = 120;
    laser_map.at(18) = 121;
    laser_map.at(97) = 122;
    laser_map.at(0) = 123;
    laser_map.at(87) = 124;
    laser_map.at(54) = 125;
    laser_map.at(69) = 126;
    laser_map.at(36) = 127;

//    // Initialize OpenCV windows
//    cv::namedWindow("DEPTH_IMG", 1);
//    cv::namedWindow("INTENSITY IMG", 1);
//    cv::namedWindow("VALID_IMG", 1);
  }

  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);

    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 performs a centered double to int conversion
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {

    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
      {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
      }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " <<
          config_.calibrationFile);
      return -1;
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }

    for (uint8_t i = 0; i < 16; i++) {
      vls_128_laser_azimuth_cache[i] = (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }

   return 0;
  }

  double RawData::absGpsTime2Week(double time) {
      double const SEC_IN_HOUR      = 3600;
      double const SEC_IN_DAY       = 24 * SEC_IN_HOUR;
      double const SEC_IN_WEEK      = 7 * SEC_IN_DAY;
      int gps_week = floor(time / SEC_IN_WEEK);
      return (time - gps_week * SEC_IN_WEEK);
  }


  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

      config_.max_range = max_range_;
      config_.min_range = min_range_;
      ROS_INFO_STREAM("data ranges to publish: ["
	      << config_.min_range << ", "
	      << config_.max_range << "]");

      config_.calibrationFile = calibration_file;

      ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

      calibration_.read(config_.calibrationFile);
      if (!calibration_.initialized) {
        ROS_ERROR_STREAM("Unable to open calibration file: " <<
            config_.calibrationFile);
        return -1;
      }

      // Set up cached values for sin and cos of all the possible headings
      for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
      }

      for (uint8_t i = 0; i < 16; i++) {
        vls_128_laser_azimuth_cache[i] = (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
      }

      return 0;
  }

  cv::Mat RawData::getIntensityImg(){
      return intensity_img;
  }

    cv::Mat RawData::getDepthImg(){
    return depth_img;
    }

    cv::Mat RawData::getValidImg(){
        return valid_img;
    }

    cv::Mat RawData::getFireImg(){
        return fire_img;
    }

    double RawData::getImgTime(){
      return img_timestamp;
  }

    void RawData::resetIntensityImg(){
        intensity_img = cv::Scalar(0);
    }

    void RawData::resetDepthImg(){
        depth_img = cv::Scalar(0.0);
    }

    void RawData::resetValidImg(){
        valid_img = cv::Scalar(0);
    }

    void RawData::resetFireImg(){
        fire_img = cv::Scalar(0);
    }

    void RawData::resetFireId(){
        std::fill(fire_id.begin(), fire_id.end(), 0);
    }

    int RawData::getCount() {
      return count;
  }


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                       VPointCloud &pc)
  {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
    // std::cerr << "Sensor ID = " << (unsigned int)(pkt.data[1205]) << std::endl;
    if (pkt.data[1205] == 34 || pkt.data[1205] == 36) { // VLP-16 or hi-res
      unpack_vlp16(pkt, pc);
    }
    else if (pkt.data[1205] == 40) { // VLP-32C
      unpack_vlp32(pkt, pc);
    }
    else if (pkt.data[1205] == 33) { // HDL-32E (NOT TESTED YET)
      unpack_hdl32(pkt, pc);
    }
    else if (pkt.data[1205] == 161 || pkt.data[1205] == 99) { // VLS 128
      unpack_vls128(pkt, pc);
    }
    else { // HDL-64E without azimuth compensation from the firing order
      unpack_hdl64(pkt, pc);
    }
  }

  /** @brief apply fixed correction from the file to each point and convert it to xyzi
             input : chan_id, azimuth_uint, distance, intensity
             output : x_coord, y_coord, z_coord, intensity
   */
  void RawData::compute_xyzi( const uint8_t chan_id
                            , const uint16_t azimuth_uint
                            , const float distance
                            , float &intensity
                            , float &x_coord
                            , float &y_coord
                            , float &z_coord
                            )
  {
    float x, y, z;
    velodyne_pointcloud::LaserCorrection &corrections =
      calibration_.laser_corrections[chan_id];

    // convert polar coordinates to Euclidean XYZ
    float cos_vert_angle = corrections.cos_vert_correction;
    float sin_vert_angle = corrections.sin_vert_correction;
    float cos_rot_correction = corrections.cos_rot_correction;
    float sin_rot_correction = corrections.sin_rot_correction;

    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    float cos_rot_angle =
      cos_rot_table_[azimuth_uint] * cos_rot_correction +
      sin_rot_table_[azimuth_uint] * sin_rot_correction;
    float sin_rot_angle =
      sin_rot_table_[azimuth_uint] * cos_rot_correction -
      cos_rot_table_[azimuth_uint] * sin_rot_correction;

    float horiz_offset = corrections.horiz_offset_correction;
    float vert_offset = corrections.vert_offset_correction;

    // Compute the distance in the xy plane (w/o accounting for rotation)
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

    // Calculate temporal X, use absolute value.
    float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    // Calculate temporal Y, use absolute value
    float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    if (xx < 0) xx=-xx;
    if (yy < 0) yy=-yy;

    // Get 2points calibration values,Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    float distance_corr_x = 0;
    float distance_corr_y = 0;
    if (corrections.two_pt_correction_available) {
      distance_corr_x =
	(corrections.dist_correction - corrections.dist_correction_x)
	  * (xx - 2.4) / (25.04 - 2.4)
	+ corrections.dist_correction_x;
      distance_corr_x -= corrections.dist_correction;
      distance_corr_y =
	(corrections.dist_correction - corrections.dist_correction_y)
	  * (yy - 1.93) / (25.04 - 1.93)
	+ corrections.dist_correction_y;
      distance_corr_y -= corrections.dist_correction;
    }

    float distance_x = distance + distance_corr_x;
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
    x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

    float distance_y = distance + distance_corr_y;
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
    y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

    // Using distance_y is not symmetric, but the velodyne manual
    // does this.
    /**the new term of 'vert_offset * cos_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

    /** Use standard ROS coordinate system (right-hand rule) */
    x_coord = y;
    y_coord = -x;
    z_coord = z;

    /** Intensity Calculation */
    float min_intensity = corrections.min_intensity;
    float max_intensity = corrections.max_intensity;

    float focal_offset = 256
		       * (1 - corrections.focal_distance / 13100)
		       * (1 - corrections.focal_distance / 13100);
    float focal_slope = corrections.focal_slope;
    intensity += focal_slope * (abs(focal_offset - 256 *
      (1 - static_cast<float>(azimuth_uint)/65535)*(1 - static_cast<float>(azimuth_uint)/65535)));
    intensity = (intensity < min_intensity) ? min_intensity : intensity;
    intensity = (intensity > max_intensity) ? max_intensity : intensity;
  }


  /** @brief convert raw HDL-64E channel packet to point cloud
   *         a default one without any time-domain azimuth correction
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_hdl64(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    for (int i = 0; i < NUM_BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }
      // Azimuth extraction
      uint16_t azimuth = raw->blocks[i].rotation;
      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        azimuth >= config_.min_angle && azimuth <= config_.max_angle)
        || (config_.min_angle > config_.max_angle)) {

	for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k += CHANNEL_SIZE) {
          uint8_t laser_number = j + bank_origin;
	  velodyne_pointcloud::LaserCorrection &corrections =
	    calibration_.laser_corrections[laser_number];

	  // Distance extraction
	  union two_bytes tmp;
	  tmp.bytes[0] = raw->blocks[i].data[k];
	  tmp.bytes[1] = raw->blocks[i].data[k+1];
	  float distance = tmp.uint * DISTANCE_RESOLUTION;
	  distance += corrections.dist_correction;

	  if (pointInRange(distance)) {
            // Intensity extraction
	    float intensity = raw->blocks[i].data[k+2];
	    float x_coord, y_coord, z_coord;

	    // apply calibration file and convert polar coordinates to Euclidean XYZ
	    compute_xyzi(laser_number, azimuth, distance, intensity, x_coord, y_coord, z_coord);

            // append this point to the cloud
	    VPoint point;
	    point.ring = corrections.laser_ring;
	    point.x = x_coord;
	    point.y = y_coord;
	    point.z = z_coord;
	    point.intensity = intensity;
	    pc.points.push_back(point);
	    ++pc.width;
	  }
        }
      }
    }
  }

  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    float azimuth_diff, azimuth_corrected_f;
    float last_azimuth_diff = 0;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance, intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    bool dual_return = (pkt.data[1204] == 57);
    for (int block = 0; block < NUM_BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }
      // Azimuth extraction
      if (block == 0) {
        azimuth = raw->blocks[block].rotation;
      } else {
        azimuth = azimuth_next;
      }
      // Calculate difference between current and next block's azimuth angle.
      if (block < (NUM_BLOCKS_PER_PACKET-(1 + dual_return))){
        azimuth_next = raw->blocks[block+(1+dual_return)].rotation; // correct for dual return
        azimuth_diff = (float)((36000 + azimuth_next - azimuth)%36000);
        last_azimuth_diff = azimuth_diff;
      } else {
        azimuth_diff = (block == NUM_BLOCKS_PER_PACKET-1) ? 0 : last_azimuth_diff;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        azimuth >= config_.min_angle && azimuth <= config_.max_angle)
        || (config_.min_angle > config_.max_angle)) {

	for (int seq_id = 0, k = 0; seq_id < VLP16_NUM_SEQS_PER_BLOCK; seq_id++){
	  for (int chan_id = 0; chan_id < VLP16_NUM_CHANS_PER_SEQ; chan_id++, k+=CHANNEL_SIZE){
	    velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[chan_id];

	    // Distance extraction
	    union two_bytes tmp;
	    tmp.bytes[0] = raw->blocks[block].data[k];
	    tmp.bytes[1] = raw->blocks[block].data[k+1];
	    distance = (float)tmp.uint * DISTANCE_RESOLUTION;
	    distance += corrections.dist_correction;

	    if (pointInRange(distance)) {
	      intensity = (float)raw->blocks[block].data[k+2];

	      // azimuth correction from the firing order in time-domain
	      azimuth_corrected_f = azimuth + (azimuth_diff * ((chan_id*CHANNEL_TDURATION) + (seq_id*SEQ_TDURATION)) / VLP16_BLOCK_TDURATION);
	      azimuth_corrected = ((uint16_t)round(azimuth_corrected_f)) % 36000;

	      // apply calibration file and convert polar coordinates to Euclidean XYZ
	      compute_xyzi(chan_id, azimuth_corrected, distance, intensity, x_coord, y_coord, z_coord);

	      // append this point to the cloud
	      VPoint point;
	      point.ring = corrections.laser_ring;
	      point.x = x_coord;
	      point.y = y_coord;
	      point.z = z_coord;
	      point.intensity = intensity;

	      pc.points.push_back(point);
	      ++pc.width;
	    }
	  }
	}
      }
    }
  }

#if 0
  void RawData::unpack_vlp32(const velodyne_msgs::VelodynePacket &pkt,
                             velodyne_rawdata::XYZIRBPointCloud &pc)
  {
    float azimuth_diff, azimuth_corrected_f;
    float last_azimuth_diff = 0;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance, intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block = 0; block < NUM_BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-32 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      if (block == 0) {
        azimuth = raw->blocks[block].rotation;
      } else {
        azimuth = azimuth_next;
      }
      if (block < (NUM_BLOCKS_PER_PACKET-1)){
        azimuth_next = raw->blocks[block+1].rotation;
        azimuth_diff = (float)((36000 + azimuth_next - azimuth)%36000);
        last_azimuth_diff = azimuth_diff;
      } else {
        azimuth_diff = last_azimuth_diff;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        azimuth >= config_.min_angle && azimuth <= config_.max_angle)
        || (config_.min_angle > config_.max_angle)) {

	for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k+=CHANNEL_SIZE){
	  uint8_t chan_id = j;
	  uint8_t firing_order = chan_id/2;
	  velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[chan_id];

	  // distance extraction
	  union two_bytes tmp;
	  tmp.bytes[0] = raw->blocks[block].data[k];
	  tmp.bytes[1] = raw->blocks[block].data[k+1];
	  distance = (float)tmp.uint * VLP32_DISTANCE_RESOLUTION;
	  distance += corrections.dist_correction;

	  if (pointInRange(distance)) {
	    intensity = (float)raw->blocks[block].data[k+2];

	    /** correct for the laser rotation as a function of timing during the firings **/
	    azimuth_corrected_f = azimuth + (azimuth_diff * (firing_order*CHANNEL_TDURATION) / SEQ_TDURATION);
	    azimuth_corrected = ((uint16_t)round(azimuth_corrected_f)) % 36000;

	    // apply calibration file and convert polar coordinates to Euclidean XYZ
	    compute_xyzi(chan_id, azimuth_corrected, distance, intensity, x_coord, y_coord, z_coord);

	    // append this point to the cloud
	    PointXYZIRB point;
	    point.ring = corrections.laser_ring;
	    point.x = x_coord;
	    point.y = y_coord;
	    point.z = z_coord;
	    point.intensity = intensity;
	    point.block = block;
	    pc.points.push_back(point);
	    ++pc.width;
	  }
        }
      }
    }
  }

#endif

  /** @brief convert raw VLP32 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp32(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    float azimuth_diff, azimuth_corrected_f;
    float last_azimuth_diff = 0;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance, intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    bool dual_return = (pkt.data[1204] == 57);

    for (int block = 0; block < NUM_BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-32 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      if (block == 0) {
        azimuth = raw->blocks[block].rotation;
      } else {
        azimuth = azimuth_next;
      }
      if (block < (NUM_BLOCKS_PER_PACKET-(1+dual_return))){
        azimuth_next = raw->blocks[block+(1+dual_return)].rotation;
        azimuth_diff = (float)((36000 + azimuth_next - azimuth)%36000);
        last_azimuth_diff = azimuth_diff;
      } else {
        azimuth_diff = (block == NUM_BLOCKS_PER_PACKET-1) ? 0 : last_azimuth_diff;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        azimuth >= config_.min_angle && azimuth <= config_.max_angle)
        || (config_.min_angle > config_.max_angle)) {

	for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k+=CHANNEL_SIZE){
	  uint8_t chan_id = j;
	  uint8_t firing_order = chan_id/2;
	  velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[chan_id];

	  // distance extraction
	  union two_bytes tmp;
	  tmp.bytes[0] = raw->blocks[block].data[k];
	  tmp.bytes[1] = raw->blocks[block].data[k+1];
	  distance = (float)tmp.uint * VLP32_DISTANCE_RESOLUTION;
	  distance += corrections.dist_correction;

	  if (pointInRange(distance)) {
	    intensity = (float)raw->blocks[block].data[k+2];

	    /** correct for the laser rotation as a function of timing during the firings **/
	    azimuth_corrected_f = azimuth + (azimuth_diff * (firing_order*CHANNEL_TDURATION) / SEQ_TDURATION);
	    azimuth_corrected = ((uint16_t)round(azimuth_corrected_f)) % 36000;

	    // apply calibration file and convert polar coordinates to Euclidean XYZ
	    compute_xyzi(chan_id, azimuth_corrected, distance, intensity, x_coord, y_coord, z_coord);

	    // append this point to the cloud
	    VPoint point;
	    point.ring = corrections.laser_ring;
	    point.x = x_coord;
	    point.y = y_coord;
	    point.z = z_coord;
	    point.intensity = intensity;

	    pc.points.push_back(point);
	    ++pc.width;
	  }
        }
      }
    }
  }

  /** @brief convert raw VLS128 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc) {
//    int64 t0 = cv::getTickCount();
    float azimuth_diff, azimuth_corrected_f;
    float last_azimuth_diff = 0;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance;
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    union two_bytes tmp;

    float cos_vert_angle, sin_vert_angle, cos_rot_correction, sin_rot_correction;
    float cos_rot_angle, sin_rot_angle;
    float xy_distance;

    uint8_t laser_number, firing_order;
    bool dual_return = (pkt.data[1204] == 57);

    // Get NMEA timing info    
    double gpsTimeSec = pkt.stamp.toSec();
    int gps_cycle = 0;
    double gpsTimeSecPastCycle = gpsTimeSec;
    int gps_week = 0;
    double gpsTimeSecPastWeek;
    double gps_hour = 0;
    double gpsTimeSecPastHour;

    // Convert gpsTimeSec from GPS time to UTC time
    // TODO no good conversion tool is available, so now just use hard-coded 18.0 seconds.
    gpsTimeSec -= 18.0;

    bool b_nmea_time_available = true;
    if(gpsTimeSec == 0){       // no time sync info      
      b_nmea_time_available = false;    
    }else{      
      gps_cycle = floor(gpsTimeSec / SEC_IN_GPS_CYCLE);
      gpsTimeSecPastCycle = gpsTimeSec - gps_cycle * SEC_IN_GPS_CYCLE;
      gps_week = floor(gpsTimeSecPastCycle / SEC_IN_WEEK);
      gpsTimeSecPastWeek = gpsTimeSecPastCycle - gps_week * SEC_IN_WEEK;
    }
    gps_hour = floor(gpsTimeSecPastWeek / SEC_IN_HOUR);
    gpsTimeSecPastHour = gpsTimeSecPastWeek - gps_hour * SEC_IN_HOUR;
    // double nmea_gps_sec_past_hour = fmod( gpsTimeSec, SEC_IN_HOUR );    

    // Get packet (Velodyne internal) timing info
    uint8_t* timestamp_raw = (uint8_t*)&(raw->timestamp);
    uint* timestamp_packet_microsec = (uint*)timestamp_raw;
    double packet_time_sec_past_hour = (double)(*timestamp_packet_microsec) / 1000000; 

    // witin a 2 sec of the hour mark, do no update gps hour using nmea message
    // Instead, rely on the more precise and higher rate timetamps from the Velodyne packets
    if( b_nmea_time_available ){

      if( gpsTimeSecPastHour > 2.0 && gpsTimeSecPastHour < 3598.0) {        
        // outside +/- 2 sec of the hour mark
        if (gps_h_past_week_ != gps_hour ){
          ROS_INFO("Internal GPS hour past week (%f) updating to NMEA hour past week (%f).",
                    gps_h_past_week_, gps_hour);
          ROS_INFO("Packet sec past hour: %f", packet_time_sec_past_hour);
          ROS_INFO("NMEA sec past hour:     %f", gpsTimeSecPastHour);
          ROS_INFO("NMEA sec past week:     %f", gpsTimeSecPastWeek);
        }
        gps_h_past_week_ = gps_hour;
        b_internal_gps_h_updated_to_nmea_h_ = true;      
        
        }else{        
          b_internal_gps_h_updated_to_nmea_h_ = false;
        }
    }    
    if(!b_internal_gps_h_updated_to_nmea_h_ && packet_time_sec_past_hour < 10){
     // std::cout << "*** " << packet_time_sec_past_hour << std::endl;
     packet_time_sec_past_hour += SEC_IN_HOUR;
   //}else{
     //std::cout << nmea_gps_sec_past_hour << " " << packet_time_sec_past_hour << std::endl;
   }
   //if( !b_internal=_gps_h_updated_to_nmea_h_ && packet_time_sec_past_hour < 10 ){
   //    packet_time_sec_past_hour += SEC_IN_HOUR;
  // }
   //std::cout << gps_h_past_week_  << " " << std::setprecision(10) << gpsTimeSec << " "
   //          << packet_time_sec_past_hour << std::endl;    
   double abs_gps_hour_in_sec = gps_h_past_week_ * SEC_IN_HOUR
                                + gps_week * SEC_IN_WEEK
                                + gps_cycle * SEC_IN_GPS_CYCLE;


    // uint8_t* timestamp_raw = (uint8_t*)&(raw->timestamp);
    // uint* timestamp_packet_microsec = (uint*)timestamp_raw;
    // double packet_time_sec_past_hour = (double)(*timestamp_packet_microsec) / 1000000;
    // double timeINS = gpsTimeBeforeHourInSec + packet_time_sec_past_hour;
    double timeINS = packet_time_sec_past_hour + abs_gps_hour_in_sec;

    // Convert gpsTimeSec from UTC time to GPS time
    // TODO no good conversion tool is available, so now just use hard-coded 18.0 seconds.
    timeINS += 18.0;

    //Calculate weekTime in advance
    double weekTime = absGpsTime2Week(timeINS);

//    int64 t1 = cv::getTickCount();

    for (int block = 0; block < NUM_BLOCKS_PER_PACKET - (4* dual_return); block++) {
      // cache block for use
      const raw_block_t &current_block = raw->blocks[block];

      int bank_origin = 0;
      // Used to detect which bank of 32 lasers is in this block
      switch (current_block.header) {
        case VLS128_BANK_1:
          bank_origin = 0;
          break;
        case VLS128_BANK_2:
          bank_origin = 32;
          break;
        case VLS128_BANK_3:
          bank_origin = 64;
          break;
        case VLS128_BANK_4:
          bank_origin = 96;
          break;
        default:
          // ignore packets with mangled or otherwise different contents
          // Do not flood the log with messages, only issue at most one
          // of these warnings per minute.
          ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLS-128 packet: block "
                    << block << " header value is "
                    << raw->blocks[block].header);
          return; // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      if (block == 0) {
        azimuth = current_block.rotation;
      } else {
        azimuth = azimuth_next;
      }
      if (block < (NUM_BLOCKS_PER_PACKET - (1+dual_return))) {
        // Get the next block rotation to calculate how far we rotate between blocks
        azimuth_next = raw->blocks[block + (1+dual_return)].rotation;

        // Finds the difference between two sucessive blocks
        azimuth_diff = (float)((36000 + azimuth_next - azimuth) % 36000);

        // This is used when the last block is next to predict rotation amount
        last_azimuth_diff = azimuth_diff;
      } else {
        // This makes the assumption the difference between the last block and the next packet is the
        // same as the last to the second to last.
        // Assumes RPM doesn't change much between blocks
        azimuth_diff = (block == NUM_BLOCKS_PER_PACKET - (4*dual_return)-1) ? 0 : last_azimuth_diff;
      }

      // condition added to avoid calculating points which are not in the interesting defined area (min_angle < area < max_angle)
      if ((config_.min_angle < config_.max_angle && azimuth >= config_.min_angle && azimuth <= config_.max_angle) || (config_.min_angle > config_.max_angle)) {
        for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k += CHANNEL_SIZE) {
          // distance extraction
          tmp.bytes[0] = current_block.data[k];
          tmp.bytes[1] = current_block.data[k + 1];
          distance = tmp.uint * VLP32_DISTANCE_RESOLUTION;
          bool no_return = false;
          if (distance == 0) {
            no_return = true;
            // distance  = 2;
          }
          // distance = 3;

          if (pointInRange(distance)) {
            laser_number = j + bank_origin;   // Offset the laser in this block by which block it's in
            firing_order = laser_number / 8;  // VLS-128 fires 8 lasers at a time

            velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

            // correct for the laser rotation as a function of timing during the firings
            azimuth_corrected_f = azimuth + (azimuth_diff * vls_128_laser_azimuth_cache[firing_order]);
            azimuth_corrected = ((uint16_t) round(azimuth_corrected_f)) % 36000;

            // convert polar coordinates to Euclidean XYZ
            cos_vert_angle = corrections.cos_vert_correction;
            sin_vert_angle = corrections.sin_vert_correction;
            cos_rot_correction = corrections.cos_rot_correction;
            sin_rot_correction = corrections.sin_rot_correction;

            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            cos_rot_angle =
              cos_rot_table_[azimuth_corrected] * cos_rot_correction +
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            sin_rot_angle =
              sin_rot_table_[azimuth_corrected] * cos_rot_correction -
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;

            // Compute the distance in the xy plane (w/o accounting for rotation)
            xy_distance = distance * cos_vert_angle;

            /** Use standard ROS coordinate system (right-hand rule) */
            // append this point to the cloud
            VPoint point;
            point.ring = corrections.laser_ring;
            point.x = xy_distance * cos_rot_angle;    // velodyne y
            point.y = -(xy_distance * sin_rot_angle); // velodyne x
            point.z = distance * sin_vert_angle;      // velodyne z

            // Intensity extraction
            point.intensity = current_block.data[k + 2];

            // time
            point.timeINS = timeINS;

            if (no_return)
              point.intensity = 0;

            pc.points.push_back(point);
            ++pc.width;

            // I am only interested in intensity image so far
            int element_id = fire_id.at(laser_map.at(laser_number));
            fire_img.at<uchar>(laser_map.at(laser_number), element_id) = point.intensity;
            fire_id.at(laser_map.at(laser_number)) = element_id + 1;

            // Update intensity/depth/valid images
            int nRows = intensity_img.rows;
            int nCols = intensity_img.cols;

            // calculate mapping between point clouds 3d position and 2d image
            float azimuth_rad = atan2(cos_rot_angle, -sin_rot_angle);
            float elevation_rad = atan2(sin_vert_angle, cos_vert_angle);

            // Do not proceed if point falls outside interested region
            if (azimuth_rad <= M_PI  && azimuth_rad >= -M_PI) {
//            if (true) {
                int c = int(floor(azimuth_rad / azi_res_rad));
                int r = int(floor(elevation_rad / elev_res_rad));

                // assign pixel values to 2d intensity image
                // if assigned before, take the max
                // width / 2 = 900 and 10 + 150 (buffer + 15 elevation) = 160 - 1 is the center
                int cc = c < nCols / 2 ? nCols / 2 + c : c - nCols / 2;
                int rc = int((15 + 0.5) / 180 * M_PI / elev_res_rad) - 1 - r;

                // Update intensity image
                if (intensity_img.at<uchar>(rc, cc) == 0) {
                    intensity_img.at<uchar>(rc, cc) = int(point.intensity);
                } else {
                    if (intensity_img.at<uchar>(rc, cc) < int(point.intensity)) {
                        intensity_img.at<uchar>(rc, cc) = int(point.intensity);
                    }
                }

                // Update depth image
                if (distance == 0) {
                    depth_img.at<float>(rc, cc) = 0;
                } else {
                    depth_img.at<float>(rc, cc) = distance;
                }

                // Update valid image
                if (distance == 0) {
                    valid_img.at<uchar>(rc, cc) = 0;
                } else {
                    valid_img.at<uchar>(rc, cc) = 255;
                }

                // Update img_timestamp
                img_timestamp = weekTime;
            }
          }
          else{
              // I am only interested in intensity image so far
              laser_number = j + bank_origin;
              int element_id = fire_id.at(laser_map.at(laser_number));
              fire_img.at<uchar>(laser_map.at(laser_number), element_id) = 0;
              fire_id.at(laser_map.at(laser_number)) = element_id + 1;

              // Update img_timestamp
              img_timestamp = weekTime;
          }

        }
      }
    }

//    int64 t2 = cv::getTickCount();
//    double interval01 = (t1-t0)/cv::getTickFrequency();
//    double interval12 = (t2-t1)/cv::getTickFrequency();
//
//    ROS_INFO("time intervals 0-1: %f", interval01);
//    ROS_INFO("time intervals 1-2: %f", interval12);

//      // Show images
//      cv::imshow("INTENSITY_IMG", intensity_img);
//      cv::waitKey(0);
  }

  /** @brief convert raw HDL-32E channel packet to point cloud
   *         a default one without any time-domain azimuth correction
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_hdl32(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    float azimuth_diff, azimuth_corrected_f;
    float last_azimuth_diff = 0;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance, intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block = 0; block < NUM_BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-32 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      if (block == 0) {
        azimuth = raw->blocks[block].rotation;
      } else {
        azimuth = azimuth_next;
      }
      if (block < (NUM_BLOCKS_PER_PACKET-1)){
        azimuth_next = raw->blocks[block+1].rotation;
        azimuth_diff = (float)((36000 + azimuth_next - azimuth)%36000);
        last_azimuth_diff = azimuth_diff;
      } else {
        azimuth_diff = last_azimuth_diff;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        azimuth >= config_.min_angle && azimuth <= config_.max_angle)
        || (config_.min_angle > config_.max_angle)) {

	for (int j = 0, k = 0; j < NUM_CHANS_PER_BLOCK; j++, k+=CHANNEL_SIZE){
	  uint8_t chan_id = j;
	  velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[chan_id];

	  // distance extraction
	  union two_bytes tmp;
	  tmp.bytes[0] = raw->blocks[block].data[k];
	  tmp.bytes[1] = raw->blocks[block].data[k+1];
	  distance = (float)tmp.uint * DISTANCE_RESOLUTION;
	  distance += corrections.dist_correction;

	  if (pointInRange(distance)) {
	    intensity = (float)raw->blocks[block].data[k+2];

	    /** correct for the laser rotation as a function of timing during the firings **/
	    azimuth_corrected_f = azimuth + (azimuth_diff * (chan_id*HDL32_CHANNEL_TDURATION) / HDL32_SEQ_TDURATION);
	    azimuth_corrected = ((uint16_t)round(azimuth_corrected_f)) % 36000;

	    // apply calibration file and convert polar coordinates to Euclidean XYZ
	    compute_xyzi(chan_id, azimuth_corrected, distance, intensity, x_coord, y_coord, z_coord);

	    // append this point to the cloud
	    VPoint point;
	    point.ring = corrections.laser_ring;
	    point.x = x_coord;
	    point.y = y_coord;
	    point.z = z_coord;
	    point.intensity = intensity;

	    pc.points.push_back(point);
	    ++pc.width;
	  }
        }
      }
    }
  }

} // namespace velodyne_rawdata
