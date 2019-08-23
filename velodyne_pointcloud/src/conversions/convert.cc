/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);
    image_transport::ImageTransport it(node);
    intensity_pub = it.advertise("camera/intensity_image", 1000);
    depth_pub = it.advertise("camera/depth_image", 1000);
    valid_pub = it.advertise("camera/valid_image", 1000);
    fire_pub = it.advertise("camera/fire_image", 1000);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
//    if (output_.getNumSubscribers() == 0)         // no one listening?
//      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
    outMsg(new velodyne_rawdata::VPointCloud());
   //   velodyne_rawdata::XYZIRBPointCloud::Ptr
   //   outMsg(new velodyne_rawdata::XYZIRBPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
//    ROS_INFO("packet size: %d", scanMsg->packets.size());
    // Reset all images to zeros
    data_->resetIntensityImg();
    data_->resetDepthImg();
    data_->resetValidImg();
    data_->resetFireImg();
    data_->resetFireId();
//    int64 t0 = cv::getTickCount();
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *outMsg);
      }
    sensor_msgs::ImagePtr fireMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", data_->getFireImg()).toImageMsg();
    fire_pub.publish(fireMsg);
    sensor_msgs::ImagePtr intensityMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", data_->getIntensityImg()).toImageMsg();
    intensity_pub.publish(intensityMsg);

    // For depth images, need to convert float to mono8 first
    cv::Mat dm = data_->getDepthImg();
    cv::Mat dm_h;
    dm.convertTo(dm_h, CV_8U);
    sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", dm_h).toImageMsg();
    depth_pub.publish(depthMsg);
    sensor_msgs::ImagePtr validMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", data_->getValidImg()).toImageMsg();
    valid_pub.publish(validMsg);

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);

//    int64 t1 = cv::getTickCount();
//    double interval01 = (t1-t0)/cv::getTickFrequency();
//    ROS_INFO("time intervals 0-1: %f", interval01);
  }

} // namespace velodyne_pointcloud
