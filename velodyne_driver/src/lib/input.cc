/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <velodyne_driver/input.h>

namespace velodyne_driver
{
  static const size_t packet_size =
    sizeof(velodyne_msgs::VelodynePacket().data);
  static const size_t ppacket_size =
    sizeof(velodyne_msgs::VelodynePosPacket().data);

  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Input::Input(ros::NodeHandle private_nh, uint16_t dport, uint16_t pport):
    private_nh_(private_nh),
    dport_(dport),
    pport_(pport)
  {
    private_nh.param("device_ip", devip_str_, std::string(""));
    if (!devip_str_.empty())
      ROS_INFO_STREAM("Only accepting packets from IP address: "
                      << devip_str_);
  }

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t dport, uint16_t pport):
    Input(private_nh, dport, pport)
  {
    sockfd_d_ = -1;
    
    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to Velodyne UDP port for data packets
    ROS_INFO_STREAM("Opening UDP socket for data packets: port " << dport);
    sockfd_d_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_d_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(dport);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
  
    if (bind(sockfd_d_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }
  
    if (fcntl(sockfd_d_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }

    ROS_DEBUG("Velodyne socket fd for data packet is %d\n", sockfd_d_);

    // connect to Velodyne UDP port for position packets for VLP devices
    sockfd_p_ = -1;
    if( pport != 0 ){

      ROS_INFO_STREAM("Opening UDP socket for position packets: port " << pport);
      sockfd_p_ = socket(PF_INET, SOCK_DGRAM, 0);
      if (sockfd_p_ == -1){
        perror("socket");               
        return;
      }
    
      // resuing my_addr;                     
      memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
      my_addr.sin_family = AF_INET;            // host byte order
      my_addr.sin_port = htons(pport);         // port in network byte order
      my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
    
      if (bind(sockfd_p_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
        {
          perror("bind");                
          return;
        }
    
      if (fcntl(sockfd_p_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
        {
          perror("non-block");
          return;
        }

      ROS_DEBUG("Velodyne socket fd for position packet is %d\n", sockfd_p_);
    }
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_d_);
    (void) close(sockfd_p_);
  }

  void InputSocket::setPacketRate ( const double packet_rate)
  { 
      return;
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getPacket(velodyne_msgs::VelodynePacket *pkt, 
      velodyne_msgs::VelodynePosPacketPtr &ppkt,
      const double time_offset)
  {
    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = sockfd_d_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
      {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do
          {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
              {
                if (errno != EINTR)
                  ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
              }
            if (retval == 0)            // poll() timeout?
              {
                ROS_WARN("Velodyne poll() timeout");
                return 1;
              }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) // device error?
              {
                ROS_ERROR("poll() reports Velodyne error");
                return 1;
              }
          } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(sockfd_d_, &pkt->data[0],
                                  packet_size,  0,
                                  (sockaddr*) &sender_address,
                                  &sender_address_len);

        if (nbytes < 0)
          {
            if (errno != EWOULDBLOCK)
              {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
              }
          }
        else if ((size_t) nbytes == packet_size)
          {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if(devip_str_ != ""
               && sender_address.sin_addr.s_addr != devip_.s_addr)
              continue;
            else
              break; //done
          }

        ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                         << nbytes << " bytes");
      }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred. Add the time offset.
    double time2 = ros::Time::now().toSec();
    pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);

    // sockfd_p_ should only be not equal -1 when device is VLP
    if( sockfd_p_ != -1){

      // Poll to see if a position packet is available
      // From example PCAP files supplied by Velodyne, position packets don't appear to be sent at a fixed rate
      // but they appear in between every few data packets
      ppkt.reset(new velodyne_msgs::VelodynePosPacket );
      uint nPosPackets = 0;
      velodyne_msgs::VelodynePosPacket ppkt_tmp;
      struct pollfd pos_poll_fds;
      pos_poll_fds.fd = sockfd_p_;  
      pos_poll_fds.events = POLLIN; // Alert when data is ready to recv() on this socket.
      do{
        int pos_poll_retval = poll(&pos_poll_fds, 1, 0); // timeout == 0 causes poll() to return immediately, even if no file descriptors are ready
        if (pos_poll_fds.revents & POLLIN){ // position packet available
           // read the packet and check its size
           ssize_t nbytes = recvfrom(sockfd_p_, &(ppkt_tmp.data[0]),
                                    ppacket_size,  0,
                                    (sockaddr*) &sender_address,
                                    &sender_address_len);

          // check if it's from the correct scanner
          if(devip_str_ == "" ||
             devip_str_ != "" && sender_address.sin_addr.s_addr == devip_.s_addr){
            *ppkt = ppkt_tmp;
            ppkt->stamp = ros::Time::now();
            nPosPackets++;
          }

        }

      }while (pos_poll_fds.revents & POLLIN); // keep reading if there are packets available. 
      
      if( nPosPackets == 0 ){
        ppkt.reset();
      }
    }

    return 0;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
  InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t dport, uint16_t pport,
                       double packet_rate, std::string filename,
                       bool read_once, bool read_fast, double repeat_delay):
    Input(private_nh, dport, pport),
    packet_rate_(packet_rate),
    filename_(filename)
  {
    pcap_ = NULL;  
    empty_ = true;

    // get parameters using private node handle
    private_nh.param("read_once", read_once_, false);
    private_nh.param("read_fast", read_fast_, false);
    private_nh.param("repeat_delay", repeat_delay_, 0.0);

    if (read_once_)
      ROS_INFO("Read input file only once.");
    if (read_fast_)
      ROS_INFO("Read input file as quickly as possible.");
    if (repeat_delay_ > 0.0)
      ROS_INFO("Delay %.3f seconds before repeating input file.",
               repeat_delay_);

    // Open the PCAP dump file
    ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
      {
        ROS_FATAL("Error opening Velodyne socket dump file.");
        return;
      }

    std::stringstream data_filter;
    std::stringstream position_filter;
    if( devip_str_ != "" )              // using specific IP?
      {
        data_filter << "src host " << devip_str_ << " && ";
        position_filter << "ip host " << devip_str_ << " and ";
      }

    data_filter << "udp port " << dport;
    if( pcap_compile(pcap_, &pcap_data_packet_filter_,
                     data_filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) == -1 ){
      ROS_WARN("PCAP data filter construction failed");
    }

    pcap_position_packet_filter_.bf_insns = NULL;
    if(pport != 0){
    
      position_filter << "udp port " << pport;
      if( pcap_compile(pcap_, &pcap_position_packet_filter_,
                       position_filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) == -1 ){
        ROS_WARN("PCAP position filter construction failed"); 
      }     
    }
    pwait_time = NULL;
  }

  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }

  void InputPCAP::setPacketRate ( const double packet_rate)
  { 
    //packet_rate_(packet_rate); 
    if(pwait_time != NULL) 
    {
      delete pwait_time ;
    }
    pwait_time = new ros::Duration(1.0/packet_rate);
  }
  /** @brief Get one velodyne packet. */
  // int InputPCAP::getPacket(velodyne_msgs::VelodynePacket *pkt, 
  //     velodyne_msgs::VelodynePosPacketPtr &ppkt,
  //     const double time_offset)
  // {
  //   struct pcap_pkthdr *header;
  //   const u_char *pkt_data;

  //   while (true)
  //     {
  //       int res;
  //       if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
  //         {
  //           // Skip packets not for the correct port and from the
  //           // selected IP address.
  //           if ( /* !devip_str_.empty() &&  Let the filter take care of skipping bad packets ... not dependent on device IP setting*/  
  //               (0 == pcap_offline_filter(&pcap_packet_filter_,
  //                                         header, pkt_data))) 
  //           {
  //             continue;
  //           }

  //           // Keep the reader from blowing through the file.
  //           if (read_fast_ == false)
  //           {
  //             if(pwait_time == NULL) // use initial estimated wait from configs
  //               packet_rate_.sleep();
  //             else 
  //               pwait_time->sleep();  // use auto rpm derived wait time
  //           }
            
  //           memcpy(&pkt->data[0], pkt_data+42, packet_size);
  //           pkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
  //           empty_ = false;
  //           return 0;                   // success
  //         }

  //       if (empty_)                 // no data in file?
  //         {
  //           ROS_WARN("Error %d reading Velodyne packet: %s", 
  //                    res, pcap_geterr(pcap_));
  //           return -1;
  //         }

  //       if (read_once_)
  //         {
  //           ROS_INFO("end of file reached -- done reading.");
  //           return -1;
  //         }
        
  //       if (repeat_delay_ > 0.0)
  //         {
  //           ROS_INFO("end of file reached -- delaying %.3f seconds.",
  //                    repeat_delay_);
  //           usleep(rint(repeat_delay_ * 1000000.0));
  //         }

  //       ROS_DEBUG("replaying Velodyne dump file");

  //       // I can't figure out how to rewind the file, because it
  //       // starts with some kind of header.  So, close the file
  //       // and reopen it with pcap.
  //       pcap_close(pcap_);
  //       pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
  //       empty_ = true;              // maybe the file disappeared?
  //     } // loop back and try again
  // }
    /** @brief Get one velodyne packet. */
  int InputPCAP::getPacket(velodyne_msgs::VelodynePacket *pkt, 
         velodyne_msgs::VelodynePosPacketPtr &ppkt,
         const double time_offset)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    ppkt.reset();

    while (true)
      {
        int res = pcap_next_ex(pcap_, &header, &pkt_data);
        if (res >= 0)
        {
            if ( !devip_str_.empty() && // Check if packet is a data packet
                pcap_offline_filter(&pcap_data_packet_filter_, header, pkt_data)  > 0 )
      {

              // Keep the reader from blowing through the file.
              if (read_fast_ == false)
                packet_rate_.sleep();
                            
              memcpy(&pkt->data[0], pkt_data+42, packet_size);
              pkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
              empty_ = false;
              return 0;                   // success
        

            }else if(!devip_str_.empty() && // check if packet is a position packet
                     pcap_position_packet_filter_.bf_insns != NULL &&
                     pcap_offline_filter(&pcap_position_packet_filter_, header, pkt_data) > 0 ){

                // Keep the reader from blowing through the file.
                if (read_fast_ == false)
                  packet_rate_.sleep();

                ppkt.reset(new velodyne_msgs::VelodynePosPacket );
                            
                memcpy(&ppkt->data[0], pkt_data+42, ppacket_size);
                ppkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
                empty_ = false;

                continue;                   // success

            }else{

              continue;
            }
            
        }

        if (res == -1 || empty_)                 // no data in file?
          {
            ROS_WARN("Error %d reading Velodyne packet: %s", 
                     res, pcap_geterr(pcap_));
            return -1;
          }

        else if (res == -2 && read_once_)
          {
            ROS_INFO("end of file reached -- done reading.");
            return -1;
          }
        
        else if (res == -2 && repeat_delay_ > 0.0)
          {
            ROS_INFO("end of file reached -- delaying %.3f seconds.",
                     repeat_delay_);
            usleep(rint(repeat_delay_ * 1000000.0));
          }

        ROS_DEBUG("replaying Velodyne dump file");

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;              // maybe the file disappeared?
      } // loop back and try again
  }

} // velodyne_driver namespace
