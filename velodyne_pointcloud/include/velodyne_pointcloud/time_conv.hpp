/** 
 *  \file time_conv.hpp
 *  \brief  C++ library for GPS-related time conversion
 *  \copyright Copyright 2018 Applanix Corp (Trimble Inc.). All rights reserved.
 *  \license Applanix Corp (Trimble Inc.) proprietary.
 *  \author Keith Leung
 */

#ifndef TIME_CONV_HPP
#define TIME_CONV_HPP

#include <boost/date_time/gregorian/gregorian.hpp>
#include <string>
#include <vector>

namespace conversion { 
  namespace time {
   
    uint const SECS_PER_DAY = 86400; /**< Seconds in a day */
    uint const SECS_PER_WEEK = SECS_PER_DAY * 7; /**< Seconds in a week */
    double const TAI_minus_GPS_OFFSET = 19; /**< TAI - GPS in seconds */
    double const JD_minus_MJD_OFFSET = 2400000.5; /**< JD - MJD in days */

    /**
     * \class LeapSecondsLookup
     * \brief For storing and query of leap second data
     */
    class LeapSecondsLookup
    {
    public:

      /** \brief Leap second entry */
      struct LeapSeconds{
	uint year;
	uint month;
	uint day;
	double JD; /**< Julian date corresponding to year, month, day*/
	double TAI_minus_UTC_offset;
	double TAI_minus_UTC_drift1;
	double TAI_minus_UTC_drift2;
	double MJD; /**< modified Julian date */
	double TAI_minus_UTC;
      };

      /** 
       * \brief Default constructor
       * 
       * Default constructor. Leap seconds information is populated in entries_ .
       * Values are hard-coded using source: http://maia.usno.navy.mil/ser7/tai-utc.dat .
       * Ths US Navy updates this file every Thursday.
       */
      LeapSecondsLookup();

      /** \brief Default destructor */
      ~LeapSecondsLookup();

      /**
       * \brief Get the TAI - UTC time difference in seconds at a given modified Julian date
       * \param[in] MJD   modified Julian date
       * \return          TAI - UTC time difference in seconds
       */
      double get_TAI_minus_UTC(double MJD) const;
      
      /**
       * \brief Get the GPS - UTC time difference in seconds at a given modified Julian date
       * \param[in] MJD   modified Julian date
       * \return          GPS - UTC time difference in seconds
       */
      double get_GPS_minus_UTC(double MJD) const;

    private:
      
      std::vector<LeapSeconds> entries_; /**< Leap second info storage */
      
    };

    /**
     *  \class GPSDateTIme
     *  \brief GPS Date and time and related operations
     */
    class GPSDateTime{

    public:

      /** \brief Default constructor */
      GPSDateTime();

      /** 
       * \brief Constructor 
       * \param[in] cycle   GPS cycle
       * \param[in] week    GPS week
       * \param[in] sec     GPS seconds past the week
       */
      GPSDateTime(uint cycle, uint week, double sec);						   
      /** \brief Default destructor */
      ~GPSDateTime();

      /**
       * \brief query the GPS cycle, week and seconds past the week
       * \param[out] cycle   GPS cycle
       * \param[out] week    GPS week
       * \param[out] sec     GPS seconds past the week
       * \return     true if the GPS date/time is valid
       */
      bool get(uint &cycle, uint &week, double &sec) const;

      /**
       *  \brief get GPS date-time in seconds
       *  \returns   GPS in seconds. -1 if invalid
       */
      double getSec() const;

      /**
       * \brief Get offset = GPS time - UTC time 
       * \returns    seconds offseet to UTC time
       */
      double getUTCTimeOffset() const;

    private:

      uint gps_cycle_; /**< GPS cycle */
      uint gps_week_;  /**< GPS week */
      double gps_sec_; /**< GPS seconds past the week */
      double mjd_;     /**< modified julian date equivalent */
      bool isValid_;   /**< flag for whether date/time is valid */

      static LeapSecondsLookup const leapSecLU; /**< leap second lookup */
      static boost::gregorian::date const date_19800106_;

    };






    class UTCDateTime{

    public:

      /** \brief Default constructor */ 
      UTCDateTime();

      /** 
       * \brief Constructor 
       * \param[in] year   UTC year
       * \param[in] month  UTC month 
       * \param[in] day    UTC day
       * \param[in] hour   UTC hour
       * \param[in] min    UTC minute
       * \param[in] sec    UTC second
       */
      UTCDateTime(int year, int month, int day, int hour, int min, double sec );

      /** 
       * \brief Constructor 
       * \param[in] nmea_date   UTC date field of NMEA GPRMC sentence 
       * \param[in] nmea_time   UTC time field of NMEA GPRMC sentence 
       */
      UTCDateTime(std::string nmea_date, std::string nmea_time);

      /** 
       * \brief Constructor 
       * \param nmea_gprmc   NMEA GPRMC sentence
       */ 
      UTCDateTime(std::string nmea_gprmc);

      /** \brief Default destructor */ 
      ~UTCDateTime();

      /** 
       * \brief convert the stored UTC date time to modified Julian date 
       * \return modified Julian date, or -1 if the date/time is invalid
       */
      double to_MJD() const; 

      /** 
       * \brief convert the stored UTC date time to GPS time, taking account
       * of the leap second offset
       * \return GPS time
       */
      GPSDateTime to_GPS() const;

    private:

      // static LeapSecondsLookup const leapSecLU; /**< leap second lookup */

      /**
       * \brief Constructor helper function
       */
      void init_nmea_date_time(std::string nmea_date, std::string nmea_time);

      /**
       * \brief Constructor helper function
       */
      void init_nmea_gprmc(std::string nmea_gprmc);

      boost::gregorian::date date_; /**< stored UTC date */
      int hh_; /**< stored UTC time - hour */
      int mm_; /**< stored UTC time - minute */
      double ss_; /**< stored UTC time - second */
      bool isTimeValid_; /**< flag for whether the time is valid */
      bool isDateValid_; /**< flag for whether the date is valid */

      static LeapSecondsLookup const leapSecLU;/**< leap second lookup */
      static boost::gregorian::date const date_19800106_;

    };

  } // namespace time
} // namespace conversion

#endif // TIME_CONV_HPP
