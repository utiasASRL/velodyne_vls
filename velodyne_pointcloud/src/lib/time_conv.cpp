/** 
 *  \file time_conv.cpp
 *  \brief  C++ library for GPS-related time conversion
 *  \copyright Copyright 2018 Applanix Corp (Trimble Inc.). All rights reserved.
 *  \license Applanix Corp (Trimble Inc.) proprietary.
 *  \author Keith Leung
 */

#include "velodyne_pointcloud/time_conv.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <sstream>


namespace conversion { 
  namespace time {
 
    //--- LeapSecondsLookup class implementation ---//

    LeapSecondsLookup::LeapSecondsLookup(){

      // Source: http://maia.usno.navy.mil/ser7/tai-utc.dat
      entries_.push_back( {1961, 1,  1, 2437300.5,   1.4228180, 37300, 0.001296 } );
      entries_.push_back( {1961, 8,  1, 2437512.5,   1.3728180, 37300, 0.001296 } );
      entries_.push_back( {1962, 1,  1, 2437665.5,   1.8458580, 37665, 0.0011232} );
      entries_.push_back( {1963, 11, 1, 2438334.5,   1.9458580, 37665, 0.0011232} );
      entries_.push_back( {1964, 1,  1, 2438395.5,   3.2401300, 38761, 0.001296 } );
      entries_.push_back( {1964, 4,  1, 2438486.5,   3.3401300, 38761, 0.001296 } );
      entries_.push_back( {1964, 9,  1, 2438639.5,   3.4401300, 38761, 0.001296 } );
      entries_.push_back( {1965, 1,  1, 2438761.5,   3.5401300, 38761, 0.001296 } );
      entries_.push_back( {1965, 3,  1, 2438820.5,   3.6401300, 38761, 0.001296 } );
      entries_.push_back( {1965, 7,  1, 2438942.5,   3.7401300, 38761, 0.001296 } );
      entries_.push_back( {1965, 9,  1, 2439004.5,   3.8401300, 38761, 0.001296 } );
      entries_.push_back( {1966, 1,  1, 2439126.5,   4.3131700, 39126, 0.002592 } );
      entries_.push_back( {1968, 2,  1, 2439887.5,   4.2131700, 39126, 0.002592 } );
      entries_.push_back( {1972, 1,  1, 2441317.5,  10.0      , 41317, 0.0      } );
      entries_.push_back( {1972, 7,  1, 2441499.5,  11.0      , 41317, 0.0      } );
      entries_.push_back( {1973, 1,  1, 2441683.5,  12.0      , 41317, 0.0      } );
      entries_.push_back( {1974, 1,  1, 2442048.5,  13.0      , 41317, 0.0      } );
      entries_.push_back( {1975, 1,  1, 2442413.5,  14.0      , 41317, 0.0      } );
      entries_.push_back( {1976, 1,  1, 2442778.5,  15.0      , 41317, 0.0      } );
      entries_.push_back( {1977, 1,  1, 2443144.5,  16.0      , 41317, 0.0      } );
      entries_.push_back( {1978, 1,  1, 2443509.5,  17.0      , 41317, 0.0      } );
      entries_.push_back( {1979, 1,  1, 2443874.5,  18.0      , 41317, 0.0      } );
      entries_.push_back( {1980, 1,  1, 2444239.5,  19.0      , 41317, 0.0      } );
      entries_.push_back( {1981, 7,  1, 2444786.5,  20.0      , 41317, 0.0      } );
      entries_.push_back( {1982, 7,  1, 2445151.5,  21.0      , 41317, 0.0      } );
      entries_.push_back( {1983, 7,  1, 2445516.5,  22.0      , 41317, 0.0      } );
      entries_.push_back( {1985, 7,  1, 2446247.5,  23.0      , 41317, 0.0      } );
      entries_.push_back( {1988, 1,  1, 2447161.5,  24.0      , 41317, 0.0      } );
      entries_.push_back( {1990, 1,  1, 2447892.5,  25.0      , 41317, 0.0      } );
      entries_.push_back( {1991, 1,  1, 2448257.5,  26.0      , 41317, 0.0      } );
      entries_.push_back( {1992, 7,  1, 2448804.5,  27.0      , 41317, 0.0      } );
      entries_.push_back( {1993, 7,  1, 2449169.5,  28.0      , 41317, 0.0      } );
      entries_.push_back( {1994, 7,  1, 2449534.5,  29.0      , 41317, 0.0      } );
      entries_.push_back( {1996, 1,  1, 2450083.5,  30.0      , 41317, 0.0      } );
      entries_.push_back( {1997, 7,  1, 2450630.5,  31.0      , 41317, 0.0      } );
      entries_.push_back( {1999, 1,  1, 2451179.5,  32.0      , 41317, 0.0      } );
      entries_.push_back( {2006, 1,  1, 2453736.5,  33.0      , 41317, 0.0      } );
      entries_.push_back( {2009, 1,  1, 2454832.5,  34.0      , 41317, 0.0      } );
      entries_.push_back( {2012, 7,  1, 2456109.5,  35.0      , 41317, 0.0      } );
      entries_.push_back( {2015, 7,  1, 2457204.5,  36.0      , 41317, 0.0      } );
      entries_.push_back( {2017, 1,  1, 2457754.5,  37.0      , 41317, 0.0      } );

      for(int i = 0; i < entries_.size(); i++ ){
	// Calculate MJD and total offset for each entry
	entries_[i].MJD = entries_[i].JD - JD_minus_MJD_OFFSET;
	entries_[i].TAI_minus_UTC = 
	  entries_[i].TAI_minus_UTC_offset + entries_[i].TAI_minus_UTC_drift2 *
	  (entries_[i].MJD - entries_[i].TAI_minus_UTC_drift1);
      }
    }

    LeapSecondsLookup::~LeapSecondsLookup(){}

    double LeapSecondsLookup::get_TAI_minus_UTC(double MJD) const{

      for(uint i = entries_.size() - 1; i >= 0; i-- ){

	if(MJD >= entries_[i].MJD){
	  return entries_[i].TAI_minus_UTC;
	}
      }
      return 0;
    }


    double LeapSecondsLookup::get_GPS_minus_UTC(double MJD) const{
      
      return get_TAI_minus_UTC(MJD) - TAI_minus_GPS_OFFSET;
    }






    //--- GPSDateTime Class Implementation ---//

    // static members
    LeapSecondsLookup const GPSDateTime::leapSecLU = {};
    boost::gregorian::date const GPSDateTime::date_19800106_ = {1980, 1, 6};

    GPSDateTime::GPSDateTime():
      gps_cycle_(0), gps_week_(0), gps_sec_(-1), isValid_(false)
    {}

    GPSDateTime::GPSDateTime(uint cycle, uint week, double sec):
      gps_cycle_(cycle), gps_week_(week), gps_sec_(sec), isValid_(true)
    {
      if(gps_sec_ < 0){
	isValid_ = false;
      }else{
	while(gps_sec_ > SECS_PER_WEEK ){
	  gps_sec_ -= SECS_PER_WEEK;
	  gps_week_++;
	}
	while(gps_week_ > 1023){
	  gps_week_ -= 1024;
	  gps_cycle_++;
	}
      }

      double gps_day = gps_sec_ / SECS_PER_DAY + gps_week_ * 7 + gps_cycle_ * 1024 * 7;
      mjd_ = gps_day + date_19800106_.modjulian_day();
      
    }

    GPSDateTime::~GPSDateTime(){}

    bool GPSDateTime::get(uint &cycle, uint &week, double &sec) const{

	cycle = gps_cycle_;
	week = gps_week_;
	sec = gps_sec_;

	return isValid_;
    }


    double GPSDateTime::getSec() const{

      return ( gps_sec_ + (gps_cycle_ * 1027 + gps_week_) * SECS_PER_WEEK );
    }

    double GPSDateTime::getUTCTimeOffset() const{

      double utc_time_offset = leapSecLU.get_GPS_minus_UTC(mjd_);
      return utc_time_offset;

    }







    //--- UTCDateTime Class implementation ---//

    // static members
    LeapSecondsLookup const UTCDateTime::leapSecLU = {};
    boost::gregorian::date const UTCDateTime::date_19800106_ = {1980, 1, 6};

    UTCDateTime::UTCDateTime()
    {
      boost::posix_time::ptime t(boost::posix_time::microsec_clock::universal_time());
      date_ = t.date();
      hh_ = t.time_of_day().hours();
      mm_ = t.time_of_day().minutes();
      ss_ = (double)t.time_of_day().total_microseconds() / 1e6 - hh_ * 3600 - mm_ * 60;
    }
    
    UTCDateTime::UTCDateTime(int year, int month, int day, int hour, int min, double sec):
      hh_(hour), mm_(min), ss_(sec), date_(year, month, day), isTimeValid_(true), isDateValid_(true)
    {
      if(hh_ < 0 || mm_ < 0 || ss_ < 0){

	isTimeValid_ = false;
      }
      if(date_.is_not_a_date()){

        isDateValid_ = false;
      }
      
    }

    UTCDateTime::UTCDateTime(std::string nmea_date, std::string nmea_time)
    {
      init_nmea_date_time(nmea_date, nmea_time);
    } 
    
    UTCDateTime::UTCDateTime(std::string nmea_gprmc)
    {
      init_nmea_gprmc(nmea_gprmc);
    }

    UTCDateTime::~UTCDateTime(){}

    void UTCDateTime::init_nmea_date_time(std::string nmea_date, std::string nmea_time){

      if( nmea_date.length() == 6){

	int day = std::stoi( nmea_date.substr(0, 2) );
	int month = std::stoi( nmea_date.substr(2, 2) );
	int year = std::stoi( nmea_date.substr(4, 2) );
	if( year < 78 ){ // GPS first launch in year 1978 
	  year += 2000;
	}else{
	  year += 1900;
	}
	date_ = boost::gregorian::date(year, month, day);
	isDateValid_ = true;
      
      }else{

	isDateValid_ = false;
      }

      if( nmea_time.length() >= 6){

	int hour = std::stoi( nmea_time.substr(0, 2) );      
	int min = std::stoi( nmea_time.substr(2, 2) );
	double sec = std::stod( nmea_time.substr(4, std::string::npos) );
     
	hh_ = hour;
	mm_ = min;
	ss_ = sec;
	isTimeValid_ = true;

      }else{ // NMEA time too short to be valid

	hh_ = -1;
	mm_ = -1;
	ss_ = -1;
	isTimeValid_ = false;
      }      
    }

    void UTCDateTime::init_nmea_gprmc(std::string nmea_gprmc){

      // Extract time and date fields
      std::istringstream nmea_gprmc_ss(nmea_gprmc);
      std::string nmea_gprmc_field;
      std::getline(nmea_gprmc_ss, nmea_gprmc_field, ','); // $GPRMC
      std::string nmea_gprmc_time; 
      std::getline(nmea_gprmc_ss, nmea_gprmc_time, ','); 
      char nmea_gprmc_validity, comma;       
      float nmea_gprmc_lat;
      char nmea_gprmc_lat_dir;
      float nmea_gprmc_lon;
      char nmea_gprmc_lon_dir;
      float nmea_gprmc_course;
      float nmea_gprmc_speed_kt;
      nmea_gprmc_ss >> nmea_gprmc_validity >> comma
		    >> nmea_gprmc_lat >> comma
		    >> nmea_gprmc_lat_dir >> comma
		    >> nmea_gprmc_lon >> comma
		    >> nmea_gprmc_lon_dir >> comma
		    >> nmea_gprmc_course >> comma
		    >> nmea_gprmc_speed_kt >> comma;
      std::string nmea_gprmc_date;
      std::getline(nmea_gprmc_ss, nmea_gprmc_date, ','); 
      float nmea_gprmc_var;
      char nmea_gprmc_var_dir;
      std::string nmea_gprmc_checksum;
      nmea_gprmc_ss >> nmea_gprmc_var >> comma
		    >> nmea_gprmc_var_dir >> comma
		    >> nmea_gprmc_checksum;

      init_nmea_date_time(nmea_gprmc_date, nmea_gprmc_time);
    }

    double UTCDateTime::to_MJD() const{
      
      if(isTimeValid_ && isDateValid_){
	return (double)(date_.modjulian_day()) + hh_/24.0 + mm_/1440.0 + ss_ / SECS_PER_DAY;
      }else{
	return -1;
      }
    }

    GPSDateTime UTCDateTime::to_GPS() const{

      double const mjd_from_UTC = to_MJD();
      if(mjd_from_UTC == -1){
	return GPSDateTime();
      }
      double const GPS_UTC_offset = leapSecLU.get_GPS_minus_UTC(mjd_from_UTC);
      double const mjd_from_GPS = mjd_from_UTC + GPS_UTC_offset / SECS_PER_DAY;

      double gps_day = mjd_from_GPS - date_19800106_.modjulian_day();
      uint gps_week = gps_day / 7;
      uint gps_cycle = gps_week / 1024;
      double gps_sec_past_week = (gps_day - 7 * gps_week) * SECS_PER_DAY;
      gps_week = gps_week - gps_cycle * 1024;

      return GPSDateTime(gps_cycle, gps_week, gps_sec_past_week);
      
    }

    
  } // namespace time
} // namespace conversion
