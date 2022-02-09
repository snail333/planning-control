#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string>
#include <cmath>
#include <memory.h>	
#include <time.h>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#include "common_msgs/sensorgps.h"
#include "common_msgs/sensorimu.h"
#include "common_msgs/wheelspeed.h"

/* FOREGROUND */

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#define DEBUGHEAD "[gnss-->]"
using namespace std;

typedef struct GPS_INFO{

  double rollf, pitchf, yawf;
  double gxf, gyf, gzf;
  double axf, ayf, azf;
  double latitudef, longitudef, altitudef;
  double Nvelf, Evelf, Dvelf, velocity;
  uint32_t ins_statusf;
  double latstd, lonstd, hstd; 
  double vnstd, vestd, vdstd;
  double rollstd, pitchstd, yawstd;
  int32_t temperature;
  uint32_t position_type, numsv, heading_type;
  uint32_t wheel_speed_status;
  uint32_t gpstf;
  uint32_t gpswf;
}GPS_INFO;

//define windows time struct
typedef struct SYSTEMTIME{
  int wSecond;			 /* Seconds.	 [0-60] (1 leap second) */
  int wMinute;			 /* Minutes.	 [0-59] */
  int wHour;			 /* Hours.	 [0-23] */
  int wDay;			    /* Day.		 [1-31] */
  int wMonth;			 /* Month.	 [0-11] */
  int wYear;			 /* Year	 - 1900.  */
  int wDayOfWeek;		 /* Day of week.	 [0-6] */
  int tm_yday;			 /* Days in year.[0-365]	 */
  int tm_isdst;			 /* DST.		 [-1/0/1]*/
#ifdef	__USE_BSD
  long int tm_gmtoff;		 /* Seconds east of UTC.  */
  __const char *tm_zone;	 /* Timezone abbreviation.  */
#else
  long int __tm_gmtoff;		 /* Seconds east of UTC.  */
  __const char *__tm_zone;	 /* Timezone abbreviation.  */
#endif
}SYSTEMTIME;


class imuparse
{
  public:
  imuparse(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~imuparse();
  void run();

  //void Datadeal();
  void GPS_DATA_Parse(std::string& input, GPS_INFO& GPS);
  void publishGps(GPS_INFO& data);
  void publishImu(GPS_INFO& data);
  void publishodom(GPS_INFO& data);

  private:
  ros::Publisher pub_gps;
  ros::Publisher pub_imu;
  ros::Publisher pub_odom;
  serial::Serial ser;

  GPS_INFO _GPS;
  int baudRate_; 
  int loopFrep_;
  bool deDug_;
  std::string serialPort_;

  std::string read;
  std::string revData;
};

#endif