#pragma once

#include "pthread.h"
#include <mutex>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <stdint.h>
// add socketCAN
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "boost/algorithm/string.hpp"
#include "boost/regex.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"
#include "common_msgs/CanRecieve.h"
#include "common_msgs/SystemOverall.h"
#include "common_msgs/SystemState.h"
#include "common_msgs/ControlState.h"
#include "common_msgs/Latcontrol.h"
#include "common_msgs/Loncontrol.h"
using namespace boost;

typedef union CHAR2UINT
{
	unsigned int i;
	unsigned char ch[4];
}CHAR2UINT;

typedef struct _CANMsg
{
	unsigned char head;
	unsigned ID;
	unsigned char data[8];
} stCANMsg;

typedef struct ControlCMD
{
	float lat_targetangle;
	int lat_targettorque;
	float lon_targetspeed;
	float lon_targetaccelation;
	float lon_targettorque;
	float lon_targetpressure;
	int8_t lon_actuatormode;
	int8_t lon_shiftposition;
	int8_t lon_emergency_obstacle;
	bool lon_epbflag;
	bool lon_brakelight;
	bool lon_reverselight;
} ControlCMD;

class PcieCan
{
  public:
	PcieCan(ros::NodeHandle nh, ros::NodeHandle private_nh);
	 ~PcieCan(); 
	void run();
  	void initSocketCAN();
  	void cansend_pcan_main();
  	void canreceive_pcan_main();
	void SendCarinfoEPS();
	void SendCarinfoESP();
  	void ParserData_socketCAN(struct can_frame *frame_rev, int num_rev);
  	void callback_control(const common_msgs::SystemOverallConstPtr& msg);

  	ros::Subscriber sub_control;
	ros::Publisher pub_control;

	common_msgs::CanRecieve carinfo_baic;
	ControlCMD vechileControl;

	int epscouter;
  	int sysswicht;
	int latswicht;
	int lonswicht;
	int StrWhlAgReqActv;
	int EPS_SysAvlSts;
	float steerCmd;

  private:
  	// add for socketCAN 
  	int s_send, nbytes_send;
	struct sockaddr_can addr_send;
	struct ifreq ifr_send;
	struct can_frame frame_send[2] = {{0}};

  	int s_rev, nbytes_rev;
	struct sockaddr_can addr_rev;
	struct ifreq ifr_rev;
	struct can_frame frame_rev;
	struct can_filter rfilter_rev[1];
};
