#pragma once

#include "pthread.h"
#include <mutex>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <stdint.h>
#include "common_msgs/CanRecieve.h"
#include "common_msgs/SystemOverall.h"
#include "common_msgs/SystemState.h"
#include "common_msgs/ControlState.h"
#include "common_msgs/Latcontrol.h"
#include "common_msgs/Loncontrol.h"
#include "Boost_UDP.h"


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

const static int RECVBUFFERSIZE = 1300;

class Canet
{
   public:
	Canet(ros::NodeHandle nh, ros::NodeHandle private_nh);
	 ~Canet(); 
	void run();
	void initUdp();
	void callback_control(const common_msgs::SystemOverallConstPtr& msg);
	void ParserData(unsigned char data[], int num);
	void cansend_pcan_main();
	void canreceive_pcan_main();
	void SendCarinfoEPS();
	void SendCarinfoESP();
  
  	ros::Subscriber sub_control;
	ros::Publisher pub_control;

	std::string m_canetip1,m_canetip2;
	int m_canetport1,m_canetport2;
	std::string m_pcip1,m_pcip2;
	int m_pcport1,m_pcport2;

    Boost_UDP* boost_udp1;
    unsigned char buffer1[RECVBUFFERSIZE];

	common_msgs::CanRecieve carinfo_baic;
	ControlCMD vechileControl;

	int epscouter;
	int latswicht;
	int lonswicht;
	int StrWhlAgReqActv;
	int EPS_SysAvlSts;
	float steerCmd;
};
