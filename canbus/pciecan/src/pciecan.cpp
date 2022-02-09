#include "pciecan.h"

PcieCan::PcieCan(ros::NodeHandle nh, ros::NodeHandle private_nh)
{ 
  	latswicht = 0;
	lonswicht = 0;
	epscouter = 0;
	StrWhlAgReqActv = 0;
	EPS_SysAvlSts = 0;
	steerCmd = 0;

	memset(&carinfo_baic, 0, sizeof(carinfo_baic));
	memset(&vechileControl, 0, sizeof(vechileControl));
  	initSocketCAN();

	sub_control = nh.subscribe("control", 10, &PcieCan::callback_control, this);
  	pub_control = nh.advertise<common_msgs::CanRecieve>("pcican", 1);
}

PcieCan::~PcieCan()
{}

void PcieCan::initSocketCAN()
{
  	// send
	s_send = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	strcpy(ifr_send.ifr_name, "can3" );
	int send_ret = ioctl(s_send, SIOCGIFINDEX, &ifr_send);
	if(send_ret < 0)
		return;
	addr_send.can_family = AF_CAN;
	addr_send.can_ifindex = ifr_send.ifr_ifindex;
	::bind(s_send, (struct sockaddr *)&addr_send, sizeof(addr_send));
	setsockopt(s_send, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  
  	// receive
 	s_rev = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	strcpy(ifr_rev.ifr_name, "can3" );
	int rev_ret = ioctl(s_rev, SIOCGIFINDEX, &ifr_rev);
	if(rev_ret < 0)
		return;
	addr_rev.can_family = AF_CAN;
	addr_rev.can_ifindex = ifr_rev.ifr_ifindex;
	::bind(s_rev, (struct sockaddr *)&addr_rev, sizeof(addr_rev));
}

void PcieCan::run()
{ 
	//send
  	boost::function<void()> sendPcan = boost::bind(&PcieCan::cansend_pcan_main,this);
	boost::thread cansend_pcan_mainThread(sendPcan);
  	//receive
  	boost::function<void()> receivePcan = boost::bind(&PcieCan::canreceive_pcan_main,this);
  	boost::thread canreceive_pcan_mainThread(receivePcan);

  	canreceive_pcan_mainThread.join();
  	cansend_pcan_mainThread.join();
}

void PcieCan::ParserData_socketCAN(struct can_frame *frame_rev, int num_rev) 
{
  	for (int i = 0; i < num_rev; ++i)
  	{
    	stCANMsg frame;
    	frame.ID = frame_rev[i].can_id;
    	for (int j = 0; j < 8; ++j)
    	{
      		frame.data[j] = frame_rev[i].data[j];
    	}

		carinfo_baic.header.stamp = ros::Time::now();
      	switch (frame.ID)
      	{
	 	    case 0x347:  //转角
		    carinfo_baic.vehicleInfo.steerangle= (((frame.data[2]<<8)+ frame.data[3])*0.0625)-780;
		    pub_control.publish(carinfo_baic);
		    break;

	 	    case 0x1c4:
		    EPS_SysAvlSts = (frame.data[4]>>5)&0x03;
			carinfo_baic.vehicleInfo.current_drive_mode = EPS_SysAvlSts;
		    pub_control.publish(carinfo_baic);
		    break;

	 	    case 0x349:  //车速
		    carinfo_baic.vehicleInfo.vehcle_accel=(((frame.data[2]<<8)+ frame.data[3])*0.00125)-40.96;
		    carinfo_baic.vehicleInfo.speed = ((frame.data[4]<<8)+ frame.data[5])* 0.01-100;
		    pub_control.publish(carinfo_baic);
		    break;

	      	case 0x348:  //开关
        	sysswicht = (frame.data[3]&0x01);   //system总开关
		    latswicht = (frame.data[5]&0x02)>>1;
		    lonswicht = (frame.data[6]&0x07);
		    // if(sysswicht == 1)
		    // {
				if(latswicht == 1 && lonswicht == 1)
					carinfo_baic.keystate.drive_key_state = 1;
				else
					carinfo_baic.keystate.drive_key_state = 0;
		    // }
		    // else
		    // {
			//  	latswicht = 0;
			// 	lonswicht = 0;
			// 	carinfo_baic.keystate.drive_key_state = 0;
		    // }
		    pub_control.publish(carinfo_baic);
		    break;

		    default:
			pub_control.publish(carinfo_baic);
		    break;
      	}
  	}
}

void PcieCan::SendCarinfoEPS()
{
	int APA_Sts = 0;
	int ApSysActv = 0;
	float steer_err = 0;

	if(latswicht == 1)
	{
		APA_Sts = 4;
	  	ApSysActv = 1;
		if(StrWhlAgReqActv != 2)
		{
			StrWhlAgReqActv = 1;
			if(++epscouter > 11)
			{
				StrWhlAgReqActv = 2;
				epscouter = 0;
			}
		}
	}
	else
	{
		APA_Sts = 0;
	  	ApSysActv = 0;
		StrWhlAgReqActv = 0;
	}

	if(EPS_SysAvlSts == 2)
	{
		steer_err = vechileControl.lat_targetangle - steerCmd;
		steer_err = steer_err > 12? 12 : steer_err;
		steer_err = steer_err < -12? -12 : steer_err;
		steer_err =  fabs(steer_err) < 2? 0 : steer_err;
		steerCmd += steer_err;
		if(steer_err > 0 && steerCmd > vechileControl.lat_targetangle)
			steerCmd =  vechileControl.lat_targetangle;
		else if(steer_err < 0 && steerCmd < vechileControl.lat_targetangle)
			steerCmd =  vechileControl.lat_targetangle;
	}
	else
	{
		steerCmd = carinfo_baic.vehicleInfo.steerangle;
	}

  	if(steerCmd > 500)
		steerCmd = 500;
  	else if(steerCmd < -500)
		steerCmd = -500;

  	// socketCAN
  	frame_send[0].can_id = 0x100;
  	frame_send[0].can_dlc = 8;
  	frame_send[0].data[2] = (ushort)((vechileControl.lon_shiftposition & 0x03)<<6)+(ushort)(APA_Sts&0x0f)+(ushort)((ApSysActv&0x01)<<4);//档位请求
  	frame_send[0].data[3] = (ushort)((StrWhlAgReqActv)&0x03)>>0;//方向盘转角请求
  	frame_send[0].data[4] = (ushort)((steerCmd+780)/0.0625)>>8;//方向盘转角
  	frame_send[0].data[5] = (ushort)((steerCmd+780)/0.0625)&0xff;
  	//发送 frame[0]
  	nbytes_send = write(s_send, &frame_send[0], sizeof(frame_send[0]));
	if(nbytes_send != sizeof(frame_send[0]))
		std::cout << "Send Error frame_send[0] !" << std::endl;
}

void PcieCan::SendCarinfoESP()
{
	int torque_req = 0;
	int acc_req = 0;
	int Sts_ACC = 0;

	if(lonswicht == 1)
  	{
	  Sts_ACC = 3;
	  if(vechileControl.lon_targetaccelation >= 0)
	  {
		  torque_req = 1;
		  acc_req = 0;
		  if(vechileControl.lon_targettorque > 5000)
		  	vechileControl.lon_targettorque = 5000;
      	  else if(vechileControl.lon_targettorque < 0)
        	vechileControl.lon_targettorque = 0;
	  }
	  else
	  {
		  torque_req = 0;
		  acc_req = 1;
		  if(vechileControl.lon_targetaccelation < -7)
			  vechileControl.lon_targetaccelation = -7;
	  }
  	}

  	// socketCAN
  	frame_send[1].can_id = 0x500;
  	frame_send[1].can_dlc = 8;
  	frame_send[1].data[3] = (ushort)(((torque_req)&0x01)<<0)+(ushort)(((Sts_ACC)&0x0f)<<1);//ACC扭矩请求
  	frame_send[1].data[4] = (ushort)(vechileControl.lon_targettorque+32768)>>8;//扭矩
  	frame_send[1].data[5] = (ushort)(vechileControl.lon_targettorque+32768)&0xff;
  	frame_send[1].data[6] = (((ushort)((vechileControl.lon_targetaccelation+7.22)/0.005))&0x0ff0)>>4;//减速度
  	frame_send[1].data[7] = (ushort)((((ushort)((vechileControl.lon_targetaccelation+7.22)/0.005))&0x0f)<<4)+(ushort)((acc_req & 0x01)<<2);
  	//发送 frame[1]
  	nbytes_send = write(s_send, &frame_send[1], sizeof(frame_send[1]));
  	if(nbytes_send != sizeof(frame_send[1]))
  		std::cout << "Send Error frame_send[1] !" << std::endl;
}

void PcieCan::cansend_pcan_main()
{
  	ros::Rate loop_rate(50);
  	while(ros::ok())
  	{
    	ros::spinOnce();
		SendCarinfoEPS();
		SendCarinfoESP();
    	loop_rate.sleep();
  	}

	int out = 0;
	memset(&frame_send, 0, sizeof(frame_send));
	while (++out < 10)
	{
		frame_send[0].can_id = 0x100;
  		frame_send[0].can_dlc = 8;
		nbytes_send = write(s_send, &frame_send[0], sizeof(frame_send[0]));
		frame_send[1].can_id = 0x500;
  		frame_send[1].can_dlc = 8;
		nbytes_send = write(s_send, &frame_send[1], sizeof(frame_send[1]));
	}
	close(s_send);
}

void PcieCan::canreceive_pcan_main()
{
  	while(ros::ok())
  	{     
		nbytes_rev = read(s_rev, &frame_rev, sizeof(frame_rev));
    	if (nbytes_rev > 0 && nbytes_rev == sizeof(frame_rev))
    	{
      		ParserData_socketCAN(&frame_rev, nbytes_rev);
    	}
  	}
	close(s_rev);
}

void PcieCan::callback_control(const common_msgs::SystemOverallConstPtr& msg)
{
	vechileControl.lat_targetangle = msg->systemstate.latcontrol.targetangle;
	vechileControl.lat_targettorque = msg->systemstate.latcontrol.targettorque;
	vechileControl.lon_targetspeed = msg->systemstate.loncontrol.targetspeed;
	vechileControl.lon_targetaccelation = msg->systemstate.loncontrol.targetaccelation;
	vechileControl.lon_targettorque = msg->systemstate.loncontrol.targettorque;
	vechileControl.lon_targetpressure = msg->systemstate.loncontrol.targetpressure;
	vechileControl.lon_actuatormode = msg->systemstate.loncontrol.actuatormode;
	vechileControl.lon_shiftposition = msg->systemstate.loncontrol.shiftposition;
	vechileControl.lon_emergency_obstacle = msg->systemstate.loncontrol.emergency_obstacle;
	vechileControl.lon_epbflag = msg->systemstate.loncontrol.epbflag;
	vechileControl.lon_brakelight = msg->systemstate.loncontrol.brakelight;
	vechileControl.lon_reverselight = msg->systemstate.loncontrol.reverselight;
}
