#include "canet.h"

Canet::Canet(ros::NodeHandle nh, ros::NodeHandle private_nh)
{ 
  	latswicht = 0;
	lonswicht = 0;
	epscouter = 0;
	StrWhlAgReqActv = 0;
	EPS_SysAvlSts = 0;
	steerCmd = 0;

	private_nh.param<std::string>("m_canetip1", m_canetip1, "192.168.2.178");
  	private_nh.param<int>("m_canetport1", m_canetport1, 4003);
  	private_nh.param<std::string>("m_pcip1", m_pcip1, "192.168.2.100");
  	private_nh.param<int>("m_pcport1", m_pcport1, 8003);

	memset(&carinfo_baic, 0, sizeof(carinfo_baic));
	memset(&vechileControl, 0, sizeof(vechileControl));
	initUdp();

	sub_control = nh.subscribe("control", 10, &Canet::callback_control, this);
  	pub_control = nh.advertise<common_msgs::CanRecieve>("pcican", 1);
}

Canet::~Canet()
{
   	delete boost_udp1;
}

void Canet::initUdp()
{
   //bind socket and receive_data
   boost::asio::io_service io_service1;
   boost_udp1 = new Boost_UDP(io_service1, m_pcip1, m_pcport1, m_canetip1, m_canetport1);
   boost_udp1->start_sock();
}

void Canet::run()
{ 
	//PCan send
  	boost::function<void()> sendPcan = boost::bind(&Canet::cansend_pcan_main,this);
	boost::thread cansend_pcan_mainThread(sendPcan);
   	//P+BCan receive
   	boost::function<void()> receivePcan = boost::bind(&Canet::canreceive_pcan_main,this);
   	boost::thread canreceive_pcan_mainThread(receivePcan);

  	canreceive_pcan_mainThread.join();
  	cansend_pcan_mainThread.join();
}

void Canet::ParserData(unsigned char data[], int num)
{
  for (int i = 0; i < num / 13; ++i)
  {
    stCANMsg frame;
    CHAR2UINT ii;

    for (int j = 0; j < 4; ++j)
    {
      ii.ch[3 - j] = data[1 + j + i * 13];
    }
    unsigned int id = ii.i;
    frame.ID = id;
    for (int j = 0; j < 8; ++j)
    {
      frame.data[j] = data[5 + j + i * 13];
    }

    try
    {
      switch (frame.ID)
      {
  
	 	case 0x1c2:
		carinfo_baic.vehicleInfo.steerangle= (((frame.data[2]<<8)+ frame.data[3])*0.0625)-780;
		pub_control.publish(carinfo_baic);
		break;

	 	case 0x1c4:
		EPS_SysAvlSts = (frame.data[4]>>5)&0x03;
		pub_control.publish(carinfo_baic);
		break;

	 	case 0x300://车速
		carinfo_baic.vehicleInfo.vehcle_accel=(((frame.data[2]<<8)+ frame.data[3])*0.00125)-40.96;
		carinfo_baic.vehicleInfo.speed = ((frame.data[4]<<8)+ frame.data[5])* 0.01-100;
		pub_control.publish(carinfo_baic);
		break;

	    case 0x202://开关
		latswicht=(frame.data[5]&0x02)>>1;
		lonswicht=(frame.data[6]&0x07);
		if(latswicht == 1 && lonswicht == 1)
		{
			carinfo_baic.keystate.drive_key_state = 1;
		}
		else
		{
			carinfo_baic.keystate.drive_key_state = 0;
		}
		pub_control.publish(carinfo_baic);
		break;

		default:
			pub_control.publish(carinfo_baic);
		  break;
      }
	
    }
    catch (int control_exception)
    {
      if(control_exception==1)
        ROS_ERROR("EPS fault!!!");
    }
  }
}

void Canet::SendCarinfoEPS()
{
	int APA_Sts = 0;
	int ApSysActv = 0;
	float steer_err  = 0;
	unsigned char eps_buf[13] = {0, 0};

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
		if(steer_err >= 0 && steerCmd >  vechileControl.lat_targetangle)
			steerCmd =  vechileControl.lat_targetangle;
		else if(steer_err < 0 && steerCmd <  vechileControl.lat_targetangle)
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

	eps_buf[0] = 0x08;
	eps_buf[1] = 0x00;
	eps_buf[2] = 0x00;
	eps_buf[3] = 0x01;
	eps_buf[4] = 0x00;
	eps_buf[7] = (ushort)((vechileControl.lon_shiftposition & 0x03)<<6)+(ushort)(APA_Sts&0x0f)+(ushort)((ApSysActv&0x01)<<4);//档位请求
	eps_buf[8] = (ushort)((StrWhlAgReqActv)&0x03)>>0;//方向盘转角请求
	eps_buf[9] = (ushort)((steerCmd+780)/0.0625)>>8;//方向盘转角
	eps_buf[10] = (ushort)((steerCmd+780)/0.0625)&0xff;
	boost_udp1->send_data(eps_buf, 13); //通过UDP发送数据
}

void Canet::SendCarinfoESP()
{
	unsigned char esp_buf[13] = {0, 0};
	int torque_req = 0;
	int acc_req = 0;
	int Sts_ACC = 0;

	if(lonswicht==0)
		return;

	if(vechileControl.lon_targetaccelation >= 0)
	{
		torque_req=1;
		acc_req=0;
		Sts_ACC=3;
		if(vechileControl.lon_targettorque > 10000)
		{
			vechileControl.lon_targettorque = 10000;
		}
	}
	else
	{
		torque_req=0;
		acc_req=1;
		Sts_ACC=3;
		if(vechileControl.lon_targetaccelation < -7)
		{
			vechileControl.lon_targetaccelation = -7;
		}
	}

	esp_buf[0] = 0x08;
	esp_buf[1] = 0x00;
	esp_buf[2] = 0x00;
	esp_buf[3] = 0x05;
	esp_buf[4] = 0x00;
	esp_buf[8] = (ushort)(((torque_req)&0x01)<<0)+(ushort)(((Sts_ACC)&0x0f)<<1);//ACC扭矩请求
	esp_buf[9] = (ushort)(vechileControl.lon_targettorque+32768)>>8;//扭矩
	esp_buf[10] = (ushort)(vechileControl.lon_targettorque+32768)&0xff;
	esp_buf[11] = (((ushort)((vechileControl.lon_targetaccelation+7.22)/0.005))&0x0ff0)>>4;//减速度
	esp_buf[12] = (ushort)((((ushort)((vechileControl.lon_targetaccelation+7.22)/0.005))&0x0f)<<4)+(ushort)(((acc_req)&0x01)<<2);
	boost_udp1->send_data(esp_buf, 13); //通过UDP发送数据
}

void Canet::cansend_pcan_main()
{
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();
	SendCarinfoEPS();
	SendCarinfoESP();
    loop_rate.sleep();
  }
}

void Canet::canreceive_pcan_main()
{
  ros::Rate loop_rate(50);
  while(ros::ok())
  {     
    memset(buffer1, 0, sizeof(buffer1));
    int ret_rec = boost_udp1->receive_data(buffer1);
    if (ret_rec > 0 && ret_rec % 13 == 0)
    {
      ParserData(buffer1, ret_rec);
    }
    loop_rate.sleep();
  }
}

void Canet::callback_control(const common_msgs::SystemOverallConstPtr& msg)
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
