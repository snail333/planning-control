#include "parse_data.h"

imuparse::imuparse(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  private_nh.param<int>("baudRate", baudRate_, 115200);
  private_nh.param<std::string>("serialPort", serialPort_, "/dev/ttyUSB0");
  private_nh.param<int>("loopFrep", loopFrep_, 50);
  private_nh.param<bool>("deDug", deDug_, false);

  //paramters
  std::cout << FYEL("*****gnss:parameters*******************") << std::endl;
  std::cout << FGRN("gps_baudRate: ") << baudRate_ << std::endl;
  std::cout << FGRN("gps_serialPort: ") << serialPort_ << std::endl;
  std::cout << FGRN("gps_loopFrep: ") << loopFrep_ << std::endl;
  std::cout << FGRN("gps_deBug: ") << deDug_ << std::endl;
  std::cout << FYEL("*****gnss:parameters end***************") << std::endl;

  //data initialize
  memset(&_GPS, 0, sizeof(_GPS));

  //Create a publisher and name the topic.
  pub_gps = nh.advertise<common_msgs::sensorgps>("sensorgps", 500);
  pub_imu = nh.advertise<common_msgs::sensorimu>("sensorimu", 500);
  pub_odom = nh.advertise<common_msgs::wheelspeed>("sensor_odom", 500);

  try 
  {
    std::cout << FYEL(DEBUGHEAD) << "Serial start initialize" << std::endl;
    ser.setPort(serialPort_);
    ser.setBaudrate(baudRate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e) 
  {
    std::cout << FRED(DEBUGHEAD) << "Unable to open port" << std::endl;
    ROS_ERROR_STREAM(baudRate_);
    ROS_ERROR_STREAM(serialPort_);
  }
  if (ser.isOpen()) 
  {
    // Create time
    std::cout << FYEL(DEBUGHEAD) << "Serial port initialized successfully" << std::endl;
  }
  else 
  {
    std::cout << FRED(DEBUGHEAD) << "Serial port failed" << std::endl;
  }

}

imuparse::~imuparse(){}

void imuparse::run()
{
  ros::Rate rate(loopFrep_);
  while (ros::ok())
  {
    if(ser.available())
    {
      read = ser.read(ser.available());
      revData += read;
      GPS_DATA_Parse(revData, _GPS);
    }
    rate.sleep();
  }
}

void imuparse::GPS_DATA_Parse(std::string& input, GPS_INFO& GPS)
{
  uint32_t Length = 63;
  int32_t temp = 0;

  while (input.size() >= 3)
  {
    if (((input[0] & 0xff) == 0xbd)&&((input[1] & 0xff) == 0xdb)&&((input[2] & 0xff) == 0x0b))
    {
      if (input.size() < Length) 
      {
        break;
      }   
      uint8_t xorcheck63 = 0;
      for(size_t i = 0; i < Length - 1; i++)  
      {
        xorcheck63 = xorcheck63 ^ input[i];
      }
      if (input[Length-1] == xorcheck63)
      {
        // get RPY
        uint16_t roll = ((0xff & input[4]) << 8) | (0xff & input[3]);
        uint16_t pitch = ((0xff & input[6]) << 8) | (0xff & input[5]);
        uint16_t yaw = ((0xff & input[8]) << 8) | (0xff & input[7]);
        // calculate RPY in deg
        temp = *((int16_t*)& roll);
        GPS.rollf = (double)temp * (360.0/32768);
        temp = *((int16_t*)& pitch);
        GPS.pitchf = (double)temp * (360.0/32768);
        temp = *((int16_t*)& yaw);
        GPS.yawf = (double)temp * (360.0/32768);
        if (GPS.yawf < 0)
          GPS.yawf = GPS.yawf + 360;
                    
        // get gyro values
        uint16_t gx = ((0xff & input[10]) << 8) | (0xff & input[9]);
        uint16_t gy = ((0xff & input[12]) << 8) | (0xff & input[11]);
        uint16_t gz = ((0xff & input[14]) << 8) | (0xff & input[13]);
        // calculate gyro in deg/s
        temp = *((int16_t*)& gx);
        GPS.gxf = (double)temp * (360.0/32768);
        temp = *((int16_t*)& gy);
        GPS.gyf = (double)temp * (360.0/32768);
        temp = *((int16_t*)& gz);
        GPS.gzf = (double)temp * (360.0/32768);
                
        // get acelerometer values
        uint16_t ax = ((0xff & input[16]) << 8) | (0xff & input[15]);
        uint16_t ay = ((0xff & input[18]) << 8) | (0xff & input[17]);
        uint16_t az = ((0xff & input[20]) << 8) | (0xff & input[19]);   
        // calculate acelerometer in g
        temp = *((int16_t*)& ax);
        GPS.axf = (double)temp * (12.0/32768) * 9.8;   // m/s^2
        temp = *((int16_t*)& ay);
        GPS.ayf = (double)temp * (12.0/32768) * 9.8;
        temp = *((int16_t*)& az);
        GPS.azf = (double)temp * (12.0/32768) * 9.8;
                    
        // get gps values
        uint32_t latitude = ((0xff & input[24]) << 24) |((0xff & input[23]) << 16) |((0xff & input[22]) << 8) | (0xff & input[21]);
        uint32_t longitude = ((0xff & input[28]) << 24) |((0xff & input[27]) << 16) |((0xff & input[26]) << 8) | (0xff & input[25]);
        uint32_t altitude = ((0xff & input[32]) << 24) |((0xff & input[31]) << 16) |((0xff & input[30]) << 8) | (0xff & input[29]);
        // calculate lat、lon in deg(WGS84)
        temp = *((int32_t*)& latitude);
        GPS.latitudef = (double)temp * 1e-7L;
        temp = *((int32_t*)& longitude);
        GPS.longitudef = (double)temp * 1e-7L;
        // calculate lat、lon in m
        temp = *((int32_t*)& altitude);
        GPS.altitudef = (double)temp * 1e-3L;

        // get  NED vel values
        uint16_t Nvel = ((0xff & input[34]) << 8) | (0xff & input[33]);
        uint16_t Evel = ((0xff & input[36]) << 8) | (0xff & input[35]);
        uint16_t Dvel = ((0xff & input[38]) << 8) | (0xff & input[37]);
        // calculate NED vel in m/s
        temp = *((int16_t*)& Nvel);
        GPS.Nvelf = (double)temp * (100.0/32768);
        temp = *((int16_t*)& Evel);
        GPS.Evelf = (double)temp * (100.0/32768);
        temp = *((int16_t*)& Dvel);
        GPS.Dvelf = (double)temp * (100.0/32768);
        GPS.velocity = sqrt(GPS.Nvelf * GPS.Nvelf + GPS.Evelf * GPS.Evelf);

        // ins_status values
        uint8_t ins_status = (0x0f & input[39]);
        temp = *((uint8_t*)& ins_status);
        GPS.ins_statusf = (uint32_t)temp * 1;

        // pdata_type
        int pdata_type = (0xff & input[56]);
        
        // data 1-3
        uint16_t data1 = ((0xff & input[47]) << 8) | (0xff & input[46]);
        temp = *((int16_t*)& data1);
        int16_t data1f = (int16_t)temp * 1;
        uint16_t data2 = ((0xff & input[49]) << 8) | (0xff & input[48]);
        temp = *((int16_t*)& data2);
        int16_t data2f = (int16_t)temp * 1;
        uint16_t data3 = ((0xff & input[51]) << 8) | (0xff & input[50]);
        temp = *((int16_t*)& data3);
        int16_t data3f = (int16_t)temp * 1;

        switch (pdata_type)
        {
          case 0:
          GPS.latstd = (double)exp(data1f/100.0);
          GPS.lonstd = (double)exp(data2f/100.0);
          GPS.hstd = (double)exp(data3f/100.0);
          break;

          case 1:
          GPS.vnstd = (double)exp(data1f/100.0);
          GPS.vestd = (double)exp(data2f/100.0);
          GPS.vdstd = (double)exp(data3f/100.0);
          break;

          case 2:
          GPS.rollstd = (double)exp(data1f/100.0);
          GPS.pitchstd = (double)exp(data2f/100.0);
          GPS.yawstd = (double)exp(data3f/100.0);
          break;

          case 22:
          GPS.temperature = ((int32_t)data1f)*200/32768;
          break;

          case 32:
          GPS.position_type = (uint32_t)data1f;       //50=NARROW_INT
          GPS.numsv = (uint32_t)data2f;               //satenum
          GPS.heading_type = (uint32_t)data3f;
          break;

          case 33:
          GPS.wheel_speed_status = (uint32_t)data2f;    //0=no_wheel_speed
          break;
        }

        // get gps time values
        uint32_t gpst = ((0xff & input[55]) << 24) |((0xff & input[54]) << 16) |((0xff & input[53]) << 8) | (0xff & input[52]);
        // calculate gps time in ms
        temp = *((uint32_t*)& gpst);
        GPS.gpstf = ((uint32_t)temp) * 0.25;
                    

        uint8_t xorcheck63=0;
        for(int i = 58; i < 62; i++)  
        {
          xorcheck63=xorcheck63^input[i];
        }
        if (input[62] == xorcheck63) 
        {
          // get gps week values
          uint32_t gpsw = ((0xff & input[61]) << 24) |((0xff & input[60]) << 16) |((0xff & input[59]) << 8) | (0xff & input[58]);
          // calculate gps week
          temp = *((uint32_t*)& gpsw);
          GPS.gpswf = ((uint32_t)temp) * 1;
        }

        publishGps(GPS);
        publishImu(GPS);
        publishodom(GPS);
        
        if (deDug_)
        {
          cout << "GPS.rollf: " << GPS.rollf << " GPS.pitchf: " << GPS.pitchf << " GPS.yawf: " << GPS.yawf 
          << " GPS.gxf: " << GPS.gxf << " GPS.gyf: " << GPS.gyf << " GPS.gzf: " << GPS.gzf << endl;
          cout << " GPS.axf: " << GPS.axf << " GPS.ayf: " << GPS.ayf << " GPS.azf: " << GPS.azf << " GPS.latitudef: "
          << GPS.latitudef << " GPS.longitudef: " << GPS.longitudef << " GPS.altitudef: " << GPS.altitudef << endl;
          cout << " GPS.Nvelf: " << GPS.Nvelf << " GPS.Evelf: " << GPS.Evelf << " GPS.Dvelf: " << GPS.Dvelf 
          << " GPS.velocity: " << GPS.velocity << " GPS.ins_statusf: " << GPS.ins_statusf << endl;
          cout << " GPS.latstd: " << GPS.latstd << " GPS.lonstd: " << GPS.lonstd << " GPS.hstd: " << GPS.hstd 
          << " GPS.vnstd: " << GPS.vnstd << " GPS.vestd: " << GPS.vestd << " GPS.vdstd: " << GPS.vdstd << endl;
          cout << " GPS.rollstd: " << GPS.rollstd << " GPS.pitchstd: " << GPS.pitchstd << " GPS.yawstd: " << GPS.yawstd 
          << " GPS.temperature: " << GPS.temperature << " GPS.position_type: " << GPS.position_type << endl;
          cout << " GPS.numsv: " << GPS.numsv << " GPS.heading_type: " << GPS.heading_type << " GPS.wheel_speed_status: " 
          << GPS.wheel_speed_status << " GPS.gpstf: " << GPS.gpstf << " GPS.gpswf: " << GPS.gpswf << endl; 
        }

      }
      input.erase(0, Length); 
    }
    else 
    {
      input.erase(0,1);
    }
  }

}

void imuparse::publishGps(GPS_INFO& data)
{
	common_msgs::sensorgps msg;
  msg.lon = data.longitudef;
  msg.lat = data.latitudef;
  msg.height = data.altitudef;
  msg.heading = data.yawf;
  msg.velocity = data.velocity;
  msg.week = data.gpswf;
  msg.hdop = max(data.latstd, data.lonstd);
  msg.heading_std = data.yawstd;
  msg.status = data.ins_statusf;
  msg.satenum = data.numsv;
  msg.status_yaw = data.heading_type;
  msg.utctime = ros::Time::now().toSec();
  msg.update = true;
  msg.is_heading_valid = 1;
  
  pub_gps.publish(msg);
}

void imuparse::publishImu(GPS_INFO& data)
{
	common_msgs::sensorimu msg;
  msg.gyro_x = data.gxf;
  msg.gyro_y = data.gyf;
  msg.gyro_z = data.gzf;
  msg.acce_x = data.axf;
  msg.acce_y = data.ayf;
  msg.acce_z = data.azf;
  msg.heading = data.yawf;
  msg.pitch = data.pitchf;
  msg.roll = data.rollf;
  msg.temperature = data.temperature;
  msg.timetag = data.gpstf;
  msg.utctime = ros::Time::now().toSec();
  msg.update = true;

  pub_imu.publish(msg);
}

void imuparse::publishodom(GPS_INFO& data)
{
	common_msgs::wheelspeed msg;
  msg.update = data.wheel_speed_status;

  pub_odom.publish(msg);
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gnss");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	imuparse node(nh, private_nh);
	node.run();
	return 0;
}
