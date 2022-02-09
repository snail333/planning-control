 #include"pciecan.h"

 int main(int argc, char *argv[])
 {
   ros::init(argc, argv, "pciecan");
   ros::NodeHandle nh;
	 ros::NodeHandle private_nh("~");
   PcieCan node(nh, private_nh);
   node.run();
   return 0;
 }
