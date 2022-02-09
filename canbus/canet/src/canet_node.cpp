 #include"canet.h"

 int main(int argc, char *argv[])
 {
   ros::init(argc, argv, "canet");
   ros::NodeHandle nh;
	 ros::NodeHandle private_nh("~");
   Canet node(nh, private_nh);
   node.run();
   return 0;
 }
