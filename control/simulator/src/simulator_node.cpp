#include "simulator/simulator_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulator");
  WFSimulatorCore obj;
  ros::spin();
  return 0;
};
