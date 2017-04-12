#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_first_node");
  ros::spin();
  return 0;
}

