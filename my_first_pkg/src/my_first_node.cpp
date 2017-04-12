#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_first_node");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("my_num", 1);
  ros::Duration(1.0).sleep();
  std_msgs::Float64 msg;
  msg.data = -1.23;
  pub.publish(msg);
  ros::spin();
  return 0;
}
