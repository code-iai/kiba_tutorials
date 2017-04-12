#include "ros/ros.h"
#include "std_msgs/Float64.h"

double message_content;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_first_node");
  ros::NodeHandle nh("~");
  if (!nh.getParam("num_to_pub", message_content))
  {
    ROS_ERROR("Could not find parameter 'num_to_pub' in namespace '%s'",
        nh.getNamespace().c_str());
    return 0;
  }
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("my_num", 1);
  ros::Duration(1.0).sleep();
  std_msgs::Float64 msg;
  msg.data = message_content;
  pub.publish(msg);
  ros::spin();
  return 0;
}
