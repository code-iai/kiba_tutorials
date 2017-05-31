#include <kiba_control/kinematics_planner.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_planner");
  ros::NodeHandle nh("~");

  try
  {
    kiba_control::KinematicsPlanner kp(nh, "/follow_joint_trajectory", ros::Duration(2.0));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
     
  return 0;
}
