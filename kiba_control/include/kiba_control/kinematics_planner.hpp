#ifndef KIBA_CONTROL_KINEMATICS_PLANNER_H
#define KIBA_CONTROL_KINEMATICS_PLANNER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kiba_control/utils.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace kiba_control
{
  class KinematicsPlanner
  {
    public:
      KinematicsPlanner(const ros::NodeHandle& nh, const std::string& action_name,
          const ros::Duration& server_timeout) :
        nh_( nh ),
        action_client_( nh_, action_name, true ),
        js_sub_( nh_.subscribe("joint_states", 1, &KinematicsPlanner::js_callback, this) ),
        goal_sub_( nh_.subscribe("goal", 1, &KinematicsPlanner::goal_callback, this) ),
        robot_model_( read_robot_model(nh_) ),
        root_frame_name_( readParam<std::string>(nh, "root_frame_name") ),
        tip_frame_name_( readParam<std::string>(nh, "tip_frame_name") ),
        chain_( extract_chain(nh_, robot_model_, root_frame_name_, tip_frame_name_) )
      {
        if (!action_client_.waitForServer(server_timeout))
          throw std::runtime_error("Waited for server " + std::to_string(server_timeout.toSec()) + "s. Aborting.");
      }

      ~KinematicsPlanner()
      {}

    private:
      ros::NodeHandle nh_;
      ros::Subscriber js_sub_, goal_sub_;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
      std::string root_frame_name_, tip_frame_name_;
      urdf::Model robot_model_;
      KDL::Chain chain_;
      sensor_msgs::JointState current_joint_state_;

      void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        current_joint_state_ = *msg;
      }

      void goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
      {
        // FIXME: do sth with the goal
        ROS_INFO("Received a new goal.");

        
      }
  };
}

#endif
