#ifndef KIBA_CONTROL_KINEMATICS_PLANNER_H
#define KIBA_CONTROL_KINEMATICS_PLANNER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kiba_control/utils.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl_conversions/kdl_msg.h>

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
        chain_( extract_chain(nh_, robot_model_, root_frame_name_, tip_frame_name_) ),
        tf_listener_( tf_buffer_ )
      {
        if (!action_client_.waitForServer(server_timeout))
          throw std::runtime_error("Waited for server " + std::to_string(server_timeout.toSec()) + "s. Aborting.");
        current_joint_state_.clear();
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
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;
      std::map<std::string, double> current_joint_state_;

      void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        current_joint_state_.clear();
        for (size_t i=0; i<msg->name.size(); ++i)
          current_joint_state_.insert(std::pair<std::string, double>(msg->name[i], msg->position[i]));
      }

      void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
      {
        try
        {
          action_client_.cancelAllGoals();
          action_client_.sendGoal(calc_trajectory_goal(*msg));
        }
        catch (const std::exception& e)
        {
          ROS_ERROR("%s", e.what());
        }
      }

      control_msgs::FollowJointTrajectoryGoal calc_trajectory_goal(
          const geometry_msgs::PoseStamped& goal_pose) const
      {
        control_msgs::FollowJointTrajectoryGoal result;
        result.trajectory.header = goal_pose.header;
        result.trajectory.joint_names = get_joint_names();
        result.trajectory.points.push_back(calc_start_trajectory_point());
        result.trajectory.points.push_back(calc_end_trajectory_point(goal_pose));
        return result;
      }

      trajectory_msgs::JointTrajectoryPoint calc_start_trajectory_point() const
      {
        trajectory_msgs::JointTrajectoryPoint result;
        result.time_from_start = ros::Duration(0.1);
        for (std::map<std::string, double>::const_iterator it=current_joint_state_.begin();
             it!=current_joint_state_.end(); ++it)
          result.positions.push_back(it->second);
 
        return result;
      }

      trajectory_msgs::JointTrajectoryPoint calc_end_trajectory_point(
          const geometry_msgs::PoseStamped& goal_pose) const
      {
        trajectory_msgs::JointTrajectoryPoint result;
        result.time_from_start = ros::Duration(2.0);
       
        // set up all the KDL solvers that we need
        KDL::ChainFkSolverPos_recursive fk_solver_pos(chain_);
        KDL::ChainIkSolverVel_wdls ik_solver_vel(chain_);
        KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain_, get_lower_limits(), get_upper_limits(), 
            fk_solver_pos, ik_solver_vel);

        // convert the current joint state into a KDL-compatible order and format
        std::vector<std::string> kdl_joint_names = get_kdl_joint_names();
        KDL::JntArray q_in(current_joint_state_.size());
        for (size_t i=0; i<current_joint_state_.size(); ++i)
          q_in(i) = current_joint_state_.find(kdl_joint_names[i])->second;

        // transform out goal point into the base of the robot using TF
        geometry_msgs::PoseStamped transformed_goal_pose =
          tf_buffer_.transform<geometry_msgs::PoseStamped>(goal_pose, root_frame_name_);

        // convert goal pose into KDL::Frame because the solver needs it like that
        KDL::Frame goal_frame;
        tf::poseMsgToKDL(transformed_goal_pose.pose, goal_frame);

        // solve for the actual position-resolved IK problem
        KDL::JntArray q_goal = q_in;
        if (ik_solver_pos.CartToJnt(q_in , goal_frame, q_goal) < 0)
          throw std::runtime_error("IK Solver did not find a solution.");

        // align the computed desired joint state into the required order using a std::map, and fill the message
        std::map<std::string, double> desired_joint_state;
        for (size_t i=0; i<kdl_joint_names.size(); ++i)
          desired_joint_state.insert(std::pair<std::string, double>(kdl_joint_names[i], q_goal(i)));
        for (std::map<std::string, double>::const_iterator it=desired_joint_state.begin();
             it!=desired_joint_state.end(); ++it)
          result.positions.push_back(it->second);

        return result;
      }

      std::vector<std::string> get_joint_names() const
      {
        std::vector<std::string> result;
        for (std::map<std::string, double>::const_iterator it=current_joint_state_.begin();
             it != current_joint_state_.end(); ++it)
          result.push_back(it->first);
        return result;
      }

      std::vector<std::string> get_kdl_joint_names() const
      {
        std::vector<std::string> result;
        for (size_t i=0; i<chain_.getNrOfSegments(); ++i)
          if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
            result.push_back(chain_.getSegment(i).getJoint().getName());

        return result;
      }

      KDL::JntArray get_lower_limits() const
      {
        KDL::JntArray result(chain_.getNrOfJoints());
        for (size_t i=0; i<chain_.getNrOfSegments(); ++i)
          if (is_urdf_joint_with_limits(robot_model_.getJoint(chain_.getSegment(i).getJoint().getName())))
              result(i) = robot_model_.getJoint(chain_.getSegment(i).getJoint().getName())->limits->lower;
          else
            result(i) = -10e10; // something with a big magnitude, FIXME: make this a constant with a name

        return result;
      }

      KDL::JntArray get_upper_limits() const
      {
        KDL::JntArray result(chain_.getNrOfJoints());
        for (size_t i=0; i<chain_.getNrOfSegments(); ++i)
           if (is_urdf_joint_with_limits(robot_model_.getJoint(chain_.getSegment(i).getJoint().getName())))
              result(i) = robot_model_.getJoint(chain_.getSegment(i).getJoint().getName())->limits->upper;
          else
            result(i) = 10e10; // something with a big magnitude, FIXME: make this a constant with a name

        return result;
      }

      bool is_urdf_joint_with_limits(const urdf::JointConstSharedPtr& joint) const
      {
        return joint->type == urdf::Joint::REVOLUTE ||
            joint->type == urdf::Joint::PRISMATIC;
      }
  };
}

#endif
