#ifndef KIBA_TUTORIALS_CONVERSIONS_HPP
#define KIBA_TUTORIALS_CONVERSIONS_HPP

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kiba_msgs/CubicSpline.h>
#include <kiba_trajectory/datatypes.hpp>
#include <exception>

namespace kiba_trajectory
{
  trajectory_msgs::JointTrajectoryPoint to_msg(const SingleDOFTrajectoryPoint& point)
  {
    trajectory_msgs::JointTrajectoryPoint result;
    result.positions.push_back(point.position);
    result.velocities.push_back(point.velocity);
    result.accelerations.push_back(point.acceleration);
    result.time_from_start = ros::Duration(point.time);
    return result;
  }

  SingleDOFTrajectoryPoint from_msg(const trajectory_msgs::JointTrajectoryPoint& msg)
  {
    if (msg.positions.size() == 0)
      throw std::runtime_error("Trajectory point has no positions.");
    if (msg.velocities.size() == 0)
      throw std::runtime_error("Trajectory point has no velocities.");
    if (msg.accelerations.size() == 0)
      throw std::runtime_error("Trajectory point has no accelerations.");

    SingleDOFTrajectoryPoint result;
    result.position = msg.positions[0];
    result.velocity = msg.velocities[0];
    result.acceleration = msg.accelerations[0];
    result.time = msg.time_from_start.toSec();
    return result;
  }

  kiba_msgs::CubicSpline to_msg(const SingleDOFCubicSpline& spline)
  {
    kiba_msgs::CubicSpline result;
    result.start_time = ros::Time(spline.start_time);
    result.end_time = ros::Time(spline.end_time);
    result.start_pos = spline.start_position;
    result.end_pos = spline.end_position;
    result.start_vel = spline.start_velocity;
    result.end_vel = spline.end_velocity;
    return result;
  }

  SingleDOFCubicSpline from_msg(const kiba_msgs::CubicSpline& msg)
  {
    SingleDOFCubicSpline result;
    result.start_time = msg.start_time.toSec();
    result.end_time = msg.end_time.toSec();
    result.start_position = msg.start_pos;
    result.end_position = msg.end_pos;
    result.start_velocity = msg.start_vel;
    result.end_velocity = msg.end_vel;
    return result;
  }
}

#endif
