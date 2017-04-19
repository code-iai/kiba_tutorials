#include <ros/ros.h>
#include <kiba_msgs/GenerateTrajectory.h>
#include <kiba_trajectory/kiba_trajectory.hpp>

bool callback(kiba_msgs::GenerateTrajectory::Request& req,
              kiba_msgs::GenerateTrajectory::Response& res)
{
  using namespace kiba_trajectory;
  SingleDOFCubicSpline spline = from_msg(req.spline);
  try
  {
    if (req.num_samples < 2)
      throw std::runtime_error("Trajectory with less than 2 points requested.");
    double time_inc = (spline.end_time - spline.start_time) / 
      ((double) req.num_samples - 1);

    for (size_t i=0; i<req.num_samples; ++i)
    {
      double sample_time = spline.start_time + ((double) i) * time_inc;
      res.points.push_back(to_msg(spline.sample(sample_time)));
      res.points[i].time_from_start -= ros::Duration(spline.start_time);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh("~");
  ros::ServiceServer service = 
    nh.advertiseService("generate_trajectory", callback);
  ros::spin();
  return 0;
}
