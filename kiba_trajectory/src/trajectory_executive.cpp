#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kiba_trajectory/kiba_trajectory.hpp>
#include <kiba_msgs/MoveAction.h>
#include <kiba_msgs/GenerateTrajectory.h>

class TrajectoryExecutive
{
  public:
    TrajectoryExecutive(const ros::NodeHandle& nh, const std::string& name,
        const ros::Duration& timeout) : 
      nh_(nh), action_server_(nh_, name, boost::bind(&TrajectoryExecutive::callback, this, _1), false),
      timeout_(timeout)
    {
      pub_ = nh_.advertise<trajectory_msgs::JointTrajectoryPoint>("command", 1);
      service_client_ = nh_.serviceClient<kiba_msgs::GenerateTrajectory>("generate_trajectory");
    }

    ~TrajectoryExecutive() {}

    void start()
    {
      // make sure the service client came up
      if (!service_client_.waitForExistence(timeout_))
        throw std::runtime_error("Service 'generate_trajectory' does not exist.");

      action_server_.start();
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
    ros::Publisher pub_;
    actionlib::SimpleActionServer<kiba_msgs::MoveAction> action_server_;
    ros::Duration timeout_;

    void callback(const kiba_msgs::MoveGoalConstPtr &goal)
    {
      // prepare result and feedback messages
      kiba_msgs::MoveResult result;
      kiba_msgs::MoveFeedback feedback;

      // check whether we were asked to prepare meaningful spline
      if (goal->spline.start_time > goal->spline.end_time)
      {
        action_server_.setAborted(result, "Invalid spline with start_time > end_time.");
        return;
      }

      ROS_INFO("Asking for trajectory points");
      // ask for trajectory points
      kiba_msgs::GenerateTrajectory srv;
      srv.request.spline = goal->spline;
      srv.request.num_samples = goal->samples;
      if (!service_client_.call(srv))
      {
        action_server_.setAborted(result, "Call to 'generate_trajectory' failed.");
        return;
      }

      // periodically publish trajectory points
      ros::Rate r(((double) goal->samples)/(goal->spline.end_time - goal->spline.start_time).toSec());
      ROS_INFO("Periodically sleep for %fs.", r.expectedCycleTime().toSec());
      for (size_t i=0; i<goal->samples; ++i)
      {
        // always check whether the action client requested a cancel/preempt
        if (action_server_.isPreemptRequested() || !ros::ok())
        {
          action_server_.setPreempted();
          return;
        }

        // publish the trajectory point
        pub_.publish(srv.response.points[i]);

        // be nice and publish feedback
        feedback.progress = ((double) i) / ((double) goal->samples);
        action_server_.publishFeedback(feedback);

        // sleep to maintain constant publishing frequency
        r.sleep();
      }
      
      // prepare final result
      result.points = srv.response.points;
      action_server_.setSucceeded(result);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_executive");
  ros::NodeHandle nh("~");
  TrajectoryExecutive exec(nh, "move", ros::Duration(0.5));

  try
  {
    exec.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
