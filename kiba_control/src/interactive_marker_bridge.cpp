#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <boost/bind.hpp>
#include <kiba_control/utils.hpp>


class InteractiveMarkerBridge
{
  public:
    InteractiveMarkerBridge(const ros::NodeHandle& nh) : 
      nh_( nh ), pub_( nh_.advertise<geometry_msgs::PoseStamped>("goal", 1) ),
      marker_name_( kiba_control::readParam<std::string>(nh_, "marker_name") ),
      frame_id_( kiba_control::readParam<std::string>(nh_, "frame_id") ),
      marker_scale_( kiba_control::readParam<double>(nh_, "marker_scale") )
    {
    }

    void start()
    {
      server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>
          (nh_.getNamespace(), "", false);

      visualization_msgs::InteractiveMarker marker = create_6dof_marker();
      server_->insert(marker);
      server_->setCallback(marker.name,
          boost::bind(&InteractiveMarkerBridge::feedback_callback, this, _1));

      server_->applyChanges();
    }
  
    ~InteractiveMarkerBridge() {}

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::string marker_name_, frame_id_;
    double marker_scale_;

    void feedback_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
      if (feedback->event_type ==
          visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
      {
        geometry_msgs::Pose identity_pose;
        identity_pose.orientation.w = 1.0;
        server_->setPose( feedback->marker_name, identity_pose); 
        server_->applyChanges();

        if(feedback->marker_name.compare(marker_name_) == 0)
        {
          geometry_msgs::PoseStamped goal;
          goal.header = feedback->header;
          goal.pose = feedback->pose;
          pub_.publish(goal);
        }
        else
        {
          ROS_ERROR("Received marker with unknown name '%s'.", feedback->marker_name.c_str());
          return;
        }
      }
    }

    visualization_msgs::InteractiveMarker create_6dof_marker()
    {
      visualization_msgs::InteractiveMarker result;
      result.name = marker_name_;
      result.description = marker_name_;
      result.header.frame_id = frame_id_;
      result.scale = marker_scale_;

      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);

      return result;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_bridge");
  ros::NodeHandle nh("~");

  try
  {
    InteractiveMarkerBridge imb(nh);
    imb.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
