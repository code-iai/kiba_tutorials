#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
static const std::string TOPIC_NAME = "/simple_pc_publisher/points";


void callback(const PointCloud::ConstPtr& msg)
{
  ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    ROS_INFO("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_pc_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>(TOPIC_NAME, 1, callback);
  ros::spin();
}
