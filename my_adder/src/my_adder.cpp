#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <my_adder/my_adder.hpp>

class MyAdderRos
{
  public: 
    MyAdderRos(const ros::NodeHandle& nh) : nh_(nh) 
    {
      pub_ = nh_.advertise<std_msgs::Float64>("out", 1);
      sub_ = nh_.subscribe("in", 1, &MyAdderRos::callback, this);
    }
    ~MyAdderRos() {}

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    MyAdder my_adder_;

    void callback(const std_msgs::Float64ConstPtr& msg)
    {
      my_adder_.new_summand(msg->data);
      if (my_adder_.size() == 2)
      {
        std_msgs::Float64 result;
        result.data = my_adder_.sum();
        pub_.publish(result);
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_adder");
  ros::NodeHandle nh("~");
  MyAdderRos my_adder(nh);
  ros::spin();

  return 0;
} 
