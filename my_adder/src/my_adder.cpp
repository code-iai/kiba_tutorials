#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <deque>

class MyAdder
{
  public: 
    MyAdder(const ros::NodeHandle& nh) : nh_(nh) 
    {
      pub_ = nh_.advertise<std_msgs::Float64>("out", 1);
      sub_ = nh_.subscribe("in", 1, &MyAdder::callback, this);
    }
    ~MyAdder() {}

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::deque<double> buffer_;

    void callback(const std_msgs::Float64ConstPtr& msg)
    {
      publish_sum(msg->data);
      update_buffer(msg->data);
    }

    void publish_sum(double new_num)
    {
      if (buffer_.size() == 1)
      {
        std_msgs::Float64 result;
        for (size_t i=0; i<buffer_.size(); ++i)
        {
          result.data += buffer_[i];
        }
        result.data += new_num;
        pub_.publish(result);
      }
    }

    void update_buffer(double new_num)
    {
      if(buffer_.size() == 1)
      {
        buffer_.pop_front();
      }
      buffer_.push_back(new_num);
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_adder");
  ros::NodeHandle nh("~");
  MyAdder my_adder(nh);
  ros::spin();

  return 0;
} 
