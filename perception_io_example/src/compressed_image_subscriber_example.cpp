#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string IMAGE_TOPIC_IN = "/usb_cam/image_raw";

class ImageSubscriber
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::TransportHints hints;
  image_transport::SubscriberFilter *image_sub_;
  image_transport::Publisher image_pub_;



public:
  ImageSubscriber()
    : it_(nh_), hints("compressed")
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_  = new image_transport::SubscriberFilter(it_, IMAGE_TOPIC_IN, 1, hints);
    image_sub_->registerCallback(boost::bind(&ImageSubscriber::imageCb, this, _1));
    ROS_INFO("Subscribed to image topic");
    image_pub_ = it_.advertise("/image_subscriber_example/output_image", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageSubscriber()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    cv::circle(cv_ptr->image, 
	       cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 
	       10, 
	       CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageSubscriber ic;
  ros::spin();
  return 0;
}
