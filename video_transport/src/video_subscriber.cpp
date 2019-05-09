#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback_1(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view_1", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback_2(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view_2", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback_3(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view_3", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback_4(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view_4", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_subscriber");
  ros::NodeHandle nh;
  cv::namedWindow("view_1");
  cv::namedWindow("view_2");
  cv::namedWindow("view_3");
  cv::namedWindow("view_4");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_1 = it.subscribe("video_1", 10, imageCallback_1);
  image_transport::ImageTransport it_2(nh);
  image_transport::Subscriber sub_2 = it.subscribe("video_2", 10, imageCallback_2);
  image_transport::ImageTransport it_3(nh);
  image_transport::Subscriber sub_3 = it.subscribe("video_3", 10, imageCallback_3);
  image_transport::ImageTransport it_4(nh);
  image_transport::Subscriber sub_4 = it.subscribe("video_4", 10, imageCallback_4);
  ros::spin();
  cv::destroyWindow("view_1");
  cv::destroyWindow("view_2");
  cv::destroyWindow("view_3");
  cv::destroyWindow("view_4");
}
