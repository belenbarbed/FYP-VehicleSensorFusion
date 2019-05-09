#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL || argv[2] == NULL || argv[3] == NULL || argv[4] == NULL) return 1;
  
  ros::init(argc, argv, "video_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_1 = it.advertise("video_1", 1);
  image_transport::Publisher pub_2 = it.advertise("video_2", 1);
  image_transport::Publisher pub_3 = it.advertise("video_3", 1);
  image_transport::Publisher pub_4 = it.advertise("video_4", 1); 

  // transfer video
  cv::VideoCapture cap_1(argv[1]);
  cv::VideoCapture cap_2(argv[2]);
  cv::VideoCapture cap_3(argv[3]);
  cv::VideoCapture cap_4(argv[4]);

  cv::Mat frame_1;
  cv::Mat frame_2;
  cv::Mat frame_3;
  cv::Mat frame_4;

  cv_bridge::CvImage cv_img_1;
  cv_bridge::CvImage cv_img_2;
  cv_bridge::CvImage cv_img_3;
  cv_bridge::CvImage cv_img_4;

  sensor_msgs::ImagePtr msg_1;
  sensor_msgs::ImagePtr msg_2;
  sensor_msgs::ImagePtr msg_3;
  sensor_msgs::ImagePtr msg_4;

  ros::Rate loop_rate(25);

  int sync_1 = 0;
  int sync_2 = 0;
  int sync_3 = 0;
  while (nh.ok()) {
    cap_1 >> frame_1;
    cap_2 >> frame_2;
    cap_3 >> frame_3;
    cap_4 >> frame_4;

    cv_img_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_1);
    cv_img_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_2);
    cv_img_3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_3);
    cv_img_4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_4);

/*
    msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_1).toImageMsg();
    msg_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_2).toImageMsg();
    msg_3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_3).toImageMsg();
    msg_4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_4).toImageMsg();
*/
    cv_img_1.header.stamp = ros::Time::now ();
    msg_1 = cv_img_1.toImageMsg();
    pub_1.publish(msg_1);

    cv_img_2.header.stamp = ros::Time::now ();
    msg_2 = cv_img_1.toImageMsg();
    pub_2.publish(msg_2);

    cv_img_3.header.stamp = ros::Time::now ();
    msg_3 = cv_img_1.toImageMsg();
    pub_3.publish(msg_3);

    cv_img_4.header.stamp = ros::Time::now ();
    msg_4 = cv_img_1.toImageMsg();
    pub_4.publish(msg_4);
    
    cv::waitKey(1);
    loop_rate.sleep();
  }
}
