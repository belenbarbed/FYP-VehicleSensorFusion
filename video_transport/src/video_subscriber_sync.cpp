#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

Mat projection_matrix;
vector<float> DoF = {0.19082, -0.15932, -0.226174, -0.00032127, 0.000937961, 0.00166544};

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg->P.begin(); i != msg->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;

  }

  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}

void callback(const sensor_msgs::ImageConstPtr& msg_1,
              const sensor_msgs::ImageConstPtr& msg_2,
              const sensor_msgs::ImageConstPtr& msg_3,
              const sensor_msgs::ImageConstPtr& msg_4,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
  cv::Mat frame_rgb_1;
  cv::Mat frame_rgb_2;
  cv::Mat frame_rgb_3;
  cv::Mat frame_rgb_4;
  
  ROS_INFO_STREAM( "Video_1 received at " << msg_1->header.stamp.toNSec());
  ROS_INFO_STREAM( "Video_2 received at " << msg_2->header.stamp.toNSec());
  ROS_INFO_STREAM( "Video_3 received at " << msg_3->header.stamp.toNSec());
  ROS_INFO_STREAM( "Video_4 received at " << msg_4->header.stamp.toNSec());
  ROS_INFO_STREAM( "Msg_Pointcloud2 received at " << msg_pc->header.stamp.toNSec());

  frame_rgb_1 = cv_bridge::toCvCopy(msg_1, sensor_msgs::image_encodings::BGR8)->image;
  frame_rgb_2 = cv_bridge::toCvCopy(msg_2, sensor_msgs::image_encodings::BGR8)->image;
  frame_rgb_3 = cv_bridge::toCvCopy(msg_3, sensor_msgs::image_encodings::BGR8)->image;
  frame_rgb_4 = cv_bridge::toCvCopy(msg_4, sensor_msgs::image_encodings::BGR8)->image;
  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg_pc, pc);
  
  // x := x, y := -z, z := y,
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

  Velodyne::Velodyne transformed_1 = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;
  transformed_1.project(&frame_rgb_1, projection_matrix, Rect(0, 0, frame_rgb_1.cols, frame_rgb_1.rows), &visible_points);

  Velodyne::Velodyne transformed_2 = pointcloud.transform(DoF);
  transformed_2.project(&frame_rgb_2, projection_matrix, Rect(0, 0, frame_rgb_2.cols, frame_rgb_2.rows), &visible_points);

  Velodyne::Velodyne transformed_3 = pointcloud.transform(DoF);
  transformed_3.project(&frame_rgb_3, projection_matrix, Rect(0, 0, frame_rgb_3.cols, frame_rgb_3.rows), &visible_points);

  Velodyne::Velodyne transformed_4 = pointcloud.transform(DoF);
  transformed_4.project(&frame_rgb_4, projection_matrix, Rect(0, 0, frame_rgb_4.cols, frame_rgb_4.rows), &visible_points);

  cv::imshow("view_1", frame_rgb_1);
  cv::imshow("view_2", frame_rgb_2);
  cv::imshow("view_3", frame_rgb_3);
  cv::imshow("view_4", frame_rgb_4);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  cv::namedWindow("view_1");
  cv::namedWindow("view_2");
  cv::namedWindow("view_3");
  cv::namedWindow("view_4");

  ros::init(argc, argv, "video_subscriber");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter sub_1(it, "/pixel_1/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_2(it, "/pixel_2/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_3(it, "/pixel_3/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_4(it, "/pixel_4/camera0/image/not_compressed", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc(nh, "/velodyne_points", 10);

  ros::Subscriber info_sub = nh.subscribe("/pixel_1/camera0/camera_info", 10, cameraInfoCallback);

  // nh.getParam("/video_transport/6DoF", DoF);
  // ROS_INFO_STREAM("DoF array: " << DoF.size());
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2, sub_3, sub_4, sub_pc);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
  ros::spin();

  cv::destroyWindow("view_1");
  cv::destroyWindow("view_2");
  cv::destroyWindow("view_3");
  cv::destroyWindow("view_4");
}
