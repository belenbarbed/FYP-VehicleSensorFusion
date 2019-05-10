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
vector<float> DoF = {0.05, 0, 0, 0, -0.2, 0.5};

void cameraInfo()
{
  float p[12];
  for (int i = 0; i < 12; i++)
  {
    p[i] = 0;
  }

  p[0] = 4.0;
  p[5] = 4.0;
  p[10] = 1.0;
  ROS_INFO_STREAM(p[1]);
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}

void callback(const sensor_msgs::ImageConstPtr& msg_1, 
              const sensor_msgs::ImageConstPtr& msg_2,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
  cv::Mat frame_rgb;
  
  //ROS_INFO_STREAM( "Video_1 received at " << msg_1->header.stamp.toSec());
  //ROS_INFO_STREAM( "Video_2 received at " << msg_2->header.stamp.toSec());
  //ROS_INFO_STREAM( "Video_3 received at " << msg_3->header.stamp.toSec());
  //ROS_INFO_STREAM( "Video_4 received at " << msg_4->header.stamp.toSec());
  //ROS_INFO_STREAM( "Msg_Pointcloud2 received at " << msg_pc->header.stamp.toSec());

  frame_rgb = cv_bridge::toCvCopy(msg_1, sensor_msgs::image_encodings::BGR8)->image;

  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg_pc, pc);
  
  // x := x, y := -z, z := y,
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

  Image::Image img(frame_rgb);
  Velodyne::Velodyne transformed = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;
  transformed.project(&frame_rgb, projection_matrix, Rect(0, 0, frame_rgb.cols, frame_rgb.rows), &visible_points);

  cv::imshow("view_1", frame_rgb);
  cv::imshow("view_2", cv_bridge::toCvCopy(msg_2, "bgr8")->image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  cameraInfo();

  cv::namedWindow("view_1");
  cv::namedWindow("view_2");

  ros::init(argc, argv, "video_subscriber");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter sub_1(it, "/pixel_1/camera0/image/raw", 1);
  // image_transport::SubscriberFilter sub_2(it, "/pixel_2/camera0/image/raw", 1);
  // image_transport::SubscriberFilter sub_3(it, "/pixel_3/camera0/image/raw", 1);
  // image_transport::SubscriberFilter sub_4(it, "/pixel_4/camera0/image/raw", 1);
  image_transport::SubscriberFilter sub_2(it, "/phone_5/camera0/image/raw", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc(nh, "/velodyne_points", 1);
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2, sub_pc);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  ros::spin();

  cv::destroyWindow("view_1");
  cv::destroyWindow("view_2");
}
