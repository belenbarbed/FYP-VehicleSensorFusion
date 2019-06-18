#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <face_detection/Detected_Img.h>
#include <face_detection/Bbox.h>

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

cv::Mat projection_matrix;

PointCloud<Velodyne::Point> pc;

std::vector<float> DoF_1 = {0.19082, -0.15932, -0.226174, -0.00032127, 0.000937961, 0.00166544};
std::vector<float> DoF_2 = {0.178896, -0.153617, -0.210116, -0.000114214, 0.00115482, -0.00148477};
std::vector<float> DoF_3 = {0.166122, -0.244738, -0.194233, -0.000416666, 0.00125, -0.00171875};
std::vector<float> DoF_4 = {0.191264, -0.272736, -0.167256, -0.000995302, -0.00139342, 1.10589e-05};

void find_distance(
  cv::Mat &frame,
  std::vector<face_detection::Bbox> bboxes,
  Velodyne::Velodyne &pointcloud,
  std::vector<float> DoF)
{

  Velodyne::Velodyne transformed = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;

  for(int i = 0; i < bboxes.size(); i++) {
    int top = bboxes[i].top;
    int right = bboxes[i].right;
    int bottom = bboxes[i].bottom;
    int left = bboxes[i].left;

    cv::Point center((left + right)/2, (top + bottom)/2);
    cv::rectangle(frame,
      cv::Point(left, top),
      cv::Point(right, bottom),
      Scalar(255, 0, 0), 1, 8, 0);
    std::string distance = "distance: " +
      to_string(transformed.project(&frame, projection_matrix,
        Rect(0, 0, frame.cols, frame.rows),
        Rect(left, top, right - left, top - bottom), &visible_points))
      + "m";
    cv::putText(frame,
      distance,
      cv::Point(left + 6, bottom - 6),
      cv::FONT_HERSHEY_DUPLEX, 1.0,
      Scalar(255, 255, 255), 1, 8, false);
  }
}

void image_callback_1(const face_detection::Detected_Img::ConstPtr& msg) {

  ROS_INFO_STREAM("In image 1 callback");

  face_detection::Detected_Img det = *msg;
  cv::Mat frame_rgb;
  frame_rgb = cv_bridge::toCvCopy(det.img, sensor_msgs::image_encodings::BGR8)->image;
  
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);
  
  find_distance(frame_rgb, det.bboxes, pointcloud, DoF_1);

  cv::imshow("Video_1", frame_rgb);

  cv::waitKey(1);
}

void image_callback_2(const face_detection::Detected_Img::ConstPtr& msg) {

  ROS_INFO_STREAM("In image 2 callback");

  face_detection::Detected_Img det = *msg;
  cv::Mat frame_rgb;
  frame_rgb = cv_bridge::toCvCopy(det.img, sensor_msgs::image_encodings::BGR8)->image;
  
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, -M_PI / 2, 0);

  find_distance(frame_rgb, det.bboxes, pointcloud, DoF_2);

  cv::imshow("Video_2", frame_rgb);

  cv::waitKey(1);
}

void image_callback_3(const face_detection::Detected_Img::ConstPtr& msg) {

  ROS_INFO_STREAM("In image 3 callback");

  face_detection::Detected_Img det = *msg;
  cv::Mat frame_rgb;
  frame_rgb = cv_bridge::toCvCopy(det.img, sensor_msgs::image_encodings::BGR8)->image;
  
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI, 0);

  find_distance(frame_rgb, det.bboxes, pointcloud, DoF_3);

  cv::imshow("Video_3", frame_rgb);

  cv::waitKey(1);
}

void image_callback_4(const face_detection::Detected_Img::ConstPtr& msg) {

  ROS_INFO_STREAM("In image 4 callback");

  face_detection::Detected_Img det = *msg;
  cv::Mat frame_rgb;
  frame_rgb = cv_bridge::toCvCopy(det.img, sensor_msgs::image_encodings::BGR8)->image;
  
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI / 2, 0);

  find_distance(frame_rgb, det.bboxes, pointcloud, DoF_4);

  cv::imshow("Video_4", frame_rgb);

  cv::waitKey(1);
}

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg_pc) {
  ROS_INFO_STREAM("In pc callback");
  fromROSMsg(*msg_pc, pc);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "video_visualization");
  ROS_INFO_STREAM("Initialising node...");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  float p_matrix[12] = {515.4, 0.0, 323.0, 0.0, 0.0, 518.7, 233.9, 0.0, 0.0, 0.0, 1.0, 0.0};
  projection_matrix = cv::Mat(3, 4, CV_32FC1, p_matrix);

  cv::namedWindow("Video_1");
  cv::namedWindow("Video_2");
  cv::namedWindow("Video_3");
  cv::namedWindow("Video_4");

  ros::Subscriber sub_1  = nh.subscribe("/pixel_1/camera0/image/detected", 1, image_callback_1);
  ros::Subscriber sub_2  = nh.subscribe("/pixel_2/camera0/image/detected", 1, image_callback_2);
  ros::Subscriber sub_3  = nh.subscribe("/pixel_3/camera0/image/detected", 1, image_callback_3);
  ros::Subscriber sub_4  = nh.subscribe("/pixel_4/camera0/image/detected", 1, image_callback_4);
  ros::Subscriber sub_pc = nh.subscribe("/velodyne_points", 1, pc_callback);

  ros::spin();

  cv::destroyWindow("Video_1");
  cv::destroyWindow("Video_2");
  cv::destroyWindow("Video_3");
  cv::destroyWindow("Video_4");

  ROS_INFO_STREAM("Finished video_visualization");
}
