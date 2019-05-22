#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/PointCloud2.h>
// #include <car_detection/CarDetection.h>

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

#include <ctime>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

Velodyne::Velodyne pointcloud_1;
Velodyne::Velodyne pointcloud_2;
Velodyne::Velodyne pointcloud_3;
Velodyne::Velodyne pointcloud_4;

clock_t start_face;
double duration_face;
clock_t start_point;
double duration_point;

Mat projection_matrix;
vector<float> DoF_1 = {0.19082, -0.15932, -0.226174, -0.00032127, 0.000937961, 0.00166544};
vector<float> DoF_2 = {0.178896, -0.153617, -0.210116, -0.000114214, 0.00115482, -0.00148477};
vector<float> DoF_3 = {0.166122, -0.244738, -0.194233, -0.000416666, 0.00125, -0.00171875};
vector<float> DoF_4 = {0.191264, -0.272736, -0.167256, -0.000995302, -0.00139342, 1.10589e-05};

CascadeClassifier face_cascade;

void cameraInfo()
{
  float p[12];
  for (int i = 0; i < 12; i++)
  {
    p[i] = 0;
  }

  p[0] = 515.4;
  p[2] = 323.0;
  p[5] = 518.7;
  p[6] = 233.9;
  p[10] = 1.0;
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}

void detectAndDisplay(cv::Mat &frame, Velodyne::Velodyne &pointcloud, Mat projection_matrix, vector<float> DoF)
{
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  Velodyne::Velodyne transformed = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;
  start_face = clock();

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  duration_face = ( clock() - start_face ) / (double) CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Face duration: " << duration_face);

  start_point = clock();
  for( size_t i = 0; i < faces.size(); i++ )
  {
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    Point text_org(faces[i].x, faces[i].y + 4.0);
    rectangle(frame, faces[i], Scalar( 255, 0, 255 ), 1, 8, 0); 
    string distance = "distance: " + to_string(transformed.project(&frame, projection_matrix, Rect(0, 0, frame.cols, frame.rows), faces[i], &visible_points));
    putText(frame, distance, text_org, cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar( 118, 185, 0 ), 1, 8,false );
  }
  duration_point = ( clock() - start_point ) / (double) CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Point duration: " << duration_point);
}

void callback(const sensor_msgs::ImageConstPtr& msg_1, 
              const sensor_msgs::ImageConstPtr& msg_2,
              const sensor_msgs::ImageConstPtr& msg_3,
              const sensor_msgs::ImageConstPtr& msg_4,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
  cv::Mat frame_rgb_1;
  frame_rgb_1 = cv_bridge::toCvCopy(msg_1, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat frame_rgb_2;
  frame_rgb_2 = cv_bridge::toCvCopy(msg_2, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat frame_rgb_3;
  frame_rgb_3 = cv_bridge::toCvCopy(msg_3, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat frame_rgb_4;
  frame_rgb_4 = cv_bridge::toCvCopy(msg_4, sensor_msgs::image_encodings::BGR8)->image;
  ROS_INFO_STREAM("2");

  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg_pc, pc);
  ROS_INFO_STREAM("3");
  pointcloud_1 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);
  pointcloud_2 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, -M_PI / 2, 0);
  pointcloud_3 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI, 0);
  pointcloud_4 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI / 2, 0);
  ROS_INFO_STREAM("4");

  detectAndDisplay(frame_rgb_1, pointcloud_1, projection_matrix, DoF_1);
  ROS_INFO_STREAM("5");
  detectAndDisplay(frame_rgb_2, pointcloud_2, projection_matrix, DoF_2);
  detectAndDisplay(frame_rgb_3, pointcloud_3, projection_matrix, DoF_3);
  detectAndDisplay(frame_rgb_4, pointcloud_4, projection_matrix, DoF_4);
  ROS_INFO_STREAM("6");
  cv::imshow("view_1", frame_rgb_1);
  cv::imshow("view_2", frame_rgb_2);
  cv::imshow("view_3", frame_rgb_3);
  cv::imshow("view_4", frame_rgb_4);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  cameraInfo();

  cv::namedWindow("view_1");
  cv::namedWindow("view_2");
  cv::namedWindow("view_3");
  cv::namedWindow("view_4");

  //-- 1. Load the cascades
   if( !face_cascade.load("/home/soteris-group/bb2115/catkin_ws/src/face_detection/src/haarcascade_frontalface_alt.xml") ){ printf("--(!)Error loading\n"); return -1; };

  ros::init(argc, argv, "face_detection_cpp");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");
  image_transport::ImageTransport it(nh);

  image_transport::SubscriberFilter sub_1(it, "/pixel_1/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_2(it, "/pixel_2/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_3(it, "/pixel_3/camera0/image/not_compressed", 10);
  image_transport::SubscriberFilter sub_4(it, "/pixel_4/camera0/image/not_compressed", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc(nh, "/velodyne_points", 10);
  
  ROS_INFO_STREAM("1");
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2, sub_3, sub_4, sub_pc);
  ROS_INFO_STREAM("here");
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
  ROS_INFO_STREAM("here--");
  ros::spin();

  cv::destroyWindow("view_1");
  cv::destroyWindow("view_2");
  cv::destroyWindow("view_3");
  cv::destroyWindow("view_4");
}
