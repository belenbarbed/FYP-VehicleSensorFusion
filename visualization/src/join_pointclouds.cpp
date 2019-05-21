#include <cstdlib>
#include <cstdio>
#include <iostream>

#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// #include <but_calibration_camera_velodyne/Image.h>
// #include <but_calibration_camera_velodyne/Velodyne.h>
// #include <car_detection/CarDetection.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <image_transport/subscriber_filter.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <pcl/common/eigen.h>
// #include <pcl/common/transforms.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
// #include <velodyne_pointcloud/point_types.h>

std::vector<std::string> VELODYNE_TOPICS_IN;
std::string VELODYNE_TOPIC_OUT;

ros::Publisher pub;

void callback(
  const sensor_msgs::PointCloud2ConstPtr& msg_pc_1,
  const sensor_msgs::PointCloud2ConstPtr& msg_pc_2,
  const sensor_msgs::PointCloud2ConstPtr& msg_pc_3,
  const sensor_msgs::PointCloud2ConstPtr& msg_pc_4
) {

  ROS_INFO_STREAM("in callback");

  sensor_msgs::PointCloud2 pc_1 = *msg_pc_1;
  sensor_msgs::PointCloud2 pc_2 = *msg_pc_2;
  sensor_msgs::PointCloud2 pc_3 = *msg_pc_3;
  sensor_msgs::PointCloud2 pc_4 = *msg_pc_4;

  sensor_msgs::PointCloud2 tmp_1;
  sensor_msgs::PointCloud2 tmp_2;
  sensor_msgs::PointCloud2 final;

  // if(pcl::concatenatePointCloud(pc_1, pc_2, tmp_1)) {
  //   if(pcl::concatenatePointCloud(pc_3, pc_4, tmp_2)) {
  //     if(pcl::concatenatePointCloud(tmp_1, tmp_2, final)) {
  //       pub.publish(final);
  //     }
  //   }
  // }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "join_pointclouds");
  ros::NodeHandle nh;
  nh.getParam("/visualization/pc_to_join", VELODYNE_TOPICS_IN);
  nh.getParam("/visualization/target_pc", VELODYNE_TOPIC_OUT);
  ROS_INFO_STREAM("node is running");

  // HACK: assume only 4 velodyne nodes in
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1(nh, VELODYNE_TOPICS_IN[0], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(nh, VELODYNE_TOPICS_IN[1], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_3(nh, VELODYNE_TOPICS_IN[2], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_4(nh, VELODYNE_TOPICS_IN[3], 10);

  pub = nh.advertise<sensor_msgs::PointCloud2>(VELODYNE_TOPIC_OUT, 1);

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2, sub_3, sub_4);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
