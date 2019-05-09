// STL
#include <chrono>
#include <thread>

// ROS core
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/publisher.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
using std::cout;
using namespace boost::filesystem;

using namespace std;

class PCDGenerator
{
  protected:
    string tf_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
  public:

    // ROS messages
    sensor_msgs::PointCloud2 cloud_;

    string file_name_, cloud_topic_;
    double wait_;

    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;

    ////////////////////////////////////////////////////////////////////////////////
    PCDGenerator () : tf_frame_ ("/base_link"), private_nh_("~")
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1

      cloud_topic_ = "cloud_pcd";
      pub_.advertise (nh_, cloud_topic_.c_str (), 1);
      private_nh_.param("frame_id", tf_frame_, std::string("/base_link"));
      ROS_INFO ("Publishing data on topic %s with frame_id %s.", nh_.resolveName (cloud_topic_).c_str (), tf_frame_.c_str());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      if (file_name_ == "" || pcl::io::loadPCDFile (file_name_, cloud_) == -1)
        return (-1);
      cloud_.header.frame_id = tf_frame_;
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      int nr_points      = cloud_.width * cloud_.height;
      string fields_list = pcl::getFieldsList (cloud_);
      double interval = wait_ * 1e+6;
      if (nh_.ok ())
      {
        ROS_DEBUG_ONCE ("Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str (), nh_.resolveName (cloud_topic_).c_str (), cloud_.header.frame_id.c_str ());
        cloud_.header.stamp = ros::Time::now ();

        ROS_DEBUG ("Publishing data to %d subscribers.", pub_.getNumSubscribers ());
        pub_.publish (cloud_);

        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<uint32_t>(interval)));
      }
      return (true);
    }


};

/* ---[ */
int
  main (int argc, char** argv)
{ 

  ros::init (argc, argv, "pointcloud_publisher");

  PCDGenerator c;
  path p (argv[1]);
  c.wait_ = atof (argv[2]);

  try
  {
    if (exists(p))
    {
      if (is_directory(p))
      {
        cout << p << " is a directory containing:\n";

        std::vector<path> v;

        for (auto&& x : directory_iterator(p))
          v.push_back(x.path()); 

        std::sort(v.begin(), v.end());  

        for (auto&& x : v)
        {
          c.file_name_ = x.string();
          if (c.start () == -1)
          {
            ROS_ERROR ("Could not load file %s. Exiting.", argv[1]);
            return (-1);
          }
          ROS_INFO ("Loaded a point cloud with %d points (total size is %zu) and the following channels: %s.",  c.cloud_.width * c.cloud_.height, c.cloud_.data.size (), pcl::getFieldsList (c.cloud_).c_str ());
          c.spin ();
        }
      }
      else
        cout << p << " exists, but is not a regular file or directory\n";
    }
    else
      cout << p << " does not exist\n";
  }

  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

  return (0);
}
/* ]--- */

