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
    path p_;
    std::vector<path> v_;
    int hertz_;

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

    //////////////////////////////////////////////////////////////////////////////// create file paths vector
    int 
      all_files ()
    {
      try
      {
        if (exists(p_))
        {
          if (is_directory(p_))
          {
            for (auto&& x : directory_iterator(p_))
              v_.push_back(x.path()); 

            std::sort(v_.begin(), v_.end());
            cout << p_ << " is a directory containing " << v_.size() << " files." ;
            return (0);
          }
          else
          {
            cout << p_ << " exists, but is not a regular file or directory\n";
            return (-1);
          }    
        }
        else
        {
          cout << p_ << " does not exist\n";
          return (-1);
        }
          
      }
      catch (const filesystem_error& ex)
      {
        cout << ex.what() << '\n';
        return (-1);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      int nr_points;
      string fields_list;
      int file_index = -1;
      ros::Rate loop_rate(hertz_);
      while (nh_.ok ())
      {
        if (file_index != (v_.size()-1))
        {
          file_index ++;
        }
        else
        {
          file_index = 0;
        }
        file_name_ = v_[file_index].string();
        pcl::io::loadPCDFile (file_name_, cloud_);
        cloud_.header.frame_id = tf_frame_;
        nr_points = cloud_.width * cloud_.height;
        fields_list = pcl::getFieldsList (cloud_);
        ROS_INFO ("Loaded a point cloud with %d points (total size is %zu) and the following channels: %s.",  nr_points, cloud_.data.size (), fields_list.c_str ());

        ROS_DEBUG_ONCE ("Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str (), nh_.resolveName (cloud_topic_).c_str (), cloud_.header.frame_id.c_str ());

        ROS_DEBUG ("Publishing data to %d subscribers.", pub_.getNumSubscribers ());
        cloud_.header.stamp = ros::Time::now ();
        pub_.publish (cloud_);
        loop_rate.sleep();
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
  c.p_ = argv[1];
  c.hertz_ = atoi (argv[2]);
  if (c.all_files () == -1)
  {
    ROS_ERROR ("Could not load file paths.");
    return (0);
  }
  c.spin ();
  return (0);
}
/* ]--- */

