#include <iostream>
#include <string>
#include "sensor_msgs/PointCloud2.h"

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"

void print_usage() { 
  std::cout << "Saves a point cloud on head_camera/depth_registered/points to "
               "NAME.bag in the current directory."
            << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

class TransformCloud {
 private:
  ros::Publisher cloud_pub_;
 public:
  TransformCloud(const ros::Publisher& points_pub);
  void Callback(const sensor_msgs::PointCloud2& cloud);

};

TransformCloud::TransformCloud(const ros::Publisher& cloud_pub):cloud_pub_(cloud_pub) {}

void TransformCloud::Callback(const sensor_msgs::PointCloud2& cloud) {
  tf::TransformListener tf_listener;                                                    
  tf_listener.waitForTransform("base_link", cloud.header.frame_id,                     
                               ros::Time(0), ros::Duration(5.0));                       
  tf::StampedTransform transform;                                                       
  try {                                                                                 
    tf_listener.lookupTransform("base_link", cloud.header.frame_id,                    
                                ros::Time(0), transform);                               
  } catch (tf::LookupException& e) {                                                    
      std::cerr << e.what() << std::endl;                                                 
                                                                          
  } catch (tf::ExtrapolationException& e) {                                             
      std::cerr << e.what() << std::endl;                                                 
                                                                          
  }
  sensor_msgs::PointCloud2 cloud_out;                                                   
  pcl_ros::transformPointCloud("base_link", transform, cloud, cloud_out);
  cloud_pub_.publish(cloud_out);
  
}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "transform_cloud_main");
  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("transform_point_cloud", 1);
  TransformCloud transformcloud(cloud_pub);
  ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/points", 1, &TransformCloud::Callback, &transformcloud);
  ros::spin();
  
  return 0;
}
      
