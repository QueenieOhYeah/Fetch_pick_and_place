#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"
#include "perception_msgs/ObjectList.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  
  if (argc < 2) {
  ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
  ros::spinOnce();
  }
  std::string data_dir(argv[1]);
  
  ros::NodeHandle nh;
  ros::Publisher crop_organized_pub =
     nh.advertise<sensor_msgs::PointCloud2>("cropped_organized_cloud", 1, true);
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub, crop_organized_pub);
  ros::Subscriber crop_sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  perception::Downsampler downsampler(downsample_pub);
  ros::Subscriber downsample_sub =
      nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
  
  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);
  
  ros::Publisher segment_pub =
      nh.advertise<sensor_msgs::PointCloud2>("segment_cloud", 1, true);
  ros::Publisher marker_pub = 
      nh.advertise<visualization_msgs::Marker>("segment_marker", 1, true);    
  ros::Publisher object_pub = 
      nh.advertise<perception_msgs::ObjectList>("object_list", 1, true);     
  
//  perception::Segmenter segmenter(segment_pub, marker_pub);
  perception::Segmenter segmenter(segment_pub, marker_pub, object_pub, recognizer);
  
  ros::Subscriber segment_sub =
//  nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
      nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  

  ros::spin(); 
  return 0;
}
