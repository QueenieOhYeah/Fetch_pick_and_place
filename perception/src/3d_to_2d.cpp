//#include "perception/crop.h"
//#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
//#include "perception/segmentation.h"
//#include "visualization_msgs/Marker.h"
//#include <vector>
//#include "perception/object_recognizer.h"
//#include "perception_msgs/ObjectFeatures.h"
//#include "perception_msgs/ObjectList.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/common.h>


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());
  PointC minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  std::cout << "Min x: " << minPt.x << "\t\tMax x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << "\t\tMax y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << "\t\tMax z: " << maxPt.z << std::endl;
  
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "projection_to_2dgedit ");
  
//  if (argc < 2) {
//  ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
//  ros::spinOnce();
//  }
//  std::string data_dir(argv[1]);
  
  ros::NodeHandle nh;
  ros::Subscriber crop_sub =
      nh.subscribe("mock_point_cloud", 1, callback);


  ros::spin(); 
  return 0;
}
