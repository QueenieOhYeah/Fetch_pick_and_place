#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "perception/object.h"
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectList.h"
#include "perception_msgs/Object.h"
#include "perception_msgs/Target.h"

namespace perception {

// Add function definitions here later

class Segmenter_final {
 public:
/*  Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub);*/
/*  Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub, const ObjectRecognizer& recognizer);*/
  Segmenter_final(const ros::Publisher& points_pub, const ros::Publisher& markers_pub, const ros::Publisher& objects_pub, const ObjectRecognizer& recognizer);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices>* indices);
  void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);
  void Euclid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices);
  void RegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices);
  void ColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices);
  void UpdateTarget(const perception_msgs::Target& target);
  void CutPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud);
  
  
// Does a complete bin segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the bin and the objects in it.
//  objects: The output objects.
  void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects);
 private:
  ros::Publisher points_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher objects_pub_;
  ObjectRecognizer recognizer_;
  perception_msgs::Target target_;
  
};
}  // namespace perception
