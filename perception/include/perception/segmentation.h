#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace perception {

// Add function definitions here later

class Segmenter {
 public:
  Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub);
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

 private:
  ros::Publisher points_pub_;
  ros::Publisher markers_pub_;
};
}  // namespace perception