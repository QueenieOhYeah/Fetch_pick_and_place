#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"

namespace perception {

// Add function definitions here later

class Segmenter {
 public:
  Segmenter(const ros::Publisher& points_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices>* indices);

 private:
  ros::Publisher points_pub_;
};
}  // namespace perception
