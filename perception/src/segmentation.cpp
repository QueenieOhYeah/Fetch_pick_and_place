#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {


Segmenter::Segmenter(const ros::Publisher& points_pub)
    : points_pub_(points_pub) {}



//void Segmenter::SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                                      std::vector<pcl::PointIndices> *indices) {

void Segmenter::SegmentBinObjects(PointCloudC::Ptr cloud,
                                      std::vector<pcl::PointIndices> *indices) {
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    // pcl::PointIndices inside_bin_indices;
    // Cloud2Indices(cloud, &inside_bin_indices);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    // euclid.setIndices(&inside_bin_indices); // << TODO: is cloud already cropped or do we get indices for the crop?
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < indices->size(); ++i) {
        // TODO: implement this
        size_t cluster_size = (*indices)[i].indices.size();
        max_size = std::max(max_size, cluster_size);
        min_size = std::min(min_size, cluster_size);
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
            indices->size(), min_size, max_size);
}
void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  std::vector<pcl::PointIndices> object_indices;
  SegmentBinObjects(cloud, &object_indices);

}
//void Segmenter::SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                                      std::vector<pcl::PointIndices> *indices) {
//  double cluster_tolerance;
//  int min_cluster_size, max_cluster_size;
//  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
//  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
//  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

//  pcl::EuclideanClusterExtraction<PointC> euclid;
////  for (PointCloudC::iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
////    float x, y, z, rgb;
////    x = it->x;
////    y = it->y;
////    z = it->z;
////    rgb = it->rgb;
////    if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) || !pcl_isfinite(rgb)) {
////      it = cloud->points.erase(it);
////    }
////  }
////    
//  euclid.setInputCloud(cloud);
//  // euclid.setIndices(inside_bin_indices); // << TODO: is cloud already cropped or do we get indices for the crop?
//  euclid.setClusterTolerance(cluster_tolerance);
//  euclid.setMinClusterSize(min_cluster_size);
//  euclid.setMaxClusterSize(max_cluster_size);
//  euclid.extract(*indices);

////   Find the size of the smallest and the largest object,
////   where size = number of points in the cluster
//  size_t min_size = std::numeric_limits<size_t>::max();
//  size_t max_size = std::numeric_limits<size_t>::min();
//  for (size_t i = 0; i < indices->size(); ++i) {
//    // TODO: implement this
//    PointCloudC::Ptr cloud_cluster (new PointCloudC());
////    for (const auto& idx : (*indices)[i].indices) {
////      cloud_cluster->push_back ((*cloud)[idx]);     
////    }
////     
////    
////    size_t cluster_size = cloud_cluster->size();
//    size_t cluster_size = (*indices)[i].indices.size();
//    min_size = std::min(min_size, cluster_size);
//    max_size = std::max(max_size, cluster_size);
//}

//ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
//         indices->size(), min_size, max_size);                       
//}
}  // namespace perception
