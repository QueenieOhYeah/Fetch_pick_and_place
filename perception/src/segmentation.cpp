#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/extract_indices.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include <pcl/common/common.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {


Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub)
    : points_pub_(points_pub), markers_pub_(markers_pub) {}



//void Segmenter::SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                                      std::vector<pcl::PointIndices> *indices) {

void Segmenter::SegmentBinObjects(PointCloudC::Ptr cloud,
                                      std::vector<pcl::PointIndices>* indices) {
    
    Euclid(cloud, indices);
    

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    // object cloud
    PointCloudC::Ptr cloud_cluster (new PointCloudC());
    for (size_t i = 0; i < indices->size(); ++i) {
        // TODO: implement this
        size_t cluster_size = (*indices)[i].indices.size();
        max_size = std::max(max_size, cluster_size);
        min_size = std::min(min_size, cluster_size);
        
        PointCloudC::Ptr subset_cloud(new PointCloudC);
        pcl::PointIndices::Ptr indice(new pcl::PointIndices);
        *indice = (*indices)[i];
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indice);
        extract.filter(*subset_cloud);
        cloud_cluster->insert(std::end(*cloud_cluster), std::begin(*subset_cloud), std::end(*subset_cloud));
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
            indices->size(), min_size, max_size);
    
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    points_pub_.publish(msg_out);

}

void Segmenter::GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  pose->position.x = (max_pt.x() + min_pt.x()) / 2;
  pose->position.y = (max_pt.y() + min_pt.y()) / 2;
  pose->position.z = (max_pt.z() + min_pt.z()) / 2;
  pose->orientation.w = 1;
  dimensions->x = max_pt.x() - min_pt.x();
  dimensions->y = max_pt.y() - min_pt.y();
  dimensions->z = max_pt.z() - min_pt.z();
                               
}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  std::vector<pcl::PointIndices> object_indices;
  PointCloudC::Ptr filtered_cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, index);
  SegmentBinObjects(filtered_cloud, &object_indices);
  PointCloudC::Ptr cloud_cluster (new PointCloudC());

  
  
  for (size_t i = 0; i < object_indices.size(); ++i) {
    //// Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(indices);
    extract.filter(*object_cloud);
    cloud_cluster->insert(std::end(*cloud_cluster), std::begin(*object_cloud), std::end(*object_cloud));    


    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                            &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    ros::Duration t(0.1);
    object_marker.lifetime = t;
    markers_pub_.publish(object_marker);

//  sensor_msgs::PointCloud2 msg_out;
//  pcl::toROSMsg(*cloud_cluster, msg_out);
//  points_pub_.publish(msg_out);
}

}

void Segmenter::Euclid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*indices);
    }



}  // namespace perception
