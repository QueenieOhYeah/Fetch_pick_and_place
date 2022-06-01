#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/region_growing_rgb.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include <pcl/common/common.h>
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectList.h"
#include "perception_msgs/Object.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {


Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub, const ros::Publisher& objects_pub, const ObjectRecognizer& recognizer)
    : points_pub_(points_pub), markers_pub_(markers_pub), objects_pub_(objects_pub), recognizer_(recognizer) {} 

//Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub)
//    : points_pub_(points_pub), markers_pub_(markers_pub) {}

void Segmenter::SegmentBinObjects(PointCloudC::Ptr cloud,
                                      std::vector<pcl::PointIndices>* indices) {
    int function_id;
    ros::param::param("function", function_id, 1);
    
    std::cout << function_id << std::endl;
    
    if (function_id == 1) {
      Euclid(cloud, indices);
    }
    if (function_id == 2) {
      RegionGrowing(cloud, indices);
    }
    if (function_id == 3) {
      ColorRegionGrowing(cloud, indices);
    }
    //Euclid(cloud, indices);
    //RegionGrowing(cloud, indices);
    //ColorRegionGrowing(cloud, indices);
    
    
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
    
//    sensor_msgs::PointCloud2 msg_out;
//    pcl::toROSMsg(*cloud, msg_out);
//    points_pub_.publish(msg_out);

}

void Segmenter::SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects) {
  std::vector<pcl::PointIndices> object_indices;
  SegmentBinObjects(cloud, &object_indices);
  PointCloudC::Ptr cloud_cluster (new PointCloudC());
  
  for (size_t i = 0; i < object_indices.size(); ++i) {
    //// Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.filter(*object_cloud);
    Object object = Object();
    object.cloud = object_cloud;
    GetAxisAlignedBoundingBox(object_cloud, &object.pose, &object.dimensions);
    objects->push_back(object);
    
    cloud_cluster->insert(std::end(*cloud_cluster), std::begin(*object_cloud), std::end(*object_cloud));   
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud, msg_out);
  points_pub_.publish(msg_out);                          
}
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
  PointCloudC::Ptr filtered_cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, index);

  
  std::vector<Object> objects;
  SegmentObjects(filtered_cloud, &objects);
  
  for (size_t i = 0; i < objects.size(); ++i) {

    const Object& object = objects[i];
    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose = object.pose;
    object_marker.scale = object.dimensions;
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    object_marker.lifetime = ros::Duration(3);
    markers_pub_.publish(object_marker);
    
    // Recognize the object.
    std::string name;
    double confidence;
    // TODO: recognize the object with the recognizer_. /////////////////////////////
    recognizer_.Recognize(object, &name, &confidence);
    confidence = round(1000 * confidence) / 1000;

    std::stringstream ss;
    ss << name << " (" << confidence << ")";

    // Publish the recognition result.
    visualization_msgs::Marker name_marker;
    name_marker.ns = "recognition";
    name_marker.id = i;
    name_marker.header.frame_id = "base_link";
    name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_marker.pose.position = object.pose.position;
    name_marker.pose.position.z += 0.1;
    name_marker.pose.orientation.w = 1;
    name_marker.scale.x = 0.025;
    name_marker.scale.y = 0.025;
    name_marker.scale.z = 0.025;
    name_marker.color.r = 0;
    name_marker.color.g = 0;
    name_marker.color.b = 1.0;
    name_marker.color.a = 1.0;
    name_marker.text = ss.str();
    markers_pub_.publish(name_marker);
    
    
    // Publish the object list
    // To do: add bin id
    perception_msgs::Object object_msg;
    object_msg = object_to_msg(object);
//    std::vector<perception_msgs::Object> objects;
    perception_msgs::ObjectList objects;
    objects.objects.push_back(object_msg);
  objects_pub_.publish(objects); 

}

}



void Segmenter::Euclid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.02);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 1000);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*indices);
    }
    
void Segmenter::RegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        double smoothness_thresh, curv_thresh;
        int min_cluster_size, max_cluster_size, k_search, neighbours;
        ros::param::param("reg_k", k_search, 50);
        ros::param::param("reg_min_cluster_size", min_cluster_size, 1000);
        ros::param::param("reg_max_cluster_size", max_cluster_size, 10000);
        ros::param::param("reg_n", neighbours, 30);
        ros::param::param("reg_sthres", smoothness_thresh, 3.0);
        ros::param::param("reg_cthres", curv_thresh, 1.0);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *cloud_xyz);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud_xyz);
        normal_estimator.setKSearch (k_search);
        normal_estimator.compute (*normals);
        
        
       
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (min_cluster_size);
        reg.setMaxClusterSize (max_cluster_size);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (neighbours);
        reg.setInputCloud (cloud_xyz);
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        
        
        reg.setSmoothnessThreshold (smoothness_thresh/ 180.0 * M_PI);
        reg.setCurvatureThreshold (curv_thresh);
        reg.extract(*indices);
    }
    
void Segmenter::ColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        float distance_thresh, point_color_thresh, region_color_thresh;
        int min_cluster_size, max_cluster_size, k_search, neighbours;
        ros::param::param("distance_thresh", distance_thresh, 10.0f);
        ros::param::param("creg_min_cluster_size", min_cluster_size, 1000);
        ros::param::param("creg_max_cluster_size", max_cluster_size, 10000);
        ros::param::param("point_color_thresh", point_color_thresh, 6.0f);
        ros::param::param("region_color_thresh", region_color_thresh, 5.0f);

        std::cout << distance_thresh << std::endl;
        std::cout << min_cluster_size << std::endl;
        std::cout << max_cluster_size << std::endl;
        std::cout << point_color_thresh << std::endl;
        std::cout << region_color_thresh << std::endl;
        
        pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud (cloud);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (distance_thresh);
        reg.setPointColorThreshold (point_color_thresh);
        reg.setRegionColorThreshold (region_color_thresh);
        reg.setMinClusterSize (min_cluster_size);
        reg.setMaxClusterSize (max_cluster_size);
        reg.extract (*indices);
    }



}  // namespace perception
