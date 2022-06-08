#include "perception/final_segmentation.h"

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
#include <pcl/common/point_tests.h>
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectList.h"
#include "perception_msgs/Object.h"
#include "perception_msgs/Target.h"


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {


Segmenter_final::Segmenter_final(const ros::Publisher& points_pub, const ros::Publisher& markers_pub, const ros::Publisher& objects_pub, const ObjectRecognizer& recognizer)
    : points_pub_(points_pub), markers_pub_(markers_pub), objects_pub_(objects_pub), recognizer_(recognizer) {} 

//Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& markers_pub)
//    : points_pub_(points_pub), markers_pub_(markers_pub) {}

void Segmenter_final::UpdateRCNN(const perception_msgs::ObjectList& object_list) {
    std::vector<std::string> objects = object_list.objects;
    std::vector<std::string>::iterator it = std::find(objects.begin(), objects.end(), target_.name);
    std::cout << "here" << std::endl;
    std::cout << target_.name << std::endl;
    std::cout << (it != objects.end()) << std::endl;
    if (it != objects.end()) {
        std::vector<int> indices = object_list.object_indices[it - objects.begin()].indices;
        std::cout << it - objects.begin() << std::endl;
        indices_ = indices;
    }
        
}

void Segmenter_final::RCNNCutPointCloud(PointCloudC::Ptr cloud, PointCloudC::Ptr new_cloud) {
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  inliers -> header = cloud -> header;
  for (int i = 0; i <= indices_.size(); i++) {
    inliers->indices = indices_;   
  }
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*new_cloud);
  std::cerr << "cloud width " << new_cloud->width << " data points." << std::endl;


}

void Segmenter_final::SegmentBinObjects(PointCloudC::Ptr cloud,
                                      std::vector<pcl::PointIndices>* indices) {
    int function_id;
    ros::param::param("function", function_id, 1);
    
   std::cout << "Using segmentation method:" << function_id << std::endl;
    
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

void Segmenter_final::CutPointCloud(PointCloudC::Ptr cloud, PointCloudC::Ptr new_cloud) {
  int top_right_x, top_right_y, bottom_left_x, bottom_left_y;
  ros::param::param("top_right_x", top_right_x, -1);
  ros::param::param("top_right_y", top_right_y, -1);
  ros::param::param("bottom_left_x", bottom_left_x, -1);
  ros::param::param("bottom_left_y", bottom_left_y, -1);

  if (top_right_x < 0) return;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  inliers -> header = cloud -> header;
  for (int x = top_right_x; x <= bottom_left_x; x++) {
    for (int y = top_right_y; y <= bottom_left_y; y++) {
      inliers->indices.push_back(y * 640 + x);   
    }
  }

  
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*new_cloud);
  std::cerr << "cloud width" << new_cloud->width << " data points." << std::endl;
}

void Segmenter_final::GetSuggestPoint(PointCloudC::Ptr cloud, PointC* suggest_point) {
  int double_click_x, double_click_y;
  ros::param::param("double_click_x", double_click_x, -1);
  ros::param::param("double_click_y", double_click_y, -1);
  *suggest_point = (*cloud)[double_click_y * 640 + double_click_x];

  if (isFinite(*suggest_point)) {
    visualization_msgs::Marker point_marker;
    point_marker.ns = "suggested point";
    point_marker.id = 1000;
    point_marker.header.frame_id = "base_link";
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.pose.position.x = suggest_point->x;
    point_marker.pose.position.y = suggest_point->y;
    point_marker.pose.position.z = suggest_point->z;
    point_marker.pose.orientation.x = 0.0;
    point_marker.pose.orientation.y = 0.0;
    point_marker.pose.orientation.z = 0.0;
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.05;
    point_marker.scale.y = 0.05;
    point_marker.scale.z = 0.05;
    point_marker.color.r = 1;
    point_marker.color.a = 0.7;
    point_marker.lifetime = ros::Duration(8);
    markers_pub_.publish(point_marker);   
    
    ros::param::set("suggested_point_x", suggest_point->x);
    ros::param::set("suggested_point_y", suggest_point->y);
    ros::param::set("suggested_point_z", suggest_point->z);
  } else {
    ros::param::set("suggested_point_x", -1);
    ros::param::set("suggested_point_y", -1);
    ros::param::set("suggested_point_z", -1);
  
  }
}


void Segmenter_final::SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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

void Segmenter_final::GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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

//// Update target once received new command
void Segmenter_final::UpdateTarget(const perception_msgs::Target& target) {
  target_ = target;
  std::cout << target_.name << std::endl;

}

void Segmenter_final::Callback(const sensor_msgs::PointCloud2& msg) {
  std::cout << target_.name << std::endl;
  std::cout << "indices_len" << indices_.size() << std::endl;
  int HUMAN;
  ros::param::param("human", HUMAN, 0);
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  PointCloudC::Ptr filtered_cloud(new PointCloudC());
  std::vector<int> index;
  
  
  PointCloudC::Ptr new_cloud(new PointCloudC());
  if (HUMAN == 1) {
    CutPointCloud(cloud, new_cloud);
    PointC suggest_point;
    GetSuggestPoint(cloud, &suggest_point);
  }
  else {
    ros::param::set("top_right_x", -1);
    ros::param::set("top_right_y", -1);
    ros::param::set("bottom_left_x", -1);
    ros::param::set("bottom_left_y", -1);
    ros::param::set("suggested_point_x", -1);
    ros::param::set("suggested_point_y", -1);
    ros::param::set("suggested_point_z", -1);
    RCNNCutPointCloud(cloud, new_cloud);
  }
  
  pcl::removeNaNFromPointCloud(*new_cloud, *filtered_cloud, index);
  geometry_msgs::Pose human_pose;
  geometry_msgs::Vector3 human_dimensions;
  GetAxisAlignedBoundingBox(filtered_cloud, &human_pose, &human_dimensions);

  
  visualization_msgs::Marker human_marker;
  human_marker.ns = "human_objects";
  human_marker.id = 100;
  human_marker.header.frame_id = "base_link";
  human_marker.type = visualization_msgs::Marker::CUBE;
  human_marker.pose = human_pose;
  human_marker.scale = human_dimensions;
  human_marker.color.r = 1;
  human_marker.color.a = 0.3;
  human_marker.lifetime = ros::Duration(8);
  markers_pub_.publish(human_marker); 
  
//  pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, index);

  
  std::vector<Object> objects;
  SegmentObjects(filtered_cloud, &objects);
  
  // Choose the target with max confidence
  double max_confidence = 0;
  Object target;
  for (size_t i = 0; i < objects.size(); ++i) {

    Object& object = objects[i];
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
    object_marker.lifetime = ros::Duration(8);
    markers_pub_.publish(object_marker);
             
  if (objects.size() <= 1 && !target_.name.empty()) {
    Object& object = objects[0];
    object.name = target_.name;
    int suggested_point_x, suggested_point_y, suggested_point_z;
    ros::param::param("suggested_point_x", suggested_point_x, -1);
    ros::param::param("suggested_point_y", suggested_point_y, -1);
    ros::param::param("suggested_point_z", suggested_point_z, -1);
    
    
    perception_msgs::Object object_msg = object_to_msg(object);
    object_msg.bin_id = target_.bin_id;
    object_msg.suggested_point.x = suggested_point_x;
    object_msg.suggested_point.y = suggested_point_y;
    object_msg.suggested_point.z = suggested_point_z;
    objects_pub_.publish(object_msg);
  
  }
    
    
    
//    // Publish the object list
//    // To do: add bin id
//    perception_msgs::Object object_msg;
//    object_msg = object_to_msg(object, name);
////    std::vector<perception_msgs::Object> objects;
//    perception_msgs::ObjectList objects;
//    objects.objects.push_back(object_msg);
//  objects_pub_.publish(objects); 
  

}

}



void Segmenter_final::Euclid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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
    
void Segmenter_final::RegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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
    
void Segmenter_final::ColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        float distance_thresh, point_color_thresh, region_color_thresh;
        int min_cluster_size, max_cluster_size, k_search, neighbours;
        ros::param::param("distance_thresh", distance_thresh, 10.0f);
        ros::param::param("creg_min_cluster_size", min_cluster_size, 1000);
        ros::param::param("creg_max_cluster_size", max_cluster_size, 10000);
        ros::param::param("point_color_thresh", point_color_thresh, 6.0f);
        ros::param::param("region_color_thresh", region_color_thresh, 5.0f);

//        std::cout << distance_thresh << std::endl;
//        std::cout << min_cluster_size << std::endl;
//        std::cout << max_cluster_size << std::endl;
//        std::cout << point_color_thresh << std::endl;
//        std::cout << region_color_thresh << std::endl;
        
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
