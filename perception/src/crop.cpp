#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


namespace perception {
Cropper::Cropper(const ros::Publisher& pub, const ros::Publisher& organized_pub) : pub_(pub),  organized_pub_(organized_pub){}


void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());
  //std::cout << "enter cropper" << std::endl;
  
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  double min_x, min_y, min_z, max_x, max_y, max_z;
  // display whole shelf
//  ros::param::param("crop_min_x", min_x, 1.0);
//  ros::param::param("crop_min_y", min_y, -1.0);
//  ros::param::param("crop_min_z", min_z, 0.5);
//  ros::param::param("crop_max_x", max_x, 2.0);
//  ros::param::param("crop_max_y", max_y, 1.0);
//  ros::param::param("crop_max_z", max_z, 2.0);
  
// For segmentation
  ros::param::param("crop_min_x", min_x, 1.0);
  ros::param::param("crop_min_y", min_y, -0.25);
  ros::param::param("crop_min_z", min_z, 1.12);
  ros::param::param("crop_max_x", max_x, 2.0);
  ros::param::param("crop_max_y", max_y, 0.0);
  ros::param::param("crop_max_z", max_z, 1.32);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
//  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//  Eigen::Vector3f translation;
//  Eigen::Vector3f rotation;
  double t_x, t_y, t_z, r_x, r_y, r_z;
  //ros::param::param("translation", translation, [0,0,0]);
  //ros::param::param("rotation", rotation, [0,0,0]);
  //ros::param::param("transform", transform);
  ros::param::param("t_x", t_x, 0.0);
  ros::param::param("t_y", t_y, 0.0);
  ros::param::param("t_z", t_z, 0.0);
  ros::param::param("r_x", r_x, 0.0);
  ros::param::param("r_y", r_y, 0.0);
  ros::param::param("r_z", r_z, 0.0);
  Eigen::Vector3f translation(t_x, t_y, t_z);
  Eigen::Vector3f rotation(r_x, r_y, r_z);
  
  
//  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//  //Eigen::Transform<float, 3, Eigen::Affine> transform;
//  //transform = Eigen::Translation(translation);
//  transform.translation() = translation;

//  transform.rotate(Eigen::AngleAxisf(r_x, Eigen::Vector3f(1, 0, 0)));
//  transform.rotate(Eigen::AngleAxisf(r_y, Eigen::Vector3f(0, 1, 0)));
//  transform.rotate(Eigen::AngleAxisf(r_z, Eigen::Vector3f(0, 0, 1)));
  pcl::CropBox<PointC> crop2;
  PointCloudC::Ptr cropped_cloud2(new PointCloudC());
  crop2.setInputCloud(cloud);
  crop2.setMin(min_pt);
  crop2.setMax(max_pt);
  //crop.setTransform(transform);
  crop2.setTranslation(translation);
  crop2.setRotation(rotation);
  crop2.filter(*cropped_cloud2);
  ROS_INFO("Cropped to %ld points", cropped_cloud2->size());
  sensor_msgs::PointCloud2 msg_out2;
  pcl::toROSMsg(*cropped_cloud2, msg_out2);
  pub_.publish(msg_out2);
  
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setKeepOrganized(true);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  //crop.setTransform(transform);
  crop.setTranslation(translation);
  crop.setRotation(rotation);
  crop.filter(*cropped_cloud);
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());
  //converse back to sensor_msgs::PointClouds2
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  organized_pub_.publish(msg_out);
  
  // pcl::CropBox<PointC> crop2;
  // PointCloudC::Ptr cropped_cloud2(new PointCloudC());
  // crop2.setInputCloud(cloud);
  // crop2.setKeepOrganized(false);
  // crop2.setMin(min_pt);
  // crop2.setMax(max_pt);
  // //crop.setTransform(transform);
  // crop2.setTranslation(translation);
  // crop2.setRotation(rotation);
  // crop2.filter(*cropped_cloud2);
  // ROS_INFO("Cropped to %ld points", cropped_cloud2->size());
  // sensor_msgs::PointCloud2 msg_out2;
  // pub_.publish(msg_out);
  // pcl::toROSMsg(*cropped_cloud2, msg_out2);
}
}
