#ifndef _PERCEPTION_OBJECT_H_
#define _PERCEPTION_OBJECT_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "perception_msgs/Object.h"

namespace perception {
struct Object {
  std::string name;
  double confidence;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 dimensions;
};

perception_msgs::Object object_to_msg(Object object) {
  perception_msgs::Object object_msg;
  object_msg.name = object.name;
  object_msg.confidence = object.confidence;
  object_msg.pose = object.pose;
  return object_msg;
}

/*struct Target {*/
/*  std::string name;*/
/*  int bin_id;*/
/*};*/

/*Target object_to_msg(perception_msgs::Target msg) {*/
/*  Target target;*/
/*  target.name = msg.name;*/
/*  target*/
/*  return object_msg;*/
/*}*/

}  // namespace perception

#endif  // _PERCEPTION_OBJECT_H_
