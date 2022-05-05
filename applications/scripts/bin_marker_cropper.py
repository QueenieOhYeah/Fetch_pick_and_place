#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from tf import TransformListener
import tf
import numpy as np
import robot_api
import rospy
from scipy.spatial.transform import Rotation as R


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class SmartCropper(object):
    def __init__(self):
        self.markers = []
        self.tl = TransformListener()

    def callback(self, msg):
        WIDTH = 0.2
        HEIGHT = 0.2
        DEPTH = 0.3
        self.markers = msg.markers


        
        for marker in msg.markers:
#            marker_in_base = self.transform_to_matrix(marker)
            point1 = [WIDTH/2, HEIGHT/2, 0, 1]
            point2 = [-WIDTH/2, -HEIGHT/2, -DEPTH, 1]
#            point1_in_base = self.point_transform(marker_in_base, point1)
#            point2_in_base = self.point_transform(marker_in_base, point2)
#            min_x = min(point1_in_base[0], point2_in_base[0])
#            max_x = max(point1_in_base[0], point2_in_base[0])
#            min_y = min(point1_in_base[1], point2_in_base[1])
#            max_y = max(point1_in_base[1], point2_in_base[1])
#            min_z = point1_in_base[2]
#            max_z = point2_in_base[2]
#            rospy.loginfo(point1_in_base)
#            rospy.loginfo(point2_in_base)
            
#            marker_pose = marker.pose.pose

            

#            rospy.set_param("crop_min_x", 0)
#            rospy.set_param("crop_min_y", -WIDTH/2)
#            rospy.set_param("crop_min_z", -HEIGHT/2)
#            rospy.set_param("crop_max_x", DEPTH)
#            rospy.set_param("crop_max_y", WIDTH/2)
#            rospy.set_param("crop_max_z", HEIGHT/2)
            rospy.set_param("crop_min_x", -WIDTH/2)
            rospy.set_param("crop_min_y", -HEIGHT/2)
            rospy.set_param("crop_min_z", -DEPTH)
            rospy.set_param("crop_max_x", WIDTH/2)
            rospy.set_param("crop_max_y", HEIGHT/2)
            rospy.set_param("crop_max_z", 0)
            (t_x, t_y, t_z, r_x, r_y, r_z) = self.marker_tf_to_parameters(marker)
            
            

#            rospy.set_param("transform", transform)


            rospy.set_param("t_x", t_x);
            rospy.set_param("t_y", t_y);
            rospy.set_param("t_z", t_z);
            rospy.set_param("r_x", r_x);
            rospy.set_param("r_y", r_y);
            rospy.set_param("r_z", r_z);
            
            rospy.sleep(1)

            
    def marker_tf_to_parameters(self, marker:AlvarMarker):
        p = marker.pose.pose.position
        tq = marker.pose.pose.orientation
        quat = np.array([tq.x, tq.y, tq.z, tq.w])
        r = R.from_quat(quat)
        rot_vec = r.as_rotvec()
        return p.x, p.y, p.z, rot_vec[0].item(), rot_vec[1].item(), rot_vec[2].item()
        
    def transform_to_matrix(self, marker:AlvarMarker):
        position = marker.pose.pose.position
        quaternion = marker.pose.pose.orientation
        matrix = self.tl.fromTranslationRotation(position, quaternion)
        return matrix
        
    def point_transform(self, matrix:np.matrix, point_in_marker_frame:np.array):
        return np.dot(matrix, point_in_marker_frame)
        


def main():
    rospy.init_node('bin_marker_cropper')
    wait_for_time()
                                                  
    reader = SmartCropper()
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    rospy.spin()
   


if __name__ == '__main__':
    main()
