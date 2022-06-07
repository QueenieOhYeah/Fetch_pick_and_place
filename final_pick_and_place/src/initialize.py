#!/usr/bin/env python

import rospy
import pickle
import robot_api
import numpy as np
from tf import TransformListener
import tf
import tf.transformations as tft
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker


class Initialize(object):
    def __init__(self):
        self.tl = TransformListener()
        #self.nav_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 10)
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback) # Subscribe to AR tag poses, use reader.callback
        self.marker_in_map = None
        self.flag = True

    def callback(self, markers):
        if markers.markers:
            self.marker_in_map = markers.markers[0]
            #self.marker_in_map = self.transform(marker.pose, marker.header.frame_id,'map')
            matrix = PoseStamped_2_mat(self.marker_in_map.pose)  
            b = np.array([0,0,1,1])
            x = matrix.dot(b)

            dp = np.dot(x[0:3], [1,0,0])
            print(dp)
    
    def get_marker_in_map(self):
        return self.marker_in_map

def transform(pose, from_frame, to_frame, tl):
        ''' Function to transform a pose between two ref. frames
        if there is a TF exception or object does not exist it
        will return the pose back without any transforms'''
        
        pose_stamped = PoseStamped()
        try:
            (trans,rot) = tl.lookupTransform(from_frame, to_frame, rospy.Time(0))
            print(rot)
            print(trans)
            common_time = tl.getLatestCommonTime(from_frame, to_frame)
            pose_stamped.header.stamp = common_time
            pose_stamped.header.frame_id = from_frame
            pose_stamped.pose = pose.pose
            rel_pose = tl.transformPose(to_frame, pose_stamped)
            return rel_pose
        except tf.Exception:
            rospy.logerr('TF exception during transform.')
            return None
        except rospy.ServiceException:
            rospy.logerr('Exception during transform.')
            return None

def transform_to_pose(matrix, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
#    rotation_matrix = matrix[0:3, 0:3]
#    transformation = matrix[0:3, -1:]
    #TODO: explain with comments
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = tft.quaternion_from_matrix(matrix)
    (
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ) = tft.translation_from_matrix(matrix)
    return pose

def PoseStamped_2_mat(p):
    q = p.pose.orientation
    pos = p.pose.position
    T = tft.quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def main():
    print("Initialize the pose")
    rospy.init_node('init_node')
    # Raise torso
    torso = robot_api.Torso()
    torso.set_height(0.4)
    filename = "/home/fetch/catkin_ws/src/Team-GIX-main/final_pick_and_place/data/ar_18_pose"
    try:
        infile = open(filename, 'rb')
        marker = pickle.load(infile)
        marker_in_base = marker.pose
        infile.close()
    except Exception as e:
        print(e)

    marker_T_base = PoseStamped_2_mat(marker_in_base)
    base_T_marker = np.linalg.inv(marker_T_base)
    base_in_marker = transform_to_pose(base_T_marker, "ar_marker_1")
    
    obj = Initialize()
    base = robot_api.Base()
    marker_in_map = None
    while not (marker_in_map):
        base.move(0, -0.1)
        marker_in_map = obj.get_marker_in_map()
    base.stop()
    target = None
    tl = TransformListener()
    while not target:
        target = transform(base_in_marker, base_in_marker.header.frame_id, "map", tl)
   
    #target.pose.position.z = 0
    nav_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, latch=True, queue_size = 1)
    print(target.pose.position)
    print(target.pose.orientation)
    marker_goal = rospy.Publisher('marker', Marker, latch=True, queue_size = 1)
    arrow_marker = Marker()
    arrow_marker.type = Marker.SPHERE
    arrow_marker.header.frame_id = "map"
    arrow_marker.pose = target.pose
    arrow_marker.scale.x = 0.5
    arrow_marker.scale.y = 0.5
    arrow_marker.scale.z = 0.5
    arrow_marker.color.r = 0
    arrow_marker.color.g = 0
    arrow_marker.color.b = 1
    arrow_marker.color.a = 1
    marker_goal.publish(arrow_marker)


    nav_goal.publish(target)
    print("Should move")


    

                     
    rospy.spin()


if __name__ == '__main__':
    main()

