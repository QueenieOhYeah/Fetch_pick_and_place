#! /usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from tf import TransformListener
from visualization_msgs.msg import Marker
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

def show_marker(position, orientation, scale_, lifetime_, marker_pub_):
    marker_ = Marker()
    marker_.header.frame_id = "base_link"
    # marker_.header.stamp = rospy.Time.now()
    marker_.type = marker_.CUBE
    marker_.action = marker_.ADD

    marker_.pose.position = position
    marker_.pose.orientation = orientation

    marker_.lifetime = rospy.Duration.from_sec(lifetime_)
    marker_.scale.x = scale_[0]
    marker_.scale.y = scale_[1]
    marker_.scale.z = scale_[2]
    marker_.color.a = 0.5
    marker_.color.r = 0
    marker_.color.g = 1
    marker_.color.b = 0
    marker_pub_.publish(marker_)
    

    
class SmartCropper(object):
    def __init__(self):
        self.markers = []
        self.tl = TransformListener()
        self.marker_pub = rospy.Publisher("/shelf_debug_marker", Marker, queue_size = 2)

    def callback(self, msg):
        self.markers = msg.markers
        marker = self.markers[1]
        self.crop(marker)


    def crop(self, marker):
        WIDTH = 0.2
        HEIGHT = 0.2
        DEPTH = 0.3
        SHIFT = 0.16
#        marker_in_base = self.transform_to_matrix(marker)
#        point1 = [WIDTH/2, HEIGHT/2, 0, 1]
#        point2 = [-WIDTH/2, -HEIGHT/2, -DEPTH, 1]
#       point1_in_base = self.point_transform(marker_in_base, point1)
#       point2_in_base = self.point_transform(marker_in_base, point2)
#       min_x = min(point1_in_base[0], point2_in_base[0])
#       max_x = max(point1_in_base[0], point2_in_base[0])
#       min_y = min(point1_in_base[1], point2_in_base[1])
#       max_y = max(point1_in_base[1], point2_in_base[1])
#       min_z = point1_in_base[2]
#       max_z = point2_in_base[2]
#       rospy.loginfo(point1_in_base)
#       rospy.loginfo(point2_in_base)
            
#       marker_pose = marker.pose.pose

            #works
#        rospy.set_param("crop_min_x", -WIDTH/2)
#        rospy.set_param("crop_min_y", -HEIGHT/2)
#        rospy.set_param("crop_min_z", -DEPTH/2)
#        rospy.set_param("crop_max_x", WIDTH/2)
#        rospy.set_param("crop_max_y", HEIGHT/2)
#        rospy.set_param("crop_max_z", DEPTH/2)
        shift_pose = Pose(Point(0, 0, -SHIFT), Quaternion())
        shifted_pose = self.transform(shift_pose, 'ar_marker_'+ str(marker.id), 'base_link')
        print('ar_marker_'+ str(marker.id))
            

        orientation = shifted_pose.orientation
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = tf.transformations.euler_from_quaternion(quat)
        position = shifted_pose.position
            
        show_marker(shifted_pose.position, shifted_pose.orientation, [WIDTH, HEIGHT, DEPTH], 2.0, self.marker_pub)
#        rospy.set_param("t_x", position.x.item());
#        rospy.set_param("t_y", position.y.item());
#        rospy.set_param("t_z", position.z.item());
#        rospy.set_param("r_x", euler[0]);
#        rospy.set_param("r_y", euler[1]);
#        rospy.set_param("r_z", euler[2]);
 


        rospy.set_param("crop_min_x", -WIDTH/2)
        rospy.set_param("crop_min_y", -HEIGHT/2)
        rospy.set_param("crop_min_z", -DEPTH)
        rospy.set_param("crop_max_x", WIDTH/2)
        rospy.set_param("crop_max_y", HEIGHT/2)
        rospy.set_param("crop_max_z", 0)
#        #(t_x, t_y, t_z, r_x, r_y, r_z) = self.marker_tf_to_parameters(marker)
#            
##       shift_pose = Pose(Point(0, 0, SHIFT), Quaternion())
##       shifted_pose = self.transform(shift_pose, 'ar_marker_1', 'base_link')

        orientation = marker.pose.pose.orientation
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = tf.transformations.euler_from_quaternion(quat)
###      position = shifted_pose.position

        position = marker.pose.pose.position            
#        #position.x = position.x + SHIFT
#        show_marker(position, orientation, [WIDTH, HEIGHT, DEPTH], 2.0, self.marker_pub)
        rospy.set_param("t_x", position.x);
        rospy.set_param("t_y", position.y);
        rospy.set_param("t_z", position.z);
        rospy.set_param("r_x", euler[0]);
        rospy.set_param("r_y", euler[1]);
        rospy.set_param("r_z", euler[2]);       
            
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
        

    def transform(self, pose, from_frame, to_frame):
        ''' Function to transform a pose between two ref. frames
        if there is a TF exception or object does not exist it
        will return the pose back without any transforms'''
        
        pose_stamped = PoseStamped()
        try:
            common_time = self.tl.getLatestCommonTime(from_frame, to_frame)
            pose_stamped.header.stamp = common_time
            pose_stamped.header.frame_id = from_frame
            pose_stamped.pose = pose
            rel_pose = self.tl.transformPose(to_frame, pose_stamped)
            return rel_pose.pose
        except tf.Exception:
            rospy.logerr('TF exception during transform.')
            return pose
        except rospy.ServiceException:
            rospy.logerr('Exception during transform.')
            return pose



def main():
    rospy.init_node('bin_marker_cropper')
    wait_for_time()
                                                  
    reader = SmartCropper()
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    rospy.spin()
   


if __name__ == '__main__':
    main()
