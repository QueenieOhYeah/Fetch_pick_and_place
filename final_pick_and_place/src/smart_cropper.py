#! /usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from tf import TransformListener
from visualization_msgs.msg import Marker
import tf
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
import perception_msgs.msg


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
        self.target_bin = 1
        

    def callback(self, msg):
        self.markers = msg.markers
        marker = self.markers[0]
        self.crop(marker)
        
    def target_callback(self, msg):
        self.target_bin = msg.bin_id
        print(self.target_bin)


    def crop(self, marker):
        WIDTH = 0.2
        HEIGHT = 0.2
        DEPTH = 0.3
        SHIFT = Point(0.5, 0.13, 0.8)
#        bin_id = rospy.get_param("bin_id", 1)
#         bin_id = 2
       
        if self.target_bin == 1:
            WIDTH = 0.26
            HEIGHT = 0.14
            DEPTH = 0.27
            SHIFT = Point(0.38, 0.29, 0.13)

        if self.target_bin == 2:
            WIDTH = 0.1
            HEIGHT = 0.11
            DEPTH = 0.27
            SHIFT = Point(0.43, 0.075, 0.11)
        print("here")
        #cropped cloud
        rospy.set_param("crop_min_x", -HEIGHT/2)
        rospy.set_param("crop_min_y", -WIDTH/2)
        rospy.set_param("crop_min_z", -DEPTH)
        rospy.set_param("crop_max_x", HEIGHT/2)
        rospy.set_param("crop_max_y", WIDTH/2)
        rospy.set_param("crop_max_z", 0)

        
#        position, orientation, euler = self.get_shifted_pose(SHIFT, marker)
        # For real robot
        position, orientation, euler = self.get_shifted_pose(SHIFT, marker.id, 'head_camera_rgb_optical_frame')
        # For simulation
        #position, orientation, euler = self.get_shifted_pose(SHIFT, marker.id, 'base_link')

        
        rospy.set_param("t_x", position.x.item());
        rospy.set_param("t_y", position.y.item());
        rospy.set_param("t_z", position.z.item());
        rospy.set_param("r_x", euler[0]);
        rospy.set_param("r_y", euler[1]);
        rospy.set_param("r_z", euler[2]);
        
        
        # Visualize cube for debugging
        SHIFT_CUBE = SHIFT
        SHIFT_CUBE.z -= DEPTH/2
        
#        position_cube, orientation_cube, euler_cube = self.get_shifted_pose(SHIFT_CUBE, marker)
        position_cube, orientation_cube, euler_cube = self.get_shifted_pose(SHIFT_CUBE, marker.id, 'base_link')
        show_marker(position_cube, orientation_cube, [HEIGHT, WIDTH, DEPTH], 2.0, self.marker_pub)
              
            
#    def get_shifted_pose(self, shift, marker):
#        shift_pose = Pose(shift, Quaternion())
#        shift_pose_to_base = self.transform(shift_pose, 'ar_marker_'+ str(marker.id), marker.header.frame_id[1:])
#               
#        orientation = shift_pose_to_base.orientation
#        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
#        euler = tf.transformations.euler_from_quaternion(quat)
#        position = shift_pose_to_base.position
#        return position, orientation, euler

    def get_shifted_pose(self, shift, marker_id, marker_frame):
        shift_pose = Pose(shift, Quaternion())
        shift_pose_to_base = self.transform(shift_pose, 'ar_marker_'+ str(marker_id), marker_frame)
               
        orientation = shift_pose_to_base.orientation
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = tf.transformations.euler_from_quaternion(quat)
        position = shift_pose_to_base.position
        return position, orientation, euler
        

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
    target_sub = rospy.Subscriber('final_pick_and_place/target', perception_msgs.msg.Target, reader.target_callback)
    rospy.spin()
   


if __name__ == '__main__':
    main()
