#!/usr/bin/env python

import rospy
import pickle
from geometry_msgs.msg import Pose, PoseStamped
from demonstration.msg import PoseNames, UserAction
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from tf import TransformListener



DEBUG = True

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


        
#subscribe to useraction
class PbD_Create_Pose_Server(object):
    def __init__(self):
        # Attributes
        self.pickle_file = "PbDpositions"
        # List of detected markers
        self.markers = []
        self.poses = []

#        self.dict = self.unpickling(self.pickle_file)
#        if self.dict == None:
#            self.dict = {}
        
        # Subscribe to AR tag poses, use reader.callback
        # To detect markers:
        # Need to: roslaunch robot_api ar_desktop.launch cam_image_topic:=<cloud point topic name>
        seld.marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_callback) 

        self.user_action = rospy.Subscriber('demonstration/user_actions', UserAction, self.callback_user_action)

        
    def marker_callback(self, msg):
        self.markers = msg.markers

# data structure for saving can be [[pose1, gripper_state],[pose2, gripper2], [pose3, gripper3]]
    def pickling(self, filename, d):
        outfile = open(filename, 'wb')
        pickle.dump(d, outfile)
        outfile.close()

    def unpickling(self, filename):
        #filename = "fetch_position"
        try:   
            infile = open(filename, 'rb')
            obj = pickle.load(infile)
            infile.close()
            return obj
        except:
            return None
            
# Helper function, might not needed
    def pose_to_stampedpose(self, pose: Pose):
        stamped_pose = PoseStamped()
        stamped_pose.pose = pose
        return stamped_pose

    def callback_user_action(self, data: UserAction):
        if data.command == UserAction.RELAX:
            # create the program, relax arms
        if data.command == UserAction.SAVE_TO_BASE:
            # transform pose and save poses related to base_link
            # ...
            self.save_poses(pose, gripper)
        if data.command == UserAction.SAVE_TO_TAG:
            # transform pose and save poses related to tag
            # ...
            self.save_poses(pose, gripper)
        if data.command == UserAction.OPEN:
            # open gripper
        if data.command == UserAction.CLOSE:
            # close gripper
        if data.command == UserAction.CREATE:
            # save program, use pickling()
        
        
    def save_poses(self, pose: PoseStamped, gripper_status):
        self.poses.append([pose, gripper_status])
    
    '''Get pose for each marker usage:
       for marker in self.markers: 
           get_marker_pose(marker) '''    
    def get_marker_pose(self, marker:AlvarMarker):
        # usage: position.x, position.y, position.z
        position = marker.pose.pose.position        
        quaternion = marker.pose.pose.orientation
        return position, quaternion
        


def main():
    rospy.init_node('create_pose_server')
    server = PbD_Create_Pose_Server()
    wait_for_time()

                     
    rospy.spin()


if __name__ == '__main__':
    main()
