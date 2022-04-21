#!/usr/bin/env python

import rospy
import pickle
import geometry_msgs.msg import Pose, PoseStamped
from map_annotator.srv import Save, List, Delete, Goto
from map_annotator.msg import PoseNames, UserAction
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback



def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


        
#subscribe to useraction
class MapAnnotationServer(object):
    def __init__(self):
        # Attributes
        self.pickle_file = "fetch_position"

        self.dict = self._unpickling(self._pickle_file)
        if self.dict == None:
            self.dict = {}
        #rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)
        self.nav_goal = rospy.Publisher('move_base_simple/goal', PoseStamped)
        self.pose_names = rospy.Publisher('map_annotator/pose_names', PoseNames, latch=True)
        self.user_action = rospy.Subscriber('map_annotator/user_actions', UserAction, self.callback_user_action)
        self.interactive_marker_server = InteractiveMarkerServer("/map_annotator/map_poses")
        #self.initial_pose = geometry_msgs.msg.Pose()
        # create saved markers
        for key, value in self.dict:
            self.create_interactive_marker(key, value)
        
#    def callback(self, data):
#        self._data = data

    def pickling(self, filename, d: dict):
        #filename = "fetch_position"
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
# Marker pose type in geometry_msgs.msg.Pose while goto function uses geometry_msgs.msg.PoseStamped
    def pose_to_stampedpose(self, pose: Pose):
        stamped_pose = PoseStamped()
        stamped_pose.pose = pose
        return stamped_pose

    def callback_user_action(self, data: UserAction):
        if data.command == UserAction.CREATE:
            self.create_interactive_marker(data.name)
        if data.command == UserAction.DELETE:
            self.delete_pose(data.name)
        if data.command == UserAction.GOTO:
            self.goto_pose(data.name)


    def create_interactive_marker(self, name: str, stamped_pose: PoseStamped = PoseStamped()):
        marker = InteractiveMarker()
        marker.header = stamped_pose.header
        marker.header.frame_id = "base_link"
        marker.pose = stamped_pose.pose
        marker.pose.position.z = 0.05
        marker.name = name
        marker.description = name

        marker_control = InteractiveMarkerControl()
        marker_control.orientation.w = 1;
        marker_control.orientation.x = 0;
        marker_control.orientation.y = 1;
        marker_control.orientation.z = 0;
        marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        marker.append(marker_control)
        arrow_marker = Marker()
        arrow_marker.scale.x = 1
        arrow_marker.scale.y = 0.3
        arrow_marker.scale.z = 0.3
        arrow_marker.color.r = 0
        arrow_marker.color.g = 0
        arrow_marker.color.b = 1

        rotate_control = InteractiveMarkerControl()
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation.w = 1;
        rotate_control.orientation.x = 0;
        rotate_control.orientation.y = 1;
        rotate_control.orientation.z = 0;

        marker_control.always_visible = True
        marker_control.append(arrow_marker)
        rotate_control.append(arrow_marker)
        marker.controls.append(marker_control)
        marker.controls.append(rotate_control_control)

        self.interactive_marker_server.insert(marker, self.interactive_marker_callback)
        self.interactive_marker_server.applyChanges()

    def interactive_marker_callback(self, feedback: InteractiveMarkerFeedback):
        pose = feedback.pose
        stamped_pose = self.pose_to_stampedpose(pose)
        name = feedback.marker_name
        self.save_pose(name, stamped_pose)
        self.list_saved_poses()

    def list_saved_poses(self):
        pose_names = PoseNames()
        pose_names.names = self.dict.keys()
        self.pose_names.publish(pose_names)
        
    def check_name(self, name):
        if name in self._dict:
            return True
        return False
        
    def save_pose(self, name, pos: geometry_msgs.msg.PoseStamped):
#        pos = geometry_msgs.msg.PoseStamped()
#        pos.header = self._data.header
#        pos.pose = self._data.pose.pose
        # Get position of the marker
        self.dict[name] = pos
        self.list_saved_poses()     
        pickling(self.pickle_file, self.dict)
        return 1
        
    def delete_pose(self, name):
        self.interactive_marker_server.erase(name)
        self.dict.pop(name)
        self.list_saved_poses()  
        pickling(self.pickle_file, self.dict)
        return 1
        
    def goto_pose(self, name):
        pos = self.dict[name]

        self.nav_goal.publish(pos)
        return 1

def main():
    rospy.init_node('map_annotator_node')
    wait_for_time()
    
    
    # server = PickleNode()
    # save_service = rospy.Service('map_annotator/save', Save,
    #                               server.save_pose)
    # list_service = rospy.Service('map_annotator/list', List,
    #                               server.list_saved_poses)
    # delete_service = rospy.Service('map_annotator/delete', Delete,
    #                               server.delete_pose)
    # goto_service = rospy.Service('map_annotator/goto', Goto,
    #                               server.goto_pose)
                     
    rospy.spin()


if __name__ == '__main__':
    main()
