#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from robot_api import Arm, Gripper
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# Meshes for gripper marker
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


#def __make_6dof_control(self, int_marker, base_markers):
#def make_6dof_controls(int_marker, marker):
def make_6dof_controls():
    controls = []

    for axis in ["x", "y", "z"]:
        move_control = InteractiveMarkerControl()
        move_control.always_visible = True
        move_control.name = f"gripper_move_{axis}"
        move_control.orientation.w = 1
        setattr(move_control.orientation, axis, 1)
        move_control.interaction_mode = getattr(InteractiveMarkerControl, "MOVE_AXIS")
        controls.append(move_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.always_visible = True
        rotate_control.name = f"gripper_rotate_{axis}"
        rotate_control.orientation.w = 1
        setattr(rotate_control.orientation, axis, 1)
        rotate_control.interaction_mode = getattr(InteractiveMarkerControl, "ROTATE_AXIS")
        controls.append(rotate_control)
        #TODO: scale markers?
    
    return controls
    

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

        # When moved around, the marker should change color to reflect whether
        # the corresponding gripper pose is reachable by the robot or not.
        # Start with it green and change to red if pose is unreachable.
        self._gripper_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    def start(self):
        # Obtained from rosrun tf tf_echo wrist_roll_link gripper_link
        GRIPPER_LINK_OFFSET = [0.166, 0, 0] 

        # Obtained from rosrun tf tf_echo l_gripper_finger_link gripper_link
        FINGER_OFFSET = [0, 0.065, 0] # This is for the left finger. Right finger is [0, -.065, 0]
        
        gripper_im = InteractiveMarker()
        # To our eyes, gripper_link is the most intuitive end-effector link.
        gripper_im.header.frame_id = "gripper_link" 
        gripper_im.name = "Advanced Teleop Gripper Marker"
        gripper_im.header.stamp = rospy.Time.now()

        
        # You will need to create 3 markers: one for the gripper and two for
        # the fingertips. These markers will be added to a single 
        # InteractiveMarkerControl, which in turn is added to your InteractiveMarker
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = GRIPPER_LINK_OFFSET[0]
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 2.0
        gripper_marker.color = self._gripper_color

        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = GRIPPER_LINK_OFFSET[0]
        l_finger_marker.pose.position.y = -FINGER_OFFSET[1]
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = self._gripper_color


        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = GRIPPER_LINK_OFFSET[0]
        r_finger_marker.pose.position.y = FINGER_OFFSET[1]
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = self._gripper_color

        controls = make_6dof_controls()#gripper_im, gripper_marker)
        gripper_im.controls.extend(controls)

        gripper_control = InteractiveMarkerControl()
        #button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        gripper_control.always_visible = True
        gripper_control.markers.append(gripper_marker)
        gripper_control.markers.append(l_finger_marker)
        gripper_control.markers.append(r_finger_marker)
        gripper_im.controls.append(gripper_control)
        
        
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node("advanced_teleop")
    wait_for_time()

    arm = Arm()
    gripper = Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server')
    #auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    #auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()