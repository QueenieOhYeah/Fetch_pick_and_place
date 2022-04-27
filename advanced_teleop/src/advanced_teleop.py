#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MenuEntry
from robot_api import Arm, Gripper
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped

# Meshes for gripper marker
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


#def __make_6dof_control(self, i_marker, base_markers):
#def make_6dof_controls(i_marker, marker):
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

    def _build_menu(self, i_marker):
        MENU_OPTIONS = {"Move gripper here":1, "Open gripper":2, "Close gripper":3}

        for key in MENU_OPTIONS.keys():
            menu_entry = MenuEntry()
            menu_entry.id = MENU_OPTIONS[key]
            menu_entry.parent_id = 0
            menu_entry.title = key
            menu_entry.command_type = MenuEntry.FEEDBACK
            i_marker.menu_entries.append(menu_entry)

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

        # Add the 6 dof controls to our interactive marker
        controls = make_6dof_controls()#gripper_im, gripper_marker)
        gripper_im.controls.extend(controls)

        # Add the meshes (without any interaction mode) so we can display them
        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_control.always_visible = True
        gripper_control.markers.append(gripper_marker)
        gripper_control.markers.append(l_finger_marker)
        gripper_control.markers.append(r_finger_marker)
        gripper_im.controls.append(gripper_control)

        # Build the menu ("go to postion", "open gripper", "close gripper")
        self._build_menu(gripper_im)

        # We'll need this in order to update its colors later
        self._i_marker = gripper_im
        
        # Add the interactive marker to the server
        #self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.insert(self._i_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        # We moved the gripper to a new pose, check to see if it's reachable.
        # Change the color to green if it is and red if it's not.
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            ps = PoseStamped()
            ps.header = feedback.header
            ps.pose = feedback.pose
            #self._last_pose = ps
            print(self._arm.compute_ik(ps))
            if self._arm.compute_ik(ps):
                print("Found pose")
                self._gripper_color = ColorRGBA(0, 1.0, 0, 1.0)
            else:
                print("No pose")
                self._gripper_color = ColorRGBA(1.0, 0, 0, 1.0)
            
            for control in self._i_marker.controls:
                for marker in control.markers:
                    marker.color = self._gripper_color

            # These next two lines are so the marker is in the right pose when
            # we re-draw it
            self._i_marker.header = feedback.header
            self._i_marker.pose = feedback.pose

            self._im_server.erase(self._i_marker.name)
            self._im_server.insert(self._i_marker, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print("FEEDBACK")
            print(feedback)
            """
            if feedback.menu_entry_id == 1:
                pose = PoseStamped()
                pose.header = feedback.header
                pose.header.frame_id = self.__object_name

                # Since we changed the frame of the pose, the position should be 0
                pose.pose = Pose()
                pose.pose.orientation.w = 1.0
                self.__grasp_callback(pose)
            """

        """
        if self._im_marker is None:
            return
        for control in self._im_marker.controls:
            for marker in control.markers:
                marker.color = self._cur_color
        self._im_server.erase(self._im_marker.name)
        self._im_server.insert(self._im_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        """
        
       


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