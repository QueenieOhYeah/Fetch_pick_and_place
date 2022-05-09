#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MenuEntry
from robot_api import Arm, Gripper
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import OrientationConstraint
import copy
import tf.transformations as tft
import tf
import numpy as np

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

def get_gripper_markers(init_color, offset=[0,0,0], ids=[0,0,0]):
    # The gripper will be offset in certain directions to allow for the
    # "pre-grasp" and "lift" poses of the target interface. The offset 
    # argument takes this into account. The offsets listed below are
    # just to get the gripper in the right position using gripper_link
    # as the end-effector instead of the wrist_roll_link that MoveIt uses
    # by default. 

    # Obtained from rosrun tf tf_echo wrist_roll_link gripper_link
    GRIPPER_LINK_OFFSET = [0.166, 0, 0] 

    # Obtained from rosrun tf tf_echo l_gripper_finger_link gripper_link
    FINGER_OFFSET = [0.166, 0.065, 0] # This is for the left finger. Right finger is [0, -.065, 0]

    gripper_marker = Marker()
    gripper_marker.id = ids[0] # This will be used to individually color them later
    gripper_marker.type = Marker.MESH_RESOURCE
    gripper_marker.mesh_resource = GRIPPER_MESH
    gripper_marker.pose.position.x = GRIPPER_LINK_OFFSET[0] + offset[0]
    gripper_marker.pose.position.y = GRIPPER_LINK_OFFSET[1] + offset[1]
    gripper_marker.pose.position.z = GRIPPER_LINK_OFFSET[2] + offset[2]
    gripper_marker.scale.x = 1.0
    gripper_marker.scale.y = 1.0
    gripper_marker.scale.z = 1.0
    gripper_marker.color = init_color

    l_finger_marker = Marker()
    l_finger_marker.id = ids[1]
    l_finger_marker.type = Marker.MESH_RESOURCE
    l_finger_marker.mesh_resource = L_FINGER_MESH
    l_finger_marker.pose.position.x = FINGER_OFFSET[0] + offset[0]
    l_finger_marker.pose.position.y = -FINGER_OFFSET[1] + offset[1]
    l_finger_marker.pose.position.z = FINGER_OFFSET[2] + offset[2]
    l_finger_marker.scale.x = 1.0
    l_finger_marker.scale.y = 1.0
    l_finger_marker.scale.z = 1.0
    l_finger_marker.color = init_color

    r_finger_marker = Marker()
    r_finger_marker.id = ids[2]
    r_finger_marker.type = Marker.MESH_RESOURCE
    r_finger_marker.mesh_resource = R_FINGER_MESH
    r_finger_marker.pose.position.x = FINGER_OFFSET[0] + offset[0]
    r_finger_marker.pose.position.y = FINGER_OFFSET[1] + offset[1]
    r_finger_marker.pose.position.z = FINGER_OFFSET[2] + offset[2]
    r_finger_marker.scale.x = 1.0
    r_finger_marker.scale.y = 1.0
    r_finger_marker.scale.z = 1.0
    r_finger_marker.color = init_color

    return gripper_marker, l_finger_marker, r_finger_marker
      

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._pose_is_reachable = True

        # When moved around, the marker should change color to reflect whether
        # the corresponding gripper pose is reachable by the robot or not.
        # Start with it green and change to red if pose is unreachable.
        self._gripper_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    def _build_menu(self, i_marker):
        self._MENU_OPTIONS = {"Move gripper here":1, "Open gripper":2, "Close gripper":3}

        for key in self._MENU_OPTIONS.keys():
            menu_entry = MenuEntry()
            menu_entry.id = self._MENU_OPTIONS[key]
            menu_entry.parent_id = 0
            menu_entry.title = key
            menu_entry.command_type = MenuEntry.FEEDBACK
            i_marker.menu_entries.append(menu_entry)

    def start(self):
        gripper_im = InteractiveMarker()
        # To our eyes, gripper_link is the most intuitive end-effector link.
        gripper_im.header.frame_id = "gripper_link" 
        gripper_im.name = "Advanced Teleop Gripper Marker"
        gripper_im.header.stamp = rospy.Time.now()
        print(gripper_im.header)

        
        # You will need to create 3 markers: one for the gripper and two for
        # the fingertips. These markers will be added to a single 
        # InteractiveMarkerControl, which in turn is added to your InteractiveMarker
        gripper_marker, l_finger_marker, r_finger_marker = get_gripper_markers(self._gripper_color)

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

    def _move_gripper(self):
        if self._pose_is_reachable:
            ps = PoseStamped()
            ps.header = self._i_marker.header
            ps.pose = self._i_marker.pose
            ps.header.frame_id = "base_link"
            #print(ps)
            self._arm.move_to_pose(ps)
        else:
            print("The currently selected pose is not reachable.\nPlease move marker to a location where it is green.")
 

    def handle_feedback(self, feedback):
        # We moved the gripper to a new pose, check to see if it's reachable.
        # Change the color to green if it is and red if it's not.
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            ps = PoseStamped()
            ps.header = feedback.header
            ps.pose = feedback.pose
            #self._last_pose = ps

            if self._arm.compute_ik(ps):
                self._gripper_color = ColorRGBA(0, 1.0, 0, 1.0)
                self._pose_is_reachable = True
            else:
                self._gripper_color = ColorRGBA(1.0, 0, 0, 1.0)
                self._pose_is_reachable = False
            
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
            if feedback.menu_entry_id == self._MENU_OPTIONS["Move gripper here"]:
                self._move_gripper()
            elif feedback.menu_entry_id == self._MENU_OPTIONS["Open gripper"]:
                self._gripper.open()
            elif feedback.menu_entry_id == self._MENU_OPTIONS["Close gripper"]:
                self._gripper.close()       


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._gripper_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        self._pregrasp_offset = [-0.12,0,0]
        self._lifted_offset = [0,0,0.06]
        self._obj_offset = [0.18,0,0]
        self._poses_are_reachable = True #TODO: actually calculate this. currently needs to be moved to actually work
        self._PRE_GRASP_IDS = [1,2,3]
        self._GRASP_IDS = [4,5,6]
        self._LIFTED_IDS = [7,8,9]


        self._br = tf.TransformBroadcaster()
        rospy.sleep(0.1)

    def start(self):
        obj_im = InteractiveMarker()
        obj_im.header.frame_id = "gripper_link" 
        obj_im.name = "Advanced Teleop Object Marker"
        obj_im.header.stamp = rospy.Time.now()
        
        # You will need to create 10 markers: 3 for each of 3 positions as
        # well as one for the object itself. The positions will be 
        # "pre-grasp", "grasp", and "lift". Each position will have one 
        # marker for the gripper and two for the fingertips. These markers 
        # will be added to a single InteractiveMarkerControl, which in turn is
        # added to your InteractiveMarker.

        # First we create a marker for the object to be grasped. A cube in this 
        # case.
        obj_marker = Marker()
        obj_marker.type = Marker.CUBE
        obj_marker.color = ColorRGBA(1.0, 1.0, 0, 1.0)
        obj_marker.scale.x = 0.065 # The cube
        obj_marker.scale.y = 0.065
        obj_marker.scale.z = 0.065
        obj_marker.pose.position.x = self._obj_offset[0]

        # Now we'll create the markers for each of the three positions
        gripper_marker, l_finger_marker, r_finger_marker = get_gripper_markers(self._gripper_color, [0,0,0], self._GRASP_IDS)
        pregrasp_gripper_marker, pregrasp_l_finger_marker, pregrasp_r_finger_marker = get_gripper_markers(self._gripper_color, self._pregrasp_offset, self._PRE_GRASP_IDS)
        lifted_gripper_marker, lifted_l_finger_marker, lifted_r_finger_marker = get_gripper_markers(self._gripper_color, self._lifted_offset, self._LIFTED_IDS)

        # Add the 6 dof controls to our interactive marker
        controls = make_6dof_controls()#gripper_im, gripper_marker)
        obj_im.controls.extend(controls)


        # Add the meshes (without any interaction mode) so we can display them
        obj_control = InteractiveMarkerControl()
        obj_control.interaction_mode = InteractiveMarkerControl.MENU
        obj_control.always_visible = True
        obj_control.markers.append(obj_marker)
        obj_control.markers.append(gripper_marker)
        obj_control.markers.append(l_finger_marker)
        obj_control.markers.append(r_finger_marker)
        obj_control.markers.append(pregrasp_gripper_marker)
        obj_control.markers.append(pregrasp_l_finger_marker)
        obj_control.markers.append(pregrasp_r_finger_marker)
        obj_control.markers.append(lifted_gripper_marker)
        obj_control.markers.append(lifted_l_finger_marker)
        obj_control.markers.append(lifted_r_finger_marker)
        obj_im.controls.append(obj_control)

        # Build the menu ("go to postion", "open gripper", "close gripper")
        self._build_menu(obj_im)

        # We'll need this in order to update its colors later
        self._i_marker = obj_im
        
        # Add the interactive marker to the server
        #self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.insert(self._i_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

        # obj_im = InteractiveMarker() ...
        #self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def _build_menu(self, i_marker):
        self._MENU_OPTIONS = {"Pick from front":1, "Open gripper":2}

        for key in self._MENU_OPTIONS.keys():
            menu_entry = MenuEntry()
            menu_entry.id = self._MENU_OPTIONS[key]
            menu_entry.parent_id = 0
            menu_entry.title = key
            menu_entry.command_type = MenuEntry.FEEDBACK
            i_marker.menu_entries.append(menu_entry)

    def handle_feedback(self, feedback):
        # We moved the gripper to a new pose, check to see if it's reachable.
        # Change the color to green if it is and red if it's not.
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            #print(feedback)
            red = ColorRGBA(1.0,0,0,1.0)
            green = ColorRGBA(0,1.0,0,1.0)
            grasp_color = pregrasp_color = lifted_color = green
            self._poses_are_reachable = True

            # Go through each of the three poses to see if they're reachable. 
            # Update colors accordingly
            for control in self._i_marker.controls:
                for marker in control.markers:
                    if marker.id != None:
                        #print(marker.id)
                
                        if marker.id == self._GRASP_IDS[0] or marker.id == self._PRE_GRASP_IDS[0] or marker.id == self._LIFTED_IDS[0]:
                            ps = PoseStamped()
                            ps.header = feedback.header
                            ps.pose = copy.deepcopy(feedback.pose)

                            if marker.id == self._PRE_GRASP_IDS[0]:
                                ps.pose.position.x += self._pregrasp_offset[0]
                                ps.pose.position.y += self._pregrasp_offset[1]
                                ps.pose.position.z += self._pregrasp_offset[2]
                            elif marker.id == self._LIFTED_IDS[0]:
                                ps.pose.position.x += self._lifted_offset[0]
                                ps.pose.position.y += self._lifted_offset[1]
                                ps.pose.position.z += self._lifted_offset[2]

                            if self._arm.compute_ik(ps) == False:
                                self._poses_are_reachable = False
                                if marker.id == self._GRASP_IDS[0]:
                                    grasp_color = red
                                elif marker.id == self._PRE_GRASP_IDS[0]:
                                    pregrasp_color = red
                                elif marker.id == self._LIFTED_IDS[0]:
                                    lifted_color = red
                
                for marker in control.markers:
                    if marker.id != None:
                        if marker.id == self._PRE_GRASP_IDS[0] or marker.id == self._PRE_GRASP_IDS[1] or marker.id == self._PRE_GRASP_IDS[2]:
                            marker.color = pregrasp_color
                        if marker.id == self._GRASP_IDS[0] or marker.id == self._GRASP_IDS[1] or marker.id == self._GRASP_IDS[2]:
                            marker.color = grasp_color
                        if marker.id == self._LIFTED_IDS[0] or marker.id == self._LIFTED_IDS[1] or marker.id == self._LIFTED_IDS[2]:
                            marker.color = lifted_color
            
            self._i_marker.header = feedback.header
            self._i_marker.pose = feedback.pose

            self._im_server.erase(self._i_marker.name)
            self._im_server.insert(self._i_marker, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == self._MENU_OPTIONS["Pick from front"]:
                self._pick_object()
            elif feedback.menu_entry_id == self._MENU_OPTIONS["Open gripper"]:
                self._gripper.open()

    def _pick_object(self):
        if self._poses_are_reachable:
            # Open gripper
            self._gripper.open()
            rospy.sleep(2)

            # Move to pre-grasp position
            ps = PoseStamped()
            ps.header = self._i_marker.header
            ps.header.frame_id = "base_link"
            ps.pose = self._build_pose(self._i_marker.pose, self._pregrasp_offset)
            self._arm.move_to_pose(ps)

            # Move to grasp position
            ps2 = PoseStamped()
            ps2.header = self._i_marker.header
            ps2.pose = self._i_marker.pose

            # Add an orientation constraint
            
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = 'wrist_roll_link'
            oc.orientation = ps2.pose.orientation
            oc.absolute_x_axis_tolerance = 0.2
            oc.absolute_y_axis_tolerance = 0.2
            oc.absolute_z_axis_tolerance = 0.2
            oc.weight = 1.0
            

            self._arm.move_to_pose(ps2)#, orientation_constraint = oc)

            # Close gripper
            rospy.sleep(1) # Make sure it's actually in position before we start closing
            self._gripper.close()
            rospy.sleep(3) # Make sure it's closed

            # Move to lifted position
            ps3 = PoseStamped()
            ps3.header = self._i_marker.header
            ps3.pose = self._build_pose(self._i_marker.pose, self._lifted_offset)
            self._arm.move_to_pose(ps3)#, orientation_constraint = oc)
            
        else:
            print("At least one of the currently selected poses is not reachable.\nPlease move the object such that all markers are green.")
    
    def _build_pose(self, base_pose, translation):
        # Create a matrix from a quaternion
        q = []
        q.append(base_pose.orientation.x)
        q.append(base_pose.orientation.y)
        q.append(base_pose.orientation.z)
        q.append(base_pose.orientation.w)
        base_matrix = tft.quaternion_matrix(q)

        # Add our pose to the matrix
        base_matrix[0][3] = base_pose.position.x
        base_matrix[1][3] = base_pose.position.y
        base_matrix[2][3] = base_pose.position.z

        transform_matrix = copy.deepcopy(base_matrix)
        transform_matrix[0][3] = translation[0]
        transform_matrix[1][3] = translation[1]
        transform_matrix[2][3] = translation[2]

        transformed_pose_matrix = np.dot(base_matrix, transform_matrix)
        position = tft.translation_from_matrix(transformed_pose_matrix)
        p2 = Pose()
        p2.position.x = position[0]
        p2.position.y = position[1]
        p2.position.z = position[2]

        p2.orientation.x = base_pose.orientation.x
        p2.orientation.y = base_pose.orientation.y
        p2.orientation.z = base_pose.orientation.z
        p2.orientation.w = base_pose.orientation.w

        return p2


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
    print("got here")
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()