#! /usr/bin/env python3

import rospy
import actionlib # import in order to use SimpleActionClient
import control_msgs.msg # import based on Fetch docs

ACTION_NAME = "gripper_controller/gripper_action"
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # Create actionlib client
        # The first arg name is the namespace. It is specified in the API but 
        # could also be found in the rqt_graph.
        # We know that based on the docs this client will use a GripperCommand
        # Action. Make sure to append Action onto the end of the command name.
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.GripperCommandAction)
        # 6 messages will be generated from GripperCommandAction. They are:
        #  - GripperCommandActionGoal.msg
        #  - GripperCommandActionResult.msg
        #  - GripperCommandActionFeedback.msg
        #  - GripperCommandGoal.msg
        #  - GripperCommandResult.msg
        #  - GripperCommandFeedback.msg


        # Wait until the server has started up and is listening for goals.
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        # Create goal
        # Why this over GripperCommandActionGoal?
        goal = control_msgs.msg.GripperCommandGoal(control_msgs.msg.GripperCommand(position=OPENED_POS))

        # Send goal
        self.client.send_goal(goal)

        # Wait for result
        self.client.wait_for_result()
        #rospy.logerr('Not implemented.')

    def close(self, max_gripper_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # Create goal
        goal = control_msgs.msg.GripperCommandGoal(control_msgs.msg.GripperCommand(position=CLOSED_POS, max_effort=max_gripper_effort))
        
        # Send goal
        self.client.send_goal(goal)
        
        # Wait for result
        self.client.wait_for_result()
        #rospy.logerr('Not implemented.')
