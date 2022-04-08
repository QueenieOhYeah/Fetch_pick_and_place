#! /usr/bin/env python3

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy

from .arm_joints import ArmJoints


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # Create actionlib client
        ARM_NS = 'arm_controller/follow_joint_trajectory'
        self.client = actionlib.SimpleActionClient(ARM_NS, control_msgs.msg.FollowJointTrajectoryAction) 

        # Wait for server
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=trajectory_msgs.msg.JointTrajectory())
        goal.trajectory.joint_names = []
        goal.trajectory.points = []

        for joint_name, joint_pos in zip(arm_joints.names(), arm_joints.values()):
            # Create a trajectory point
            #print(f"joint: {joint_name}, val: {joint_pos}")
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            
            # Set position of trajectory point
            point.positions = [joint_pos]

            # Set time of trajectory point
            point.time_from_start = rospy.Duration(secs=5)

            # Add joint name to list
            goal.trajectory.joint_names.append(joint_name)
            
            # Add the trajectory point created above to trajectory
            goal.trajectory.points.append(point)

        # Send goal
        self.client.send_goal(goal)

        # Wait for result
        self.client.wait_for_result()
