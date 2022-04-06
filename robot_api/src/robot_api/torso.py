#!/usr/bin/env python3

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy

# TODO: ACTION_NAME = ???
ACTION_NAME = 'torso_controller/follow_joint_trajectory'
# TODO: JOINT_NAME = ???
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5 # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        
        # Wait for server
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height >= self.MIN_HEIGHT and height <= self.MAX_HEIGHT:
            
            # Create a trajectory point
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            # Set position of trajectory point
            point.positions = [height]
            # Set time of trajectory point
            point.time_from_start = rospy.Duration(secs=TIME_FROM_START)

            # Create goal
            goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=trajectory_msgs.msg.JointTrajectory())
            # Add joint name to list
            goal.trajectory.joint_names = [JOINT_NAME]
            # Add the trajectory point created above to trajectory
            goal.trajectory.points = [point]

            # Send goal
            self.client.send_goal(goal)

            # Wait for result
            self.client.wait_for_result()
        else:
            print(f'Please specify a height between {self.MIN_HEIGHT} and {self.MAX_HEIGHT}')
