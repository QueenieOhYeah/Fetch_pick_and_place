#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

# TODO: ACTION_NAME = ???
# TODO: JOINT_NAME = ???
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            raise Exception("INVALID HEIGHT")
        # TODO: Create a trajectory point
        p = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        p.positions.append(height)
        # TODO: Set time of trajectory point
        p.time_from_start = rospy.Duration(5)

        # TODO: Add joint name to list
        t = JointTrajectory()
        t.joint_names.append("torso_lift_joint")
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        # TODO: Add the trajectory point created above to trajectory
        t.points.append(p)
        goal.trajectory = t
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        print("sending")
        result = self._client.wait_for_result()
        print(f"result: {result}")
        #rospy.logerr('Not implemented.')
