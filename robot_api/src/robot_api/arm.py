# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
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
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        p = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        p.positions = arm_joints.values()
        # TODO: Set time of trajectory point
        p.time_from_start = rospy.Duration(5)
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = arm_joints.names()
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(p)
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
