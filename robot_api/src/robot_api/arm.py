import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


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
        self._client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Wait for server
        self._client.wait_for_server(rospy.Duration(10))

        # Create a MoveGroupAction action client
        self._move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

        # Wait for server
        self._move_group_client.wait_for_server(rospy.Duration(10))

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        p = JointTrajectoryPoint()
        # Set position of trajectory point
        p.positions = arm_joints.values()
        # Set time of trajectory point
        p.time_from_start = rospy.Duration(5)
        # Create goal
        goal = FollowJointTrajectoryGoal()
        # Add joint name to list
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = arm_joints.names()
        # Add the trajectory point created above to trajectory
        goal.trajectory.points.append(p)
        # Send goal
        self._client.send_goal(goal)
        # Wait for result
        self._client.wait_for_result()

    def _error_code_to_string(self, error_code):
        errors = {
         1: "SUCCESS",
         99999: "FAILURE",
         -1: "PLANNING_FAILED",
         -2: "INVALID_MOTION_PLAN",
         -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
         -4: "CONTROL_FAILED",
         -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
         -6: "TIMED_OUT",
         -7: "PREEMPTED",
         -10: "START_STATE_IN_COLLISION",
         -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
         -12: "GOAL_IN_COLLISION",
         -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
         -14: "GOAL_CONSTRAINTS_VIOLATED",
         -15: "INVALID_GROUP_NAME",
         -16: "INVALID_GOAL_CONSTRAINTS",
         -17: "INVALID_ROBOT_STATE",
         -18: "INVALID_LINK_NAME",
         -19: "INVALID_OBJECT_NAME",
         -21: "FRAME_TRANSFORM_FAILURE",
         -22: "COLLISION_CHECKING_UNAVAILABLE",
         -23: "ROBOT_STATE_STALE",
         -24: "SENSOR_INFO_STALE",
         -25: "COMMUNICATION_FAILURE",
         -31: "NO_IK_SOLUTION"
        }

        if error_code in errors.keys():
            return errors[error_code]
        return "Unidentified code"

    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint=None):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
                allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)        
        goal = goal_builder.build()

        self._move_group_client.send_goal(goal)
        print("Will wait for moveit")
        self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        print("...DONE...")
        result = self._move_group_client.get_result()
        
        if result is not None:
            resultStr = self._error_code_to_string(result.error_code.val)

            if resultStr == "SUCCESS":
                return None
            return resultStr
        else:
            print("Why is this None the first pass through?")

    def check_pose(self,
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    def cancel_all_goals(self):
        self._client.cancel_all_goals() # Your action client from Lab 7
        self._move_group_client.cancel_all_goals() # From this lab

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = self._error_code_to_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True
