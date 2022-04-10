#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import math
import rospy

LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -1.5708  # TODO: Minimum pan angle, in radians.
    MAX_PAN = 1.5708  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -0.785398  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = 1.5708  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self._client1 = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self._client2 = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for both servers
        self._client1.wait_for_server(rospy.Duration(10))
        self._client2.wait_for_server(rospy.Duration(10))

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        goal = control_msgs.msg.PointHeadGoal()
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        goal.target = geometry_msgs.msg.PointStamped()
        goal.target.header = std_msgs.msg.Header()
        goal.target.header.frame_id = frame_id
        goal.pointing_axis = geometry_msgs.msg.Vector3()
        goal.pointing_axis.x = x
        goal.pointing_axis.y = y
        goal.pointing_axis.z = z
        goal.min_duration = rospy.Duration(1)
        goal.max_velocity = 1.57

        # TODO: Send the goal
        self._client1.send_goal(goal)
        # TODO: Wait for result
        self._client1.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        if pan > self.MAX_PAN or pan < self.MIN_PAN or tilt > self.MAX_TILT or tilt < self.MIN_TILT:
            raise Exception("INVAID PARAMETER")
        # TODO: Create a trajectory point
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set positions of the two joints in the trajectory point
        p.positions.append(pan)
        p.positions.append(tilt)
        # TODO: Set time of the trajectory point
        p.time_from_start = rospy.Duration(PAN_TILT_TIME)

        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint names to the list
        goal.trajectory = trajectory_msgs.msg.JointTrajectory()
        goal.trajectory.joint_names.append(PAN_JOINT)
        goal.trajectory.joint_names.append(TILT_JOINT)
        # TODO: Add trajectory point created above to trajectory
        goal.trajectory.points.append(p)
        # TODO: Send the goal
        self._client2.send_goal(goal)
        # TODO: Wait for result
        self._client2.wait_for_result()
        #rospy.logerr('Not implemented.')

#import actionlib
#from actionlib_msgs.msg import GoalStatus
#from actionlib.action_client import CommState
## TODO: What messages are we going to need?
#import ?????????_msgs.msg, rospy, ???????_msgs.msg

#""" 
#This is set up for Kuri, but you can take inspiration for Fetch if you like.
#"""
#class Head(object):
#    JOINT_PAN = 'head_1_joint'
#    JOINT_TILT = 'head_2_joint'
#    JOINT_EYES = 'eyelids_joint'
#    JOINT_HEIGHT = 0.405
#    PAN_LEFT = 0.78
#    PAN_NEUTRAL = 0
#    PAN_RIGHT = -PAN_LEFT
#    TILT_UP = -0.92
#    TILT_NEUTRAL = 0.0
#    TILT_DOWN = 0.29
#    EYES_OPEN = 0.0
#    EYES_NEUTRAL = 0.1
#    EYES_CLOSED = 0.41
#    EYES_HAPPY = -0.16
#    EYES_SUPER_SAD = 0.15
#    EYES_CLOSED_BLINK = 0.35
#    # TODO: Aw shucks, ????? again?!
#    # What topics should we send trajectories to for the head and eyes?
#    HEAD_NS = '??????????????????????????????????????'
#    EYES_NS = '??????????????????????????????????????'

#    def __init__(self, js, head_ns=None, eyes_ns=None):
#        self._js = js
#        self._head_gh = None
#        self._head_goal = None
#        # TODO: What is the type of these actions? 
#        self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, ????????????)
#        self._eyes_gh = None
#        self._eyes_goal = None
#        self._eyes_ac = actionlib.ActionClient(eyes_ns or self.EYES_NS, ????????????)
#        return

#    def cancel(self):
#        head_gh = self._head_gh
#        eyes_gh = self._eyes_gh
#        if head_gh:
#            head_gh.cancel()
#        self._head_goal = None
#        self._head_gh = None
#        if eyes_gh:
#            eyes_gh.cancel()
#        self._eyes_goal = None
#        self._eyes_gh = None
#        return

#    def eyes_to(self, radians, duration=1.0, feedback_cb=None, done_cb=None):
#        """
#        Moves the robot's eye lids to the specified location in the duration
#        specified
#        
#        :param radians: The eye position.  Expected to be between
#        HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
#        
#        :param duration: The amount of time to take to get the eyes to
#        the specified location.
#        
#        :param feedback_cb: Same as send_trajectory's feedback_cb
#        
#        :param done_cb: Same as send_trajectory's done_cb
#        """
#        # TODO: Build a JointTrajectoryPoint that expresses the target configuration
#        # TODO: Put that point into the right container type, and target the 
#        # correct joint.
#        return self.send_trajectory(trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

#    def is_done(self):
#        active = {
#         GoalStatus.PENDING, GoalStatus.RECALLING,
#         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
#        if self._head_gh:
#            if self._head_gh.get_goal_status() in active:
#                return False
#        if self._eyes_gh:
#            if self._eyes_gh.get_goal_status() in active:
#                return False
#        return True

#    def pan_and_tilt(self, pan, tilt, duration=1.0, feedback_cb=None, done_cb=None):
#        """
#        Moves the robot's head to the point specified in the duration
#        specified
#        
#        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
#        and HeadClient.PAN_RIGHT
#        
#        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
#        and HeadClient.TILT_DOWN
#        
#        :param duration: The amount of time to take to get the head to
#        the specified location.
#        
#        :param feedback_cb: Same as send_trajectory's feedback_cb
#        
#        :param done_cb: Same as send_trajectory's done_cb
#        """
#         # TODO: Build a JointTrajectoryPoint that expresses the target configuration
#        # TODO: Put that point into the right container type, and target the 
#        # correct joint.
#        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

#    def send_trajectory(self, traj, feedback_cb=None, done_cb=None):
#        """
#        Sends the specified trajectories to the head and eye controllers
#        
#        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
#        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
#        
#        :param feedback_cb: A callable that takes one parameter - the feedback
#        
#        :param done_cb: A callable that takes two parameters - the goal status
#        the goal handle result
#        """
#        for point in traj.points:
#            for k in ('velocities', 'accelerations', 'effort'):
#                if getattr(point, k) is None:
#                    setattr(point, k, [])

#            if isinstance(point.time_from_start, (int, float)):
#                point.time_from_start = rospy.Duration(point.time_from_start)

#        # TODO: What should be the type of the goal?
#        goal = control_msgs.msg.???????????(trajectory=traj)

#        def _handle_transition(gh):
#            gh_goal = gh.comm_state_machine.action_goal.goal
#            if done_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
#                if gh.get_comm_state() == CommState.DONE:
#                    done_cb(gh.get_goal_status(), gh.get_result())
#            return

#        def _handle_feedback(gh, feedback):
#            gh_goal = gh.comm_state_machine.action_goal.goal
#            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
#                feedback_cb(feedback)
#            return

#        if self.JOINT_EYES in traj.joint_names:
#            if not self._eyes_ac:
#                return False
#            self._eyes_goal = goal
#            # TODO: How do we actually send the goal?
#            self._eyes_gh = self._eyes_ac.??????????(goal, _handle_transition, _handle_feedback)
#        else:
#            if not self._head_ac:
#                return False
#            self._head_goal = goal
#            # TODO: How do we actually send the goal?
#            self._head_gh = self._head_ac.???????????(goal, _handle_transition, _handle_feedback)
#        return True

#    def shutdown(self):
#        self.cancel()
#        self._head_ac = None
#        self._eyes_ac = None
#        return

#    def wait_for_server(self, timeout=rospy.Duration(0.0)):
#        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

#    def wait_for_done(self, timeout):
#        rate = rospy.Rate(10)
#        start_time = rospy.Time.now()
#        while rospy.Time.now() - start_time < rospy.Duration(timeout):
#            if self.is_done():
#                return True
#            rate.sleep()

#        return False
