#!/usr/bin/env python3

import actionlib
from math import pi
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
# What messages are we going to need?
import trajectory_msgs.msg
import rospy
import control_msgs.msg

class Head(object):
    JOINT_NAME_PAN = 'head_pan_joint'
    JOINT_NAME_TILT = 'head_tilt_joint'
    MIN_PAN = -pi/2
    MAX_PAN = pi/2
    MIN_TILT = -pi/2
    MAX_TILT = pi/4
    TIME_FROM_START = 2.5
    
    # What topics should we send trajectories to for the head and eyes?
    HEAD_NS = 'head_controller/follow_joint_trajectory'

    #def __init__(self, js, head_ns=None, eyes_ns=None):
    def __init__(self, head_ns=None):
        self.client = actionlib.SimpleActionClient(self.HEAD_NS, control_msgs.msg.FollowJointTrajectoryAction) 
        #self._js = js
        #self._head_gh = None
        #self._head_goal = None
        # TODO: What is the type of these actions? 
        #self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, ????????????)
        self.client.wait_for_server()

    def cancel(self):
        head_gh = self._head_gh
        eyes_gh = self._eyes_gh
        if head_gh:
            head_gh.cancel()
        self._head_goal = None
        self._head_gh = None
        if eyes_gh:
            eyes_gh.cancel()
        self._eyes_goal = None
        self._eyes_gh = None
        return "asdf"

    def is_done(self):
        active = {
         GoalStatus.PENDING, GoalStatus.RECALLING,
         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self._head_gh:
            if self._head_gh.get_goal_status() in active:
                return False
        if self._eyes_gh:
            if self._eyes_gh.get_goal_status() in active:
                return False
        return True

    def pan_and_tilt(self, pan, tilt, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's head to the point specified in the duration
        specified
        
        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
        and HeadClient.PAN_RIGHT
        
        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
        and HeadClient.TILT_DOWN
        
        :param duration: The amount of time to take to get the head to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        if pan >= self.MIN_PAN and pan <= self.MAX_PAN and tilt >= self.MIN_TILT and tilt <= self.MAX_TILT:
            # TODO: Build a JointTrajectoryPoint that expresses the target configuration
            pan_point = trajectory_msgs.msg.JointTrajectoryPoint()
            pan_point.positions = [pan]
            pan_point.time_from_start = rospy.Duration(secs=self.TIME_FROM_START)

            tilt_point = trajectory_msgs.msg.JointTrajectoryPoint()
            tilt_point.positions = [tilt]
            tilt_point.time_from_start = rospy.Duration(secs=self.TIME_FROM_START)
        
            # TODO: Put that point into the right container type, and target the 
            # correct joint.
            goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=trajectory_msgs.msg.JointTrajectory())
            goal.trajectory.joint_names = [self.JOINT_NAME_PAN, self.JOINT_NAME_TILT]
            goal.trajectory.points = [pan_point, tilt_point]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            #return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def send_trajectory(self, traj, feedback_cb=None, done_cb=None):
        """
        Sends the specified trajectories to the head and eye controllers
        
        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
        
        :param feedback_cb: A callable that takes one parameter - the feedback
        
        :param done_cb: A callable that takes two parameters - the goal status
        the goal handle result
        """
        for point in traj.points:
            for k in ('velocities', 'accelerations', 'effort'):
                if getattr(point, k) is None:
                    setattr(point, k, [])

            if isinstance(point.time_from_start, (int, float)):
                point.time_from_start = rospy.Duration(point.time_from_start)

        # TODO: What should be the type of the goal?
        #goal = control_msgs.msg.???????????(trajectory=traj)
        goal="asdf"

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if done_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                if gh.get_comm_state() == CommState.DONE:
                    done_cb(gh.get_goal_status(), gh.get_result())
            return

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                feedback_cb(feedback)
            return

        if self.JOINT_EYES in traj.joint_names:
            if not self._eyes_ac:
                return False
            self._eyes_goal = goal
            # TODO: How do we actually send the goal?
            #self._eyes_gh = self._eyes_ac.??????????(goal, _handle_transition, _handle_feedback)
        else:
            if not self._head_ac:
                return False
            self._head_goal = goal
            # TODO: How do we actually send the goal?
            #self._head_gh = self._head_ac.???????????(goal, _handle_transition, _handle_feedback)
        return True

    def shutdown(self):
        self.cancel()
        self._head_ac = None
        self._eyes_ac = None
        return

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()

        return False
