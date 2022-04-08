#!/usr/bin/env python3

import actionlib
from math import pi
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
# What messages are we going to need?
import trajectory_msgs.msg
import geometry_msgs.msg
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
    PAN_NS = 'head_controller/follow_joint_trajectory'
    POINT_NS = 'head_controller/point_head'

    def __init__(self):
        # Init clients
        self.pan_client = actionlib.SimpleActionClient(self.PAN_NS, control_msgs.msg.FollowJointTrajectoryAction) 
        self.point_client = actionlib.SimpleActionClient(self.POINT_NS, control_msgs.msg.PointHeadAction)

        # Wait for servers
        self.pan_client.wait_for_server()
        self.point_client.wait_for_server()


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
            # Build a JointTrajectoryPoint that expresses the target configuration
            pan_point = trajectory_msgs.msg.JointTrajectoryPoint()
            pan_point.positions = [pan]
            pan_point.time_from_start = rospy.Duration(secs=self.TIME_FROM_START)

            tilt_point = trajectory_msgs.msg.JointTrajectoryPoint()
            tilt_point.positions = [tilt]
            tilt_point.time_from_start = rospy.Duration(secs=self.TIME_FROM_START)
        
            # Put that point into the right container type, and target the 
            # correct joint.
            goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=trajectory_msgs.msg.JointTrajectory())
            goal.trajectory.joint_names = [self.JOINT_NAME_PAN, self.JOINT_NAME_TILT]
            goal.trajectory.points = [pan_point, tilt_point]

            self.pan_client.send_goal(goal)
            self.pan_client.wait_for_result()

    def look_at(self, frame_id, x, y, z):
        # Build a Point
        look_point = geometry_msgs.msg.PointStamped()
        look_point.point = geometry_msgs.msg.Point(x, y, z)
        look_point.header.frame_id = frame_id

        # Put that point into the right container type, and target the correct
        # frame
        goal = control_msgs.msg.PointHeadGoal()
        goal.target = look_point
        #goal.pointing_frame = frame_id
        goal.min_duration = rospy.Duration(secs=self.TIME_FROM_START)

        self.point_client.send_goal(goal)
        self.point_client.wait_for_result()
