#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # To move the base, we're going to publish a Twist message to the
        # base_controller/command topic. Instead of publishing directly to it, 
        # we instead publish to cmd_vel because it can be overridden via
        # cmd_vel/teleop. This allows for the use of a deadman switch if it ever
        # goes off the rails. The queue_size argument ensures that if the robot
        # cannot keep up with the rate of messages being sent to it, we'll never
        # send a queue of more than 10 messages.
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Init our node. Our node's name is relatively unique but anonymous=True
        # guarantees its uniqueness by appending it with a hash of numbers so
        # that it doesn't conflict with any other nodes.
        #rospy.init_node('techin517_move_base', anonymous=True)

        # We're going to publish messages at a frequency of 10Hz
        self.rate = rospy.Rate(10)

        #while not rospy.is_shutdown():
        #    hello_str = "hello world %s" % rospy.get_time()
        #    rospy.loginfo(hello_str)
        #    pub.publish(hello_str)
        #    rate.sleep() # sleep for .1 seconds

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist msg
        msg = Twist()

        # Fill out msg
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        
        # Publish msg
        self.pub.publish(msg)
        
        #rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub.publish(msg)
        #rospy.logerr('Not implemented.')
