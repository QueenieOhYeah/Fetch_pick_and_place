#! /usr/bin/env python

# TODO: import ????????_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import copy
import math
import tf.transformations as tft


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Create publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscribe to odom
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback, queue_size=10)


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
        # TODO: Create Twist msg
        cmd = Twist()
        # TODO: Fill out msg
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        # TODO: Publish msg
        self.pub.publish(cmd)
        #rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        cmd = Twist()
        # Fill out msg
#        cmd.linear.x = 0
#        cmd.linear.z = 0
        # Publish msg
        self.pub.publish(cmd)
        #rospy.logerr('Not implemented.')

    def _odom_callback(self, msg):
        self._latest_odom = msg
        pass

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
             means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        rospy.sleep(0.5)

        # record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(10)
        
        # Check if the robot has traveled the desired distance
        # Be sure to handle the case where the distance is negative!
        while abs(math.dist([start.pose.pose.position.x, start.pose.pose.position.y], [self._latest_odom.pose.pose.position.x, self._latest_odom.pose.pose.position.y])) < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            # Do we?

            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()


    def _quaternion_to_yaw(self, q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0] # The x value of the x-axis (first column)
        y = m[1, 0] # The y value of the x-axis
        theta_rads = math.atan2(y, x)
        #theta_degs = theta_rads * 180 / math.pi
        return theta_rads

    def _rotate_incomplete(self, start, current, angular_distance):
        if start < 0:
            start += 2 * math.pi
        if current < 0:
            current += 2 * math.pi   

        stop = start + angular_distance 

        # Angular rotation is positive
        if angular_distance > 0:
            # Rotation will cross the 0 line
            if stop > 2 * math.pi:
                stop -= 2 * math.pi
                #print(f"cross start: {start:.2f}, stop: {stop:.2f}, current: {current:.2f}")
                
                # Haven't crossed over the 0 line yet
                if current >= start:
                    return True

                # Have crossed the 0 line 
                return current < stop

            # Rotation will not cross the 0 line
            else:
                #print(f"no cross start: {start:.2f}, stop: {stop:.2f}, current: {current:.2f}")
                return current < stop

        # Angular rotation is negative
        else:
            # Rotation will cross the 0 line
            if stop < 0:
                stop += 2 * math.pi

                # Haven't crossed over the 0 line yet
                if current <= start:
                    return True
                
                # Have crossed the 0 line 
                return current > stop

            # Rotation will not cross the 0 line
            else:
                return current > stop


    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        rospy.sleep(0.5)

        # Record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._latest_odom)
        start_yaw = self._quaternion_to_yaw(start.pose.pose.orientation)

        # What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        if angular_distance > 2 * math.pi:
            angular_distance %= 2 * math.pi
        if angular_distance < -2 * math.pi:
            angular_distance %= -2 * math.pi

        rate = rospy.Rate(10)
        # Check if the robot has rotated the desired amount
        # Be sure to handle the case where the desired amount is negative!
        while self._rotate_incomplete(start_yaw, self._quaternion_to_yaw(self._latest_odom.pose.pose.orientation), angular_distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            # Do we?
            
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

        

    