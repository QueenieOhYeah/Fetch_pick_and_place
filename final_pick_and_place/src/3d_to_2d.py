#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import perception


def callback(data):
    
def main():
    rospy.init_node('3d_to_2d')
    wait_for_time()
    marker_sub = rospy.Subscriber('mock_point_cloud', PointCloud2, callback) 
    rospy.spin()

if __name__ == '__main__':
    main()
