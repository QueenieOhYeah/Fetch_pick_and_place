#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf.transformations as tft
#import quaternion


def main():
    object_in_base_quaternion = [0,0,0.38268343,0.92387953]
    object_in_base_transformation = [[0.6],[-0.1],[0.7]]
    object_in_base_matrix = tft.quaternion_matrix(object_in_base_quaternion)
    object_in_base_matrix[0:3, -1:] = object_in_base_transformation
    #print(object_in_base_matrix)
    
    gripper_in_object_matrix = np.array([
        [1, 0, 0, -0.1],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    gripper_in_base_matrix = np.dot(object_in_base_matrix, gripper_in_object_matrix)
    pose = Pose()
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = tft.quaternion_from_matrix(gripper_in_base_matrix)
    (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ) = tft.translation_from_matrix(gripper_in_base_matrix)
    
    print(pose)
    
    
    
if __name__ == '__main__':
    main()
