#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class JointStateReader(object):
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """

    def joint_callback(self, data): 
        #joint_names = data.names
        #joint_positions = data.position
        #self.joints = data
        for joint_name, joint_position in zip(data.names, data.position):
            self.joints[joint_name] = joint_position

    def __init__(self):
        self.joints = {}
        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_callback)
        #rospy.spin()                                                                                                                                                            
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                                                            
        if name in self.joints.keys():
            return self.joints[name]                                                          
        return None                                                                                      
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """
        joints = []
        for name in names:
            joints.append(self.get_joint(name))
        return joints                               
        #return [0 for x in names]
