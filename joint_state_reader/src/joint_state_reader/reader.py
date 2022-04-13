#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy     
import sensor_msgs.msg                                                                                      
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):
    	                                                                                
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.callback)
    def callback(self,data):
        self.data = data
                                                                                                
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                        
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """
        if not name:
            return None                                                                                            
        i = self.data.name.index(name)
        return self.data.position[i]
                                                                                 
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """                                                                     
        #rospy.logerr('Not implemented.')                                        
        return [self.get_joint(x) for x in names]
