#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from map_annotator.srv import *

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


#def main():
#    rospy.init_node('pickle_client')
#    wait_for_time()
#    rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback)
#    
#    
#    reader = JointStateReader()

#    rospy.sleep(0.5)

#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        # TODO: get torso joint value
#        torso_height = reader.get_joint("torso_lift_joint")
#        # TODO: publish torso joint value
#        torso_pub.publish(torso_height)
#        rate.sleep()

#class PickleClient(object):
#    def __init__(self):
#        None
##        rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)   
#        
##    def callback(self, data):
##        self._data = data
#        
def list_saved_poses():
    rospy.wait_for_service('map_annotator/list')
    try: 
        srv = rospy.ServiceProxy('map_annotator/list', List)
        l = srv()
        return l.names
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
def save_pose(name):
    rospy.wait_for_service('map_annotator/save')
#   pos = geometry_msgs.msg.PoseStamped()
#   pos.header = self._data.header
#   pos.pose = self._data.pose.pose
    try: 
        save = rospy.ServiceProxy('map_annotator/save', Save)
        result = save(name)
        return result.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
        
def delete_pose(name):
    rospy.wait_for_service('map_annotator/delete')
    try: 
        delete = rospy.ServiceProxy('map_annotator/delete', Delete)
        result = delete(name) #0-no such pose
        return result.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
def goto_pose(name):
    rospy.wait_for_service('map_annotator/goto')
    try: 
        goto = rospy.ServiceProxy('map_annotator/goto', Goto)
        result = goto(name)
        return result.result   #0-no such pose             
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node('pickle_client')
    message = "Welcome to the map annotator! \n Commands: \n list: List saved poses. \n save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists. \n delete <name>: Delete the pose given by <name>. \n goto <name>: Sends the robot to the pose given by <name>. \n help: Show this list of commands"
    wait_for_time()
    #client = PickleClient()
    print(message)
   
    while not rospy.is_shutdown():
        command = input(">")
        words = command.split(" ",1)
#        if len(words) > 2:
#            print("Too many parameters are given")
#            continue
        if len(words) == 1 and words[0] == "help":
            print(message)
            continue
        if words[0] == "help":
            print("Too many parameters are given")
            continue
        if len(words) == 1 and words[0] == "list":
            l = list_saved_poses()
            if len(l) == 0:
                print("No poses")
                continue
            print("Poses: \n") 
            print(*l, sep = '\n')
            continue
        if words[0] == "list":
            print("Too many parameters are given")
            continue
           
        if words[0] == "save":
            name = words[1]
            save_pose(name)
            continue
        if words[0] == "delete":
            name = words[1]
            result = delete_pose(name)
            if not result:
                print("No such pose " + name)
            continue
        if words[0] == "goto":
            name = words[1]
            result = goto_pose(name)
            if not result:
                print("No such pose '" + name + "'")
            continue
        print("invalid command")

        
            
        
      
if __name__ == '__main__':
    main()
