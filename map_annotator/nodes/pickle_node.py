#!/usr/bin/env python

import rospy
import pickle
import geometry_msgs.msg
from map_annotator.srv import Save, List, Delete, Goto


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def pickling(d: dict):
    filename = "fetch_position"
    outfile = open(filename, 'wb')
    pickle.dump(d, outfile)
    outfile.close()
    

  
def unpickling():
    filename = "fetch_position"
    try:   
        infile = open(filename, 'rb')
        obj = pickle.load(infile)
        infile.close()
        return obj
    except:
        return None

class PickleNode(object):
    def __init__(self):
        self._dict = unpickling()
        if self._dict == None:
            self._dict = {}
        rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)
        self._pub = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped)      
          
        
        
    def callback(self, data):
        self._data = data
        
    def list_saved_poses(self, request):
        return self._dict.keys()
        
    def save_pose(self, request):
        pos = geometry_msgs.msg.PoseStamped()
        pos.header = self._data.header
        pos.pose = self._data.pose.pose
        self._dict[request.name] = pos        
        pickling(self._dict)
        return 1
        
    def delete_pose(self, request):
        if request.name not in self._dict:
            return 0
        self._dict.pop(request.name)
        pickling(self._dict)
        return 1
        
    def goto_pose(self, request):
        if request.name not in self._dict:
            return 0
        pos = self._dict[request.name]
        self._pub.publish(pos)
        return 1

def main():
    rospy.init_node('pickle_server')
    wait_for_time()
    
    
    server = PickleNode()
    save_service = rospy.Service('map_annotator/save', Save,
                                  server.save_pose)
    list_service = rospy.Service('map_annotator/list', List,
                                  server.list_saved_poses)
    delete_service = rospy.Service('map_annotator/delete', Delete,
                                  server.delete_pose)
    goto_service = rospy.Service('map_annotator/goto', Goto,
                                  server.goto_pose)                                                               
                     
    rospy.spin()


if __name__ == '__main__':
    main()
