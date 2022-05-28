#!/usr/bin/env python 

import rospy
import csv
from read_database import DatabaseReader
import perception_msgs.msg

def print_usage():
    print ('Start the server of pick and place')
    print ('Usage: rosrun final_pick_and_place pick_and_place.py')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
        
def callback(data):
    print(data)            
        
def main():
    rospy.init_node('pick_and_place')
    wait_for_time()
    argv = rospy.myargv()
#    if len(argv) < 2:
#        print_usage()
#        return
#    path = float(argv[1])
    
    rospy.Subscriber('final_pick_and_place/target', perception_msgs.msg.Target, callback)
    # STructure of Target msg:
    # string name
    # int32 bin_id #locate in which bin
    
    rospy.Subscriber('targeted_object', perception_msgs.msg.Object, callback)
    rospy.spin()
    


if __name__ == '__main__':
    main()
