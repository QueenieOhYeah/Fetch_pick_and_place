#!/usr/bin/env python

import rospy
import pickle
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


FLAG = 1

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def callback(msg):
    markers = msg.markers
    for marker in markers:
        if marker.id == 1 and FLAG:
            filename = "ar_1_pose"
            outfile = open(filename, 'wb')
            pickle.dump(marker, outfile)
            outfile.close()
            print("SAVE")
#            infile = open(filename, 'rb')
#            obj = pickle.load(infile)
#            infile.close()

def main():
    rospy.init_node('resave')
    wait_for_time()
    filename = "/home/dell/catkin_ws/src/fetch-picker/final_pick_and_place/data/ar_1_pose"
    infile = open(filename, 'rb')
    obj = pickle.load(infile)
    infile.close()
    print(obj)
    outfile = open("/home/dell/catkin_ws/src/fetch-picker/final_pick_and_place/data/ar_2_pose", 'wb')
    pickle.dump(obj, outfile, 2)
    outfile.close()
    rospy.spin()

if __name__ == '__main__':
    main()
