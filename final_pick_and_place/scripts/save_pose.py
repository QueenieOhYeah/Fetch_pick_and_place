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
    rospy.init_node('save_ar_pose')
    wait_for_time()
    marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback) 
    rospy.spin()

if __name__ == '__main__':
    main()
