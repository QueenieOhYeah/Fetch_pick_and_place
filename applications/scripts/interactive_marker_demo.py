#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('my_interactive_marker_node')
    #marker_publisher = rospy.Publisher('visualization_marker', Marker)
    wait_for_time()
    
    # After you advertise a publisher in your code, you need to wait a little 
    # bit for the ROS master to notify other nodes and for those nodes to
    # start subscribing to you. All of this happens in the background, so you 
    # should just wait for a half second or so.
    #rospy.sleep(0.5)

    # First, create the server.
    server = InteractiveMarkerServer("simple_marker")

    # Then, create an InteractiveMarker.
    forward_marker = InteractiveMarker()
    forward_marker.header.frame_id = "base_link"
    forward_marker.name = "forward_marker"
    forward_marker.description = "Forward Click Control"
    forward_marker.pose.position.x = 1
    forward_marker.pose.orientation.w = 1

    # Next, create a teal cube Marker for the InteractiveMarker.
    forward_cyl_marker = Marker()
    forward_cyl_marker.type = Marker.CYLINDER
    forward_cyl_marker.pose.orientation.w = 1
    forward_cyl_marker.scale.x = 0.45
    forward_cyl_marker.scale.y = 0.45
    forward_cyl_marker.scale.z = 0.45
    forward_cyl_marker.color.r = 0.0
    forward_cyl_marker.color.g = 0.5
    forward_cyl_marker.color.b = 0.5
    forward_cyl_marker.color.a = 1.0

    # Next create an InteractiveMarkerControl, add the Marker to it, and add
    # the control to the InteractiveMarker.
    forward_button_control = InteractiveMarkerControl()
    forward_button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    forward_button_control.always_visible = True
    forward_button_control.markers.append(forward_cyl_marker)
    _marker.controls.append(forward_button_control)
    
    server.insert(int_marker, handle_viz_input)
    server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
    main()

