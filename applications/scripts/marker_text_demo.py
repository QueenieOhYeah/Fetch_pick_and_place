#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('my_node')
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    wait_for_time()
    
    # After you advertise a publisher in your code, you need to wait a little 
    # bit for the ROS master to notify other nodes and for those nodes to
    # start subscribing to you. All of this happens in the background, so you 
    # should just wait for a half second or so.
    rospy.sleep(0.5)
    
    show_text_in_rviz(marker_publisher, 'Hello world')

if __name__ == '__main__':
    main()

"""
MARKER MESSAGE FORMAT

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11
uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
    geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
geometry_msgs/Vector3 scale
    float64 x
    float64 y
    float64 z
std_msgs/ColorRGBA color
    float32 r
    float32 g
    float32 b
    float32 a
duration lifetime
bool frame_locked
geometry_msgs/Point[] points
    float64 x
    float64 y
    float64 z
std_msgs/ColorRGBA[] colors
    float32 r
    float32 g
    float32 b
    float32 a
string text
string mesh_resource
bool mesh_use_embedded_materials

"""