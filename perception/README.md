## Test segmentation
```
roscore
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
```
## Test smart cropper
### Simulation
```
roslaunch fetch_gazebo playground.launch
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
roslaunch robot_api ar_desktop.launch cam_image_topic:=mock_point_cloud
rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
rosrun applications smart_cropper.py
```
### Real robot
```
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
roslaunch robot_api ar_desktop.launch cam_image_topic:=head_camera/depth_registered/points
rosrun perception point_cloud_demo cloud_in:=head_camera/depth_registered/points
rosrun applications smart_cropper.py
```
## Reaching a marker
```
roslaunch fetch_gazebo playground.launch
roslaunch fetch_moveit_config move_group.launch
rosrun applications torso_demo.py 0.4
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
roslaunch robot_api ar_desktop.launch cam_image_topic:=mock_point_cloud
rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
rosrun applications hallucinated_reach.py

```
## Test object detection
```
roscore
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
roslaunch perception object_detection.launch data_dir:=/home/dell/catkin_ws/src/fetch-picker/perception/src/data/labels
```
