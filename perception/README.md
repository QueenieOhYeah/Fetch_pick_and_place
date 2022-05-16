## Test segmentation
```
roscore
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
```
## Test smart cropper
```
roslaunch fetch_gazebo playground.launch
rosrun applications publish_saved_cloud.py shelf.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
roslaunch robot_api ar_desktop.launch cam_image_topic:=mock_point_cloud
rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
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
