## Lab34
```
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
```
#### Transform cloud point to base_link frame
```
rosrun perception transform_cloud
```
#### Launch ar tag detection
```
roslaunch robot_api ar_desktop.launch cam_image_topic:=/transform_point_cloud
```
#### Smart cropper
```
rosrun final_pick_and_place smart_cropper.py
```
#### Get input and search for bin
```
rosrun final_pick_and_place read_database.py `rospack find final_pick_and_place`/data/data.csv
```
#### Launch perception
```
roslaunch perception object_detection2_final_real.launch data_dir:=`rospack find perception`/data/combined_labels_3
```
#### Pick and place
```
rosrun final_pick_and_place pick_and_place.py
```
#### Type in terminal to provide input
```
rostopic pub final_pick_and_place/object_name std_msgs/String medicinebottle(or something else)
```

## Run final project
```
roslaunch final_pick_and_place rviz_launch.launch
```
#### Transform cloud point to base_link frame
```
rosrun perception transform_cloud
```
#### Launch ar tag detection
```
roslaunch robot_api ar_desktop.launch cam_image_topic:=/transform_point_cloud
```
#### Get input and search for bin
```
rosrun final_pick_and_place read_database.py `rospack find final_pick_and_place`/data/data.csv
```
#### Launch perception
```
roslaunch perception object_detection2_final_real.launch data_dir:=`rospack find perception`/data/combined_labels_3
```
#### Run smart cropper
```
rosrun final_pick_and_place smart_cropper.py
```
### Select the mode
#### AI
```
rosparam set human 0
mask_rcnn.py
```
#### Human intelligence interface
```
rosparam set human 1
rosrun perception human.py
```
### Terminal interfaces

#### Input
```
rostopic pub final_pick_and_place/object_name std_msgs/String green_covers(or something else)
```
#### Check if input is received
```
rostopic echo final_pick_and_place/target
```
#### Check the detected pose
```
rostopic /targeted_object
```

## Initialization
```
roslaunch applications nav_rviz.launch
roslaunch robot_api ar_desktop.launch cam_image_topic:=/head_camera/depth_registered/points
rosrun final_pick_and_place initialize.py
```
