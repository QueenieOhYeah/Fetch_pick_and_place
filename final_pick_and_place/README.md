### Lab34
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
