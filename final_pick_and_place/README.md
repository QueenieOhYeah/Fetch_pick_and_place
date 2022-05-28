### Testing
```
roscore
rosrun applications publish_saved_cloud.py /home/dell/catkin_ws/src/fetch-picker/perception/src/data/whole1.bag
rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
```
#### Get input and search for bin
```
rosrun final_pick_and_place read_database.py /home/dell/catkin_ws/src/fetch-picker/final_pick_and_place/data/data.csv
```
#### Launch perception
```
roslaunch perception object_detection2_final.launch data_dir:=/home/dell/catkin_ws/src/fetch-picker/perception/src/data/color_labels
```
#### Pick and place
```
rosrun final_pick_and_place pick_and_place.py
```
#### Type in terminal to provide input
```
rostopic pub final_pick_and_place/object_name std_msgs/String medicinebottle
```
