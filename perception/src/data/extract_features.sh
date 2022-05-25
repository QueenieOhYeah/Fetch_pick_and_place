. /opt/ros/noetic/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf combined_labels
mkdir combined_labels
rosrun perception extract_features pillow.bag pillow
rosrun perception extract_features gloves.bag gloves
rosrun perception extract_features box.bag box
rosrun perception extract_features medicine.bag medicine
rosrun perception extract_features medicinebottle.bag medicinebottle
rosrun perception extract_features suction.bag suction
rosrun perception extract_features ball.bag ball


mv *_label.bag combined_labels
