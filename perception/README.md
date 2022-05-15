#Test segmentation
### roscore
### rosrun applications publish_saved_cloud.py shelf.bag
### rosrun rviz rviz -d `rospack find applications`/config/marker.rviz
### rosrun perception point_cloud_demo cloud_in:=mock_point_cloud
