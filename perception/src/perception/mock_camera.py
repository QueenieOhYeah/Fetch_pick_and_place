#!/usr/bin/env python
import rosbag


class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
        

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        try:
            bag = rosbag.Bag(path)
            for topic, t, msg on bag.read_messages():
                print(msg)
            bag.close()
            return msg
        except:
            return None
