#!/usr/bin/env python 

import rospy
import csv
import perception_msgs.msg
from std_msgs.msg import String

class DatabaseReader(object): 
    """Read csv file and find the bin_id
    """
    def __init__(self, path):
        self.path = path
        self.sub = rospy.Subscriber("final_pick_and_place/object_name", String, self.process_input)
        self.pub = rospy.Publisher('final_pick_and_place/target', perception_msgs.msg.Target, queue_size=10)

    def find_bin(self, object_name):
        """Returns object name with bin id.
    
        Args:
            path: string, the path to a csv file with items with corresponding bin id.

        Returns: A tuple of object name and bin id
        """
        try:
            csv_file = csv.reader(open(self.path, "r"), delimiter=",")
            for row in csv_file:
                #if current rows 2nd value is equal to input, print that row
                if object_name == row[0]:
                    return object_name, row[1]
            return None


        except Exception as e:
            print(e)
            return None
            
    def process_input(self, data):
        result = self.find_bin(data.data)
        if not result:
          print ("No result found")
          return None
        object_name, bin_id = result[0], result[1]
        msg = perception_msgs.msg.Target()
        msg.name = object_name
        print(type(bin_id))
        msg.bin_id = int(bin_id)
        self.pub.publish(msg) 
         
          
def print_usage():
    print ('Process the input and find bin id')
    print ('Usage: rosrun final_pick_and_place read_database.py data/data.csv')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
            
        
def main():
    rospy.init_node('find_bin')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    path = argv[1]
    cvs_reader = DatabaseReader(path)
    rospy.spin()
#    rospy.Subscriber("object_name", String, cvs_reader.find_bin)
    
    


if __name__ == '__main__':
    main()
