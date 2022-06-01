#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        self.point_pub = rospy.Publisher("points_by_human",Int32MultiArray, queue_size = 1) #Only publish parameters
        self.cv_image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.callback)
        self.window_name = "Image window"
        self.pt1 = None 
        self.pt2 = None
        self.double_click_pt = None
        self.drawing = False
    
    
    def callback(self,data):
         try:
#             if self.cv_image is None:
             if True:
                 self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                 self.open_window()
                 
                 if (self.pt1 and self.pt2):
                     cv2.rectangle(self.cv_image, pt1 = self.pt1,
                          pt2 = self.pt2,
                          color =(0, 255, 255),
                          thickness = 2)
                 if self.double_click_pt:
#                     print(self.pt1,self.pt2)
                     cv2.circle(self.cv_image, self.double_click_pt, 10, (0, 255, 255), -1)
                     
                     
                 cv2.imshow(self.window_name, self.cv_image)
             else:
                 print("no image yet")
#                 self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#                 if (self.pt1 and self.pt2):
#                     cv2.rectangle(self.cv_image, pt1 = self.pt1,
#                          pt2 = self.pt2,
#                          color =(0, 255, 255),
#                          thickness = 2)
#                 if self.double_click_pt:
#                     cv2.circle(self.cv_image, self.double_click_pt, 10, 255)
#                 cv2.imshow(self.window_name, self.cv_image)
         except CvBridgeError as e:
             print("CvBridgeError")
             print(e)
         cv2.waitKey(3)
              
    def get_cv_image(self):
        if self.cv_image is None:
            return False
        return True  
    
    def open_window(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.draw_rectangle_with_drag)
            
    def draw_rectangle_with_drag(self, event, x, y, flags, param):
      
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.pt1 = (x, y)
            self.pt2 = None  
              
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.pt2 = (x, y)
      
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.pt2 = (x, y)
            
            top_right_point = (min(self.pt1[0], self.pt2[0]), min(self.pt1[1], self.pt2[1]))
            buttom_left_point = (max(self.pt1[0], self.pt2[0]), max(self.pt1[1], self.pt2[1]))
            
            rospy.set_param("top_right_x", top_right_point[0])
            rospy.set_param("top_right_y", top_right_point[1])
            rospy.set_param("buttom_left_x", buttom_left_point[0])
            rospy.set_param("buttom_left_y", buttom_left_point[1])

            
            #TODO: Set params
#            msg = Int32MultiArray()
#            msg.data.append(self.pt1[0])
#            msg.data.append(self.pt1[1])
#            msg.data.append(x)
#            msg.data.append(y)
#            print(msg)
#            self.point_pub.publish(msg)


        elif event == cv2.EVENT_LBUTTONDBLCLK:
            self.double_click_pt = (x, y)
            # TODO: Set úser_double_click_x úser_double_click_y params
    

def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        while not ic.get_cv_image():
            continue
        #ic.open_window()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    

