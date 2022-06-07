#!/usr/bin/env python3
import csv
# from detectron2.engine import DefaultTrainer
# import some common libraries
import numpy as np
import os, json, cv2, random
import rospy
#import cv2_imshow
import matplotlib.pyplot as plt
from perception_msgs.msg import ObjectList, Indices
from sensor_msgs.msg import Image
#from cv_bridge.boost.cv_bridge_boost import getCvType
from std_msgs.msg import String, Int32MultiArray
# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
#note that img is using cv2.read to read the img path

pub = None

def seg_output(img, model_weights_path):
  #load the model
  cfg = get_cfg()
  cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml"))
  # cfg.MODEL.WEIGHTS = "/content/drive/MyDrive/517 robotics project/FormalDataSet/moretrain/model_0007799.pth" # path to the model we just trained
  cfg.MODEL.WEIGHTS = model_weights_path
  cfg.MODEL.ROI_HEADS.NUM_CLASSES = 24
  cfg.MODEL.DEVICE='cpu'
  cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.60   # set a custom testing threshold
  predictor = DefaultPredictor(cfg)

  # predict the seg and visualize
  dic_id_mask = {}
  # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  outputs = predictor(img)  # format is documented at https://detectron2.readthedocs.io/tutorials/models.html#model-output-format
  masked = np.zeros((480,640))

  plt.imshow(img, interpolation='none')
  for i in range(len(outputs["instances"].pred_classes)):
    id = int(outputs["instances"].pred_classes[i])
    mask = np.asarray(outputs["instances"].pred_masks[i].cpu())
    dic_id_mask[id] = mask



    masked = np.ma.masked_where(mask == False, mask)
    plt.imshow(masked, 'jet', interpolation='none', alpha=0.7)
  plt.draw()
  plt.pause(0.0005)
  return dic_id_mask

def post_processing(meta_data_csv_path, dic_id_mask):
  #read the mapping id:name
  id_name = {}
  with open(meta_data_csv_path, mode='r') as inp:
      reader = csv.reader(inp)
      id_name = {rows[0]:rows[1] for rows in reader}
  

  dic_name_mask = {}
  # print(id_name)
  for id, mask in dic_id_mask.items():
    name = id_name[str(id)]
    #print(len(mask))
    dic_name_mask[name] = mask


  return dic_name_mask

def callback(data):
  #bridge = CvBridge()
  #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
  model_weights_path = "/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/moretrain/model_0019999.pth"
  meta_data_csv_path = '/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/output.csv'
  object_list = ObjectList()
  dic_id_mask = seg_output(image, model_weights_path)
  dic_name_mask = post_processing(meta_data_csv_path, dic_id_mask)
  for name, mask in dic_id_mask.items():
      print(len(mask))
      result = np.where(mask == True)
      print(result)
      name_str = String()
      name_str.data = name
      object_list.objects.append(name_str)
      indices = Indices()
      for i in range (len(mask)):
        for j in range(len(mask[0])):
            if mask[i][j] == True:
                index = Int32()
                index.data = 640 * i +j
                indices.indices.append(index)
      object_list.object_indices.append(indices)
  pub.publish(object_list)
                
       

def main():
  rospy.init_node('deep_learning', anonymous=True)
  plt.ion()
  plt.show()
  # Subscribe image
  pub = rospy.Publisher("rcnn_objects",ObjectList, queue_size = 1)
  sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,callback, queue_size = 1, buff_size=2**24)


  #meta_data_csv_path = '/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/output.csv'
  #img_path = '/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/test/'
  #img_list = os.listdir(img_path)
  #im = cv2.imread(os.path.join(img_path,img_list[0]))
  #im = cv2.imread("/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/test/left0000.jpg")
  #model_weights_path = "/home/fetch/catkin_ws/src/Team-GIX-main/perception/DeepLearning/moretrain/model_0019999.pth"
  #dic_id_mask = seg_output(im, model_weights_path)
  #dic_name_mask = post_processing(meta_data_csv_path, dic_id_mask)
  

  #rospy.sleep(5.)
  #print(dic_name_mask)
  rospy.spin()

# print(dic_name_mask)
if __name__ == "__main__":
  main()
