#!/usr/bin/env python3
import csv
# from detectron2.engine import DefaultTrainer
# import some common libraries
import numpy as np
import os, json, cv2, random
import rospy
#import cv2_imshow
import matplotlib.pyplot as plt
from perception_msgs.msg import ObjectList, Indices, Target
from sensor_msgs.msg import Image
#from cv_bridge.boost.cv_bridge_boost import getCvType
from std_msgs.msg import String, Int32
# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
import rospkg

#note that img is using cv2.read to read the img path

pub = None
target = None
rospack = rospkg.RosPack()
path = rospack.get_path('perception')
model_weights_path = path + "/DeepLearning/moretrain/model_0019999.pth"
meta_data_csv_path = path + '/DeepLearning/output.csv'

def seg_output(img, model_weights_path):
  global target
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
  dic_name_mask = {}
  # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  outputs = predictor(img)  # format is documented at https://detectron2.readthedocs.io/tutorials/models.html#model-output-format
  masked = np.zeros((480,640))
  plt.imshow(img, interpolation='none')
  instances = outputs["instances"]
  confident_detections = instances[instances.scores > 0.8]

  for i in range(len(outputs["instances"].pred_classes)):
    id = int(outputs["instances"].pred_classes[i])
    mask = np.asarray(outputs["instances"].pred_masks[i].cpu())
    name = find_class(id, meta_data_csv_path)
    indices = []
    result = np.where(mask == True)
    indices = 640 * result[0] + result[1]

    if name not in dic_name_mask:
        dic_name_mask[name] = indices
    else:
        dic_name_mask[name] = np.append(dic_name_mask[name], indices)

    # if name == 'green_covers' and outputs["instances"].scores[i] > 0.8:
#    print(target)
    if name == target:
      masked = np.ma.masked_where(mask == False, mask)
      plt.imshow(masked, 'jet', interpolation='none', alpha=0.7)
  plt.draw()
  plt.pause(0.0005)
  return dic_name_mask

def find_class(id, path):
  with open(path, mode='r') as inp:
      reader = csv.reader(inp)
      id_name = {rows[0]:rows[1] for rows in reader}
  return id_name[str(id)]

def send_msg(dic_name_mask):
  object_list = ObjectList()
  for name, arr in dic_name_mask.items():
      indices = Indices()
      for i in range (len(arr)):
         indices.indices.append(arr[i])
      object_list.objects.append(name)
      object_list.object_indices.append(indices)
  if pub:
      pub.publish(object_list)


def callback(data):
  #bridge = CvBridge()
  #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
  dic_name_mask = seg_output(image, model_weights_path)
  send_msg(dic_name_mask)
                
def target_callback(data):
  global target
  target = data.name

def main():
  rospy.init_node('deep_learning', anonymous=True)
  plt.ion()
  plt.show()
  # Subscribe image
  global pub
  pub = rospy.Publisher("rcnn_objects",ObjectList, queue_size = 1, latch = True)
  sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,callback, queue_size = 1, buff_size=2**24)
  target_sub = rospy.Subscriber('final_pick_and_place/target', Target, target_callback)

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
