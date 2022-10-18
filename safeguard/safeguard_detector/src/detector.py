from copy import deepcopy
from threading import local
from charset_normalizer import detect
import cv2
import mediapipe as mp
import time
import math

from prompt_toolkit import print_formatted_text
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import numpy as np
import os

CONF_THRES = 0.5
MARGIN = 30
PADDING_RATIO = 0.1

class Person:
    def __init__(self, img, landmarks) -> None:
        self.img = img
        self.landmarks = []
        self.escaping_status = False
        if landmarks.pose_landmarks:
            for index, lm in enumerate(landmarks.pose_landmarks.landmark):
                h, w, c = img.shape
                # 得到的关键点坐标x/y/z/visibility都是比例坐标，在[0,1]之间
                # 转换为像素坐标(cx,cy)，图像的实际长宽乘以比例，像素坐标一定是整数
                cx, cy = int(lm.x * w), int(lm.y * h)
                # 保存坐标信息
                self.landmarks.append((cx, cy))
    
    def is_escaping(self):
        pass

class Detector:
    def __init__(self,debug_mode=False) -> None:
        yolov5_path = os.path.join(rospkg.RosPack().get_path('safeguard_detector'), "yolov5")
        print(yolov5_path)
        self.yolo_model = torch.hub.load(yolov5_path, 'yolov5n',device="cpu",source="local",force_reload=False)
        self.yolo_model.classes = [0]
        self.debug_mode = debug_mode
        self.mpPose = mp.solutions.pose  # 姿态识别方法
        self.pose = self.mpPose.Pose(static_image_mode=False, # 静态图模式，False代表置信度高时继续跟踪，True代表实时跟踪检测新的结果
                        #    upper_body_only=False,  # 是否只检测上半身
                        smooth_landmarks=True,  # 平滑，一般为True
                        min_detection_confidence=0.5, # 检测置信度
                        min_tracking_confidence=0.5)  # 跟踪置信度
        
    def detect_object(self,img):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return self.yolo_model(imgRGB)
    
    def detect_landmarks(self,img):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return self.pose.process(imgRGB)
    
    def detect(self, img):
        persons = []
        img2show = deepcopy(img)
        pred = self.detect_object(img).xyxy
        if len(pred) == 0:
            return []
        else:
            for (xmin, ymin, xmax, ymax, conf, cls) in pred[0].tolist():
                if conf < CONF_THRES:
                    continue
                # Generate Person Object
                bbox = np.array([xmin, ymin, xmax, ymax])
                #padding the roi to make sure the person is totally inside the roi img.
                pad_x = (xmax - xmin) * PADDING_RATIO
                pad_y = (ymax - ymin) * PADDING_RATIO
                xmax = xmax + pad_x/2
                xmin = xmin - pad_x/2
                ymax = ymax + pad_y/2
                ymin = ymin - pad_y/2
                person_img = img[int(ymin):int(ymax),int(xmin):int(xmax):]
                if self.debug_mode:
                    cv2.rectangle(img2show,(int(xmin),int(ymin)),(int(xmax),int(ymax)),(0,255,0),2)
                landmarks = self.detect_landmarks(person_img)
                person = Person(person_img,landmarks)
                person.is_escaping()
                persons.append(person)
        if self.debug_mode:
            cv2.imshow("DEBUG_IMG",img2show)
            cv2.waitKey(1)
        return persons
    
def cal_angle(point_a, point_b, point_c):
    a_x, b_x, c_x = point_a[0], point_b[0], point_c[0]  
    a_y, b_y, c_y = point_a[1], point_b[1], point_c[1]  

    if len(point_a) == len(point_b) == len(point_c) == 3:
        # print("坐标点为3维坐标形式")
        a_z, b_z, c_z = point_a[2], point_b[2], point_c[2] 
    else:
        a_z, b_z, c_z = 0,0,0  
        # print("坐标点为2维坐标形式，z 坐标默认值设为0")

    # 向量 m=(x1,y1,z1), n=(x2,y2,z2)
    x1,y1,z1 = (a_x-b_x),(a_y-b_y),(a_z-b_z)
    x2,y2,z2 = (c_x-b_x),(c_y-b_y),(c_z-b_z)

    # 两个向量的夹角，即角点b的夹角余弦值
    cos_b = (x1*x2 + y1*y2 + z1*z2) / (math.sqrt(x1**2 + y1**2 + z1**2) *(math.sqrt(x2**2 + y2**2 + z2**2))) # 角点b的夹角余弦值
    if round(cos_b,7) == 1. or round(cos_b,7) == -1. :
        B = 180
    else:
        B = math.degrees(math.acos(cos_b)) # 角点b的夹角值
    # print(cos_b)
    # print(B)
    return B


def generate_params_dict(key, params):
    for param in rospy.get_param_names():
        if param.split("/").count(key):
            param_key = param.split("/")[-1]
            params[param_key] = rospy.get_param(param)
    return params

def callback(img_raw, args):
    img = args["cv_bridge"].imgmsg_to_cv2(img_raw, "bgr8")
    detector = args["detector"]
    persons = []
    persons = detector.detect(img)
    is_someone_escaping = False
    #Check if persons is empty (tricky usage XD)
    if persons:
        for person in persons:
            if person.escaping_status:
                is_someone_escaping = True
        if is_someone_escaping:
            str = String()
            str.data = "fence_warning"
            args["pub"].publish(str)

def main():
    params = {}
    callback_args = {}
    params = generate_params_dict('safeguard_detector', params)
    rospy.init_node('safeguard_detector', anonymous=True)
    pub = rospy.Publisher(params["broadcast_topic"], String,queue_size=1)
    cv_bridge = CvBridge()
    detector = Detector(params["debug_mode"])
    callback_args["pub"] = pub
    callback_args["cv_bridge"] = cv_bridge
    callback_args["detector"] = detector
    rospy.Subscriber(params["img_topic"],Image,callback,callback_args,queue_size=1)
    rospy.loginfo("safeguard_detector is now ready...")
    rospy.spin()

if __name__ == "__main__":
    main()