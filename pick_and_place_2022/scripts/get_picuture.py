#!/usr/bin/env python3
from pickletools import float8
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib as plt

num = 70
time = 4

# コールバック関数
def color_image(msg):
    try:
        bridge = CvBridge()
        # subscribeしたImage型のmsgをopencvで扱える形式に変換
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # img_crop = img[(480 - 300) // 2:(480 - 300) // 2 + 300 ,
        #                    (640 - 300) // 2:(640 - 300) // 2 + 300]
        cv2.imwrite("/home/ericlab/ピクチャ/hamamoto/test"+str(num)+"/test" + str(num) +"_"+str(time)+ "_color.png", img)
        
    except Exception as err:
        print(err)

def depth_image(msg):
    try:
        bridge = CvBridge()
        # subscribeしたImage型のmsgをopencvで扱える形式に変換
        img = bridge.imgmsg_to_cv2(msg, "32FC1")
        cv_img8 = np.zeros(img.shape)
        cv_img8 = cv2.convertScaleAbs(img-np.min(img), cv_img8, 255/(np.max(img)-np.min(img)), beta=0)
        cv_img8 = cv_img8.astype(np.uint8)
        cv2.imwrite("/home/ericlab/ピクチャ/hamamoto/test"+str(num)+"/test"+str(num)+"_"+str(time)+"_depth.png", cv_img8)
        
    except Exception as err:
        print(err)

def grasp_image(msg):
    try:
        bridge = CvBridge()
        # subscribeしたImage型のmsgをopencvで扱える形式に変換
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("/home/ericlab/ピクチャ/hamamoto/test"+str(num)+"/test"+str(num)+"_"+str(time)+"_grasp.png", img)
        
    except Exception as err:
        print(err)

def start_node():
    # nodeの初期化
    rospy.init_node('get_picture')
    rospy.loginfo('get_picture node started')
    # 画像をシミュレーションからsubscribeする
    # rospy.wait_for_message("/camera/color/image_raw", Image,timeout=None)
    rospy.Subscriber("/camera/color/image_raw", Image, color_image)
    # rospy.wait_for_message("/ggcnn/img/depth", Image,timeout=None)
    rospy.Subscriber("/ggcnn/img/depth", Image, depth_image)
    # rospy.wait_for_message("/ggcnn/img/grasp_plain", Image,timeout=None)
    rospy.Subscriber("/ggcnn/img/grasp_plain", Image, grasp_image)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass