#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import rospy
from robotsystem.msg import webcammsg
import numpy as np


class webcam:
    
    def __init__(self):
        
        self.cap = cv2.VideoCapture(0) # 노트북 사용시 9은 노트북 웹캠 /dev/video 넘버 확인 필요
        self.MsgMap = webcammsg()
        self.pub = rospy.Publisher("/Web_Cam", webcammsg, queue_size=1)
    
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,360)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,360)
        self.cap.set(cv2.CAP_PROP_FPS,30)
        
        #self.bridge = CvBridge()

if __name__ == '__main__':
    
    rospy.init_node('Webcam', anonymous=True)
    cam = webcam()
    
    while not rospy.is_shutdown():
        
        ret, img = cam.cap.read()
        
        if ret:
        
            img = cv2.flip(img,1) # 좌우 대칭
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            height, width, layers = img.shape
            print(height, height, layers)
            #print(img)
            
            img = np.array(img).flatten().tolist()
            
            cam.MsgMap.data = img
            
            cam.pub.publish(cam.MsgMap.data)
            
        else:
            
            print("WebCam Connect Check!!!!!")
            break
        
        
        
    
    cam.cap.release()
    cv2.destroyAllWindows()
