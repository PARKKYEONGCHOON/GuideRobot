#!/usr/bin/python3
# -*- coding: utf-8 -*-

#import cvlib as CV
#from cvlib.object_detection import draw_bbox

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from numpy import size
from cls_File import *
from cls_GlobalVarialble import *

import cv2
import mainwindow as main



class Cam:
    
    def __init__(self):
        
        self.camNum = int(GlobalVariable.CameraNum);
        #self.cap = self.cam_cap()
        self.videoStart = False
        #self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.fcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        
        self.detectMode = False
        self.frame = None
        self.detectObject = []

        
    def cam_cap(self):
        
        tmpcap = cv2.VideoCapture(self.camNum)
        return tmpcap
    
        
    def cam_isOpen(self):
        
        if self.cap.isOpened():
            print("Cam Connect")
            return True
        else:
            print("Cam Connect Fail")
            return True
        
    def cam_capture(self):
        
        if self.cam_isOpen:
            ret, frame = self.cap.read()
            frame = cv2.flip(frame,1) # 좌우 대칭
            tmpframe = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = None
            
            if ret:
                
                if self.detectMode:
                    
                    #bbox, self.detectObject, conf = CV.detect_common_objects(tmpframe)
                    #print(bbox, label, conf)
                    #drawframe = draw_bbox(tmpframe,bbox,self.detectObject,conf,write_conf=True)
                    pass
                
            
                else:
                    
                   
                    return tmpframe

                
        
    def cam_video_Save(self):
        
        i = 1
        date = File.get_Today()
        dir1 = GlobalVariable.videoPath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.videoPath + "/" + date
        File.make_foloder(dir2)
        
        while True:
            
            Videopath = dir2 + "/" + str(i)+ ".avi"
            
            if not os.path.exists(Videopath):
                out = cv2.VideoWriter(Videopath, self.fcc, self.fps, (self.width, self.height),isColor=True) # 3 width, 4 height
                break
            else:
                i += 1
        
        while self.videoStart:
            
            if self.cam_isOpen:
                
                ret, frame = self.cap.read()
                frame = cv2.flip(frame,1) # 좌우 대칭
                
                if ret:
                    out.write(frame)

                
            else:
                print("Cam is Not Open")
                
    def cam_video_Save(self,imageArr):
        
       
            i = 1
            date = File.get_Today()
            dir1 = GlobalVariable.videoPath
            File.make_foloder(dir1)
            dir2 = GlobalVariable.videoPath + "/" + date
            File.make_foloder(dir2)
            
            #size = (imageArr[0].width,imageArr[0].height)
            
            
            while True:
                
                Videopath = dir2 + "/" + str(i)+ ".avi"
                
                if not os.path.exists(Videopath):
                    # img_arry = []
                    # for i in range(len(imageArr)):
                    #     #image = cv2.cvtColor(imageArr[i],cv2.COLOR_BGR2RGB)
                    #     #img = cv2.imread(image)
                    #     height, width, layers = imageArr[0].shape
                    #     size = (width,height)
                    #     img_arry.append(imageArr)
                
                    # #out = cv2.VideoWriter(Videopath, self.fcc, 30, (1280, 720),isColor=True)
                    # out = cv2.VideoWriter(Videopath, cv2.VideoWriter_fourcc(*'DIVX'),15,size)
                    # for i in range(len(img_arry)):
                    #     out.write(img_arry[i])
                    # #out.release()    
                    fps = 30
                    frame_array = []
                        
                    out = cv2.VideoWriter(Videopath,cv2.VideoWriter_fourcc(*'DIVX'), fps, (640,480))
                    for i in range(len(imageArr)):
                        # writing to a image array
                        out.write(imageArr[i])
                    out.release()
                    break
                else:
                    i += 1
        
            
                
        
        
    def cam_Image_Save(self,image):
        
        i = 1
            
        date = File.get_Today()
        dir1 = GlobalVariable.imagePath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.imagePath + "/" + date
        File.make_foloder(dir2)
        
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        
        while True:
            
            imgpath = dir2 + "/" + str(i)+ ".jpg"
            
            if not os.path.exists(imgpath):
                cv2.imwrite(imgpath,image)
                break
            
            else:
                
                i += 1
                
    def cam_Video_Image_Save(self,image):
        
        i = 1
        j = 1
            
        date = File.get_Today()
        dir1 = GlobalVariable.videoPath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.videoPath + "/" + date
        File.make_foloder(dir2)
        dir3 = GlobalVariable.videoPath + "/" + date + "/" + str(j)
        
        while True:
            if not os.path.isdir(dir3):
                os.mkdir(dir3)
                break
            else:
                j +=1
        
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        
        while True:
            
            imgpath = dir3 + "/" + str(i)+ ".jpg"
            
            if not os.path.exists(imgpath):
                cv2.imwrite(imgpath,image)
                break
            
            else:
                
                i += 1

    def cam_Image_Load(self,Path):
        
        try:
            
            img = cv2.imread(Path, cv2.IMREAD_COLOR)
            frame = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
            #resizeframe = cv2.resize(frame, dsize=(int(GlobalVariable.cameraX),int(GlobalVariable.cameraY)),interpolation=cv2.INTER_AREA)
            return frame
        
        except:
            pass

    def cam_Video_Load(self,path,GrabVideo):
        
        cap = cv2.VideoCapture(path)

        # 프레임 너비/높이, 초당 프레임 수 확인
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH) # 또는 cap.get(3)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) # 또는 cap.get(4)
        fps = cap.get(cv2.CAP_PROP_FPS) # 또는 cap.get(5)
        print('프레임 너비: %d, 프레임 높이: %d, 초당 프레임 수: %d' %(width, height, fps))
    
        while cap.isOpened(): 
            ret, frame = cap.read()
            
            if not ret:
                
                break

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            tmpImage = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            scene = QGraphicsScene()
            pixmap = QPixmap(tmpImage)
            item = QGraphicsPixmapItem(pixmap)
            scene.addItem(item)
            GrabVideo.setScene(scene)
            
            #return frame

        
        

   
    def SLAM_Image_Save(self,image):
        
        i = 1
            
        date = File.get_Today()
        dir1 = GlobalVariable.slamimagepath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.slamimagepath + "/" + date
        File.make_foloder(dir2)
        
        #image = cv2.cvtColor(image)
        
        while True:
            
            imgpath = dir2 + "/" + str(i)+ ".png"
            
            if not os.path.exists(imgpath):
                cv2.imwrite(imgpath,image)
                break
            
            else:
                
                i += 1
        
        