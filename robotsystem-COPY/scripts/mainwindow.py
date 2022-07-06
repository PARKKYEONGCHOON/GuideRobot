#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os,sys
import json

import rospy
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import roslib
import cv2
import cvlib as cv
import pyzed.sl as sl
import imageio
import message_filters

from PIL import Image as imim
from matplotlib.backends.backend_qt5agg import FigureCanvas as FigureCanvas
from matplotlib.figure import Figure
from time import *

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore, QtGui
from PyQt5 import uic
from python_qt_binding import loadUi

from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData, Path
from nav_msgs.srv import GetMap, GetMapRequest
from geometry_msgs.msg import Twist
from scout_msgs.msg import ScoutLightCmd
from robotsystem.msg import my_msg, depthmsg, automodemsg, map, webcammsg
from robotsystem.srv import *
from std_msgs.msg import Bool

from cls_GlobalVarialble import *
from cls_Cam import *
from cls_Slam import *
from cls_car import *
from cls_Log import *
from cls_ImageProcessing import *
from cls_sqllite import *
from mainSubDestinationwindow import *
#from mainLoginWIndow import *




matplotlib.use('Qt5Agg')

diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)

form_class = uic.loadUiType("widget.ui")[0]



class robotsystem(QWidget,form_class):

    def __init__(self):
        #super(robotsystem, self).__init__()
        super().__init__()
        
        self.tmpSaveImage = None
        self.tmpSaveImage2 = None
        self.tmpConImage = None
        self.tmpSaveVideo_ImageArray = []
        self.tmpSaveVideo_ImageArray2 = []
        self.tmpSaveVideo_ImageArray3 = []
        self.tmpSlamSaveImage = None
        self.OperationMode = False
        self.VideoMode = False 
        self.VideoSave = False
        self.SlamSave = False
        self.QRCODEUSE = False
        self.DetectUSE = False
        self.depthUSE = False
        self.LoadVideo = ""
        self.tmpDepth = ""
        self.dictOdom = None
        self.dictRange = None
        self.slamMap = None
        self.slamMaps = None
        self.tmpstampdic = {}
        self.tmpslamMapdic = {}
        
        self.MonitorImage = None
        self.LidarImage = None
        

        self.occupancy_map = None
        self.SlamStart = False

        
        # #스카웃 메세지, 퍼블리셔
        # self.Tmsg = Twist()
        # self.ScoutLightmsg = ScoutLightCmd()
        
        # self.Movepub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        # self.ScoutLightpub = rospy.Publisher("/scout_light_control",ScoutLightCmd,queue_size=10)
        
        #client
        
        
        #제드 관련 메세지 서브스크라이버
        self.imagearray = None
        self.Imagemsg = Image()
        self.Pathmsg = Path()
        
        self.image_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image,self.ZED_Image_Callback)
        self.image_sub2 = rospy.Subscriber("/zed/zed_node/right/image_rect_color",Image,self.ZED_Image_Callback2)
        self.Confidence_image_sub = rospy.Subscriber("/zed/zed_node/confidence/confidence_map",Image,self.ZED_Confidence_Image_Callback)
        self.path_map_sub = rospy.Subscriber("/zed/zed_node/path_map",Path,self.ZED_pathMap_Callback)
        self.path_odom_sub = rospy.Subscriber("/zed/zed_node/path_odom",Path,self.ZED_pathOdom_Callback)
        #self.odom_sub = rospy.Subscriber("/zed/zed_node/odom",Odometry,self.ZED_Odom_Callback)
        
        #self.zed_rate = rospy.Rate(100)
        
        #t265
        
        # self.odom_sub = rospy.Subscriber("/camera/odom/sample",Odometry,self.Odom_Callback)
        
        #셀프 메세지 관련
        self.Selfmsg = Bool()
        self.SelfDeth = depthmsg()
        self.SelfMode = automodemsg()
        self.selpub = rospy.Publisher("/self_msg",Bool,queue_size=1)
        self.selModepub = rospy.Publisher("/mode",automodemsg,queue_size=1)
        self.depth_sub = rospy.Subscriber("/depth",depthmsg,self.ZED_Depth_Callback)
        self.OFastSlam_sub = rospy.Subscriber("/Self_map",map,self.OFastSlam_Callback)
        self.Monitor_sub = rospy.Subscriber("/Web_Cam",webcammsg,self.Monitor_Callback)
        #self.Monitor_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.Monitor_Callback)
        
        self.DestiClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.Destiopersrv = robotservice()
        self.Destireqsrv = robotserviceRequest()
        
        #SLAM
        # self.scanmsg = LaserScan()
        # self.scan_sub = rospy.Subscriber("/scan", LaserScan,self.Lidar_Scan_Callback)
        self.Mapmsg = OccupancyGrid()
        self.MapMetamsg = MapMetaData()
        self.MapMetaSub = rospy.Subscriber("/map_metadata",MapMetaData,self.map_metadata_Callback)
        self.MapOccupancySub = rospy.Subscriber('/map',OccupancyGrid,callback=self.map_occupancy_Callback)
        
        
        
        #동영상 쓰레드
        self.ThreadVideo = VideoWorker(self) #재생
        self.ThreadVideo.changePixmap.connect(self.setImage)
        
        self.ThreadVideo2 = VideoWorker2(self) #재생
        self.ThreadVideo2.changePixmap.connect(self.setImage2)
        
        self.DepthThreadVideo = DepthVideoWorker(self) #재생
        self.DepthThreadVideo.changePixmap.connect(self.setImage3)
        
        self.ThreadVideoSave = VideoSaveWorker(self) #저장
        #self.ThreadVideoSave.imagearray.connect(self.Video_Save)
        self.ThreadVideoLoad = VideoLoadWorker(self) #로드
        self.ThreadVideoLoad.updateImage.connect(self.Image_LoadVideoStream)
        
        self.ThreadMonitorVideo = MonitorWorker(self)
        self.ThreadMonitorVideo.changeMonitor.connect(self.setImage4)
        
        #DEPTH THREAD
        self.ThreadDepth = DEPTHMWorker(self)
        self.ThreadDepth.updateText.connect(self.set_DEPTH_text)
        
        #SLAM
        
        self.ThreadSlam = SLAMWorker(self)
        self.ThreadSlam.updateMap.connect(self.setSlamImage)
        
        
        
        self.UI_init() #UI 초기화
        self.Setting_Load() #셋팅 로딩
        #self.showFullScreen()
        self.show() # 메인폼 출력
        
        if self.depthUSE:
            self.ThreadDepth.DEPTHThread = True
            self.ThreadDepth.start()
            
    def UI_init(self):
        
        # # QT Creator UI File Load
        dir = os.path.dirname(os.path.realpath(__file__))
        uidir = os.path.join(dir, 'widget.ui')
        loadUi(uidir, self)

        
        # 메인폼 사이즈 조정,고정,센터,제목
        self.resize(int(GlobalVariable.FormSizeX),int(GlobalVariable.FormSizeY))
        self.setFixedSize(int(GlobalVariable.FormSizeX),int(GlobalVariable.FormSizeY))
        self.Center()
        self.setWindowTitle("Robot System ")


        #메인 레이아웃 지정
        
        self.MainLayout = QVBoxLayout()
        self.setLayout(self.MainLayout)
        
        
        self.MainHigh = QHBoxLayout()
        self.MainHighLayout1 = QHBoxLayout()
        self.MainHighLayout2 = QHBoxLayout()
        self.MainHighLayout3 = QHBoxLayout()
        
        self.MainHigh2 = QHBoxLayout()
        self.MainHigh2Layout1 = QHBoxLayout()
        self.MainHigh2Layout2 = QHBoxLayout()

        self.MainLow = QHBoxLayout()
        self.MainLowLayout1 = QHBoxLayout()
        self.MainLowLayout2 = QHBoxLayout()

        self.MainLayout.addLayout(self.MainHigh,stretch= 2)
        self.MainLayout.addLayout(self.MainHigh2,stretch= 2)
        self.MainLayout.addLayout(self.MainLow,stretch= 1)
        
        self.MainHigh.addLayout(self.MainHighLayout1,stretch= 1)
        self.MainHigh.addLayout(self.MainHighLayout3,stretch= 1)
        self.MainHigh.addLayout(self.MainHighLayout2,stretch= 1)
        
        
        self.MainHigh2.addLayout(self.MainHigh2Layout1,stretch= 1)
        self.MainHigh2.addLayout(self.MainHigh2Layout2,stretch= 1)


        self.MainLow.addLayout(self.MainLowLayout1)
        self.MainLow.addLayout(self.MainLowLayout2)

        ##로그창 부분
        self.MainTestLay = QHBoxLayout()
        self.MainTestLay1 = QHBoxLayout()
        self.GrbLogLay = QHBoxLayout()

        #self.MainLayout.addLayout(self.MainTestLay,stretch= 1)
        self.MainLow.addLayout(self.MainTestLay)


        self.MainTestLay.addLayout(self.MainTestLay1)
        self.GrbLog = QGroupBox(self)
        self.GrbLog.setTitle("LOG")
        self.txt_Log = QTextEdit()
        self.txt_Log.setEnabled(False)

        self.GrbLog.setLayout(self.GrbLogLay)
        self.MainTestLay1.addWidget(self.GrbLog)
        self.GrbLogLay.addWidget(self.txt_Log)
        
        
        
        


        #비디오Left 화면 부분
        self.GrbVideo = QGroupBox(self)
        self.GrbVideo.setTitle("LEFT CAMERA VIDEO(QR CODE)")
        self.MainHighLayout1.addWidget(self.GrbVideo)
        self.GrbVideoLayout = QHBoxLayout()
        self.GrbVideo.setLayout(self.GrbVideoLayout)

        self.lblVideo = QLabel()
        self.GrbVideoLayout.addWidget(self.lblVideo)
        self.lblVideo.setStyleSheet("background-Color : white")
        #self.GraVideo = QGraphicsView()
        #self.GrbVideoLayout.addWidget(self.GraVideo)
        self.VideoScene = QGraphicsScene()
        
        #뎁스 화면
        self.GrbVideo3 = QGroupBox(self)
        self.GrbVideo3.setTitle("DEPTH VIDEO")
        self.MainHighLayout1.addWidget(self.GrbVideo3)
        self.GrbVideo3Layout = QHBoxLayout()
        self.GrbVideo3.setLayout(self.GrbVideo3Layout)
        
        self.lblVideo3 = QLabel()
        self.GrbVideo3Layout.addWidget(self.lblVideo3)
        self.lblVideo3.setStyleSheet("background-Color : white")
        
        #비디오 Right
        self.GrbVideo2 = QGroupBox(self)
        self.GrbVideo2.setTitle("RIGHT CAMERA VIDEO(Recognation)")
        self.MainHighLayout1.addWidget(self.GrbVideo2)
        self.GrbVideo2Layout = QHBoxLayout()
        self.GrbVideo2.setLayout(self.GrbVideo2Layout)
        
        self.lblVideo2 = QLabel()
        self.GrbVideo2Layout.addWidget(self.lblVideo2)
        self.lblVideo2.setStyleSheet("background-Color : white")
        
        
        
        #모니터링 화면
        self.GrbMonitorVideo = QGroupBox(self)
        self.GrbMonitorVideo.setTitle("Monitor VIDEO")
        self.MainHigh2Layout1.addWidget(self.GrbMonitorVideo)
        self.GrbMonitorVideoLayout = QHBoxLayout()
        self.GrbMonitorVideo.setLayout(self.GrbMonitorVideoLayout)
        
        self.lbl_MonitorVideo = QLabel()
        self.GrbMonitorVideoLayout.addWidget(self.lbl_MonitorVideo)
        self.lbl_MonitorVideo.setStyleSheet("background-Color : white")
        
        
        #슬램 화면 부분
        self.GrbSLAM = QGroupBox("SLAM")
        self.MainHigh2Layout2.addWidget(self.GrbSLAM)
        self.GrbSlamLayout = QHBoxLayout()
        self.GrbSLAM.setLayout(self.GrbSlamLayout)


        self.lblSlam = QLabel()
        self.lblSlam.setStyleSheet("background-Color : white")
        #self.GraSlam = QGraphicsView()
        self.GrbSlamLayout.addWidget(self.lblSlam)
        #self.GrbSlamLayout.addWidget(self.GraSlam)
        self.SlamScene = QGraphicsScene()

        

        #오퍼레이션 부분
        self.GrbOperation = QGroupBox("OPERATION")
        self.MainLowLayout1.addWidget(self.GrbOperation)
        self.GrbOperationLayOut = QGridLayout()
        self.GrbOperation.setLayout(self.GrbOperationLayOut)

        self.btn_start = QPushButton("START")
        self.btn_start.clicked.connect(self.btn_start_clicked)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.btn_stop_clicked)
        self.btn_Manual = QPushButton("Manual Mode")
        self.btn_Manual.clicked.connect(self.btn_Manual_clicked)
        self.btn_Manual.setStyleSheet("background-Color : white")

        self.btnMoveGroup = QButtonGroup()
        self.btnMoveGroup.setExclusive(False)
        self.btnMoveGroup.buttonPressed[int].connect(self.btn_Move_Pressed)
        self.btnMoveGroup.buttonReleased[int].connect(self.btn_Move_Realsed)
        
        self.btn_straight = QPushButton("▲")
        self.btn_left = QPushButton("◀")
        self.btn_break = QPushButton("●")
        self.btn_right = QPushButton("▶")
        self.btn_back = QPushButton("▼")

        self.btnMoveGroup.addButton(self.btn_straight,1)
        self.btnMoveGroup.addButton(self.btn_left,2)
        self.btnMoveGroup.addButton(self.btn_break,3)
        self.btnMoveGroup.addButton(self.btn_right,4)
        self.btnMoveGroup.addButton(self.btn_back,5)


        self.btnLightGroup = QButtonGroup()
        self.btnLightGroup.setExclusive(False)
        self.btnLightGroup.buttonClicked[int].connect(self.btn_Light_Clicked)

        self.btn_light_on = QPushButton("LIGHT NO")
        self.btn_Light_off = QPushButton("LIGHT NC")
        self.btn_Light_flash = QPushButton("LIGHT BL")

        self.btnLightGroup.addButton(self.btn_light_on,1)
        self.btnLightGroup.addButton(self.btn_Light_off,2)
        self.btnLightGroup.addButton(self.btn_Light_flash,3)
        
        self.GrbOperationLayOut.addWidget(self.btn_start,0,0)
        self.GrbOperationLayOut.addWidget(self.btn_stop,0,1)
        self.GrbOperationLayOut.addWidget(self.btn_Manual,0,2)
        

        self.GrbOperationLayOut.addWidget(self.btn_straight,1,1)
        self.GrbOperationLayOut.addWidget(self.btn_left,2,0)
        self.GrbOperationLayOut.addWidget(self.btn_break,2,1)
        self.GrbOperationLayOut.addWidget(self.btn_right,2,2)
        self.GrbOperationLayOut.addWidget(self.btn_back,3,1)

        self.GrbOperationLayOut.addWidget(self.btn_light_on,4,0)
        self.GrbOperationLayOut.addWidget(self.btn_Light_off,4,1)
        self.GrbOperationLayOut.addWidget(self.btn_Light_flash,4,2)
        
        
        #셋팅부분
        self.btn_SettingSave = QPushButton("Setting Save")
        self.chk_QRcode = QCheckBox("QR Code")
        self.chk_Detect = QCheckBox("Detect")
        self.chk_depth = QCheckBox("Depth")
        
        self.GrbSetting = QGroupBox("SETTING")
        self.MainLowLayout2.addWidget(self.GrbSetting)
        self.GrbSettingLayOut = QGridLayout()
        self.GrbSetting.setLayout(self.GrbSettingLayOut)
        self.lbl_depth = QLabel("DEPTH(m) : ")
        self.lbl_depth.setAlignment(QtCore.Qt.AlignRight)
        self.txt_depth = QLineEdit()
        self.txt_depth.setEnabled(False)
        self.txt_depth.setStyleSheet("background-Color : white; color: black")
        
        
        
        self.GrbSettingLayOut.addWidget(self.btn_SettingSave,0,0)
        self.GrbSettingLayOut.addWidget(self.chk_QRcode,0,1)
        self.GrbSettingLayOut.addWidget(self.chk_Detect,0,2)
        self.GrbSettingLayOut.addWidget(self.chk_depth,0,3)
        self.GrbSettingLayOut.addWidget(self.lbl_depth,0,4)
        self.GrbSettingLayOut.addWidget(self.txt_depth,0,5)

        self.chk_VideoSave = QCheckBox("VIDEO SAVE")
        self.chk_VideoSave.clicked.connect(self.chk_VideoSave_clicked)

        self.btn_imageCapture = QPushButton(self)
        self.btn_imageCapture.setText("Image Grab")
        self.btn_imageCapture.clicked.connect(self.Image_Grab)

        self.btn_imageSave = QPushButton(self)
        self.btn_imageSave.setText("Image Save")
        self.btn_imageSave.clicked.connect(self.Image_Save)

        self.btn_imageLoad = QPushButton(self)
        self.btn_imageLoad.setText("Image Load")
        self.btn_imageLoad.clicked.connect(self.Image_Load)

        self.btn_VideoStart = QPushButton(self)
        self.btn_VideoStart.setText("Video Start")
        self.btn_VideoStart.clicked.connect(self.Image_Video)

        self.btn_VideoLoad = QPushButton(self)
        self.btn_VideoLoad.setText("Video Laod")
        self.btn_VideoLoad.clicked.connect(self.Image_VideoLoad)

        self.GrbSettingLayOut.addWidget(self.chk_VideoSave,1,0)
        self.GrbSettingLayOut.addWidget(self.btn_imageCapture,1,1)
        self.GrbSettingLayOut.addWidget(self.btn_imageSave,1,2)
        self.GrbSettingLayOut.addWidget(self.btn_imageLoad,1,3)
        self.GrbSettingLayOut.addWidget(self.btn_VideoStart,1,4)
        self.GrbSettingLayOut.addWidget(self.btn_VideoLoad,1,5)

        self.chk_SlamSave = QCheckBox("SLAM SAVE")
        self.chk_SlamSave.clicked.connect(self.chk_SlamSave_clicked)

        self.btn_SLAMGrab = QPushButton(self)
        self.btn_SLAMGrab.setText("SLAM Grab")
        self.btn_SLAMGrab.clicked.connect(self.btn_SLAMGrab_clicked)

        self.btn_SLAMVideo = QPushButton(self)
        self.btn_SLAMVideo.setText("SLAM Start")
        self.btn_SLAMVideo.clicked.connect(self.btn_SLAMVideo_clicked)

        self.btn_SLAMSave = QPushButton(self)
        self.btn_SLAMSave.setText("SLAM Save")
        self.btn_SLAMSave.clicked.connect(self.SLAM_Save)

        self.btn_SLAMLoad = QPushButton(self)
        self.btn_SLAMLoad.setText("SLAM Laod")
        self.btn_SLAMLoad.clicked.connect(self.SLAM_Load)
        
        self.btn_NaviWindow = QPushButton(self)
        self.btn_NaviWindow.setText("Destination Set")
        self.btn_NaviWindow.clicked.connect(self.NaviWindow_Load)

        self.GrbSettingLayOut.addWidget(self.chk_SlamSave,2,0)
        self.GrbSettingLayOut.addWidget(self.btn_SLAMGrab,2,1)
        self.GrbSettingLayOut.addWidget(self.btn_SLAMVideo,2,2)
        self.GrbSettingLayOut.addWidget(self.btn_SLAMSave,2,3)
        self.GrbSettingLayOut.addWidget(self.btn_SLAMLoad,2,4)
        self.GrbSettingLayOut.addWidget(self.btn_NaviWindow,2,5)
        

    #메인폼 화면 센터 지정 함수
    def Center(self): 
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    
    #동영상 저장 할지 말지 체크
    def chk_VideoSave_clicked(self):

        if self.chk_VideoSave.isChecked():
            self.chk_VideoSave.setChecked(True)
            self.VideoSave = True
            
        else:
            self.chk_VideoSave.setChecked(False)
            self.VideoSave = False
            
    def chk_QRCODE_clicked(self):
        
        if self.chk_QRcode.isChecked():
            self.chk_QRcode.setChecked(True)
            self.QRCODEUSE = True
            
        else:
            self.chk_QRcode.setChecked(False)
            self.QRCODEUSE = False

    def chk_Detect_clicked(self):
        
        if self.chk_Detect.isChecked():
            self.chk_Detect.setChecked(True)
            self.DetectUSE = True
            
        else:
            self.chk_Detect.setChecked(False)
            self.DetectUSE = False
    
    def chk_Depth_clicked(self):
        
        if self.chk_depth.isChecked():
            self.chk_depth.setChecked(True)
            self.depthUSE = True
            self.ThreadDepth.start()
            self.ThreadDepth.DEPTHThread = True
            
        else:
            self.chk_depth.setChecked(False)
            self.depthUSE = False
            self.ThreadDepth.stop()
            self.ThreadDepth.DEPTHThread = False
    
    #셋팅 저장
    def Setting_Save(self):
        
        ini = Inifile()
        path = 'Setting.ini'
        ini.InIfile_Set_Path(path)
        
        if ini.InIfile_Exist(path):

            tmpval = int(self.chk_VideoSave.isChecked())
            if tmpval == None or tmpval == " ":
                ini.InIfile_WriteValue('Setting','videosave','0')
            else:
                ini.InIfile_WriteValue('Setting','videosave',str(tmpval))

            tmpval = int(self.chk_SlamSave.isChecked())
            if tmpval == None or tmpval == " ":
                ini.InIfile_WriteValue('Setting','slamsave','0')
            else:
                ini.InIfile_WriteValue('Setting','slamsave',str(tmpval))
                
            tmpval = int(self.chk_QRcode.isChecked())
            if tmpval == None or tmpval == " ":
                ini.InIfile_WriteValue('Setting','QRCODEUSE','0')
            else:
                ini.InIfile_WriteValue('Setting','QRCODEUSE',str(tmpval))
                
            tmpval = int(self.chk_Detect.isChecked())
            if tmpval == None or tmpval == " ":
                ini.InIfile_WriteValue('Setting','detectuse','0')
            else:
                ini.InIfile_WriteValue('Setting','detectuse',str(tmpval))
                
            tmpval = int(self.chk_depth.isChecked())
            if tmpval == None or tmpval == " ":
                ini.InIfile_WriteValue('Setting','depth','0')
            else:
                ini.InIfile_WriteValue('Setting','depth',str(tmpval))

            self.Write_LOG("Setting Save")
        

    #셋팅 로드
    def Setting_Load(self):

        iniFile = Inifile()
        iniFile.InIfile_Set_Path('Setting.ini')
        
        tmpval = Inifile.InIfile_ReadValue(iniFile,'Setting','videosave')
        if tmpval == "1":
            self.chk_VideoSave.setChecked(True)
        else:
            self.chk_VideoSave.setChecked(False)
            
        self.VideoSave = bool(int(tmpval))

        tmpval = Inifile.InIfile_ReadValue(iniFile,'Setting','slamsave')
        if tmpval == "1":
            self.chk_SlamSave.setChecked(True)
        else:
            self.chk_SlamSave.setChecked(False)
            
        self.SlamSave = bool(int(tmpval))
        
        tmpval = Inifile.InIfile_ReadValue(iniFile,'Setting','qrcodeuse')
        if tmpval == "1":
            self.chk_QRcode.setChecked(True)
        else:
            self.chk_QRcode.setChecked(False)
            
        self.QRCODEUSE = bool(int(tmpval))
        
        tmpval = Inifile.InIfile_ReadValue(iniFile,'Setting','detectuse')
        if tmpval == "1":
            self.chk_Detect.setChecked(True)
        else:
            self.chk_Detect.setChecked(False)
            
        self.DetectUSE = bool(int(tmpval))
        
        tmpval = Inifile.InIfile_ReadValue(iniFile,'Setting','depth')
        if tmpval == "1":
            self.chk_depth.setChecked(True)
        else:
            self.chk_depth.setChecked(False)
            
        self.depthUSE = bool(int(tmpval))

    
    #현재 비디오화면 이미지 저장
    def Image_Save(self):
        
        # camera.cam_Image_Save(self.tmpSaveImage)
        camera.cam_Image_Save(self.MonitorImage)
        
        self.Write_LOG("IMAGE SAVE")
    
    #이미지 비디오 화면에 로드
    def Image_Load(self):
        FileOpen = QFileDialog.getOpenFileName(self, 'Open file', './')
        FileName = FileOpen[0]
        tmpImage = camera.cam_Image_Load(FileName)
        tmpImage = cv2.resize(tmpImage,dsize=(self.lblVideo.width(),self.lblVideo.height()),interpolation=cv2.INTER_AREA)
        self.tmpSaveImage = tmpImage
        im = QImage(tmpImage.data, tmpImage.shape[1], tmpImage.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap(im)
        self.lblVideo.setPixmap(pixmap)
        self.Write_LOG("IMAGE Load")
    
    #이미지 촬영
    def Image_Grab(self):
        
        if self.QRCODEUSE == 1 and self.DetectUSE == 1:
            
            QRImage, QRres = ImagePro.read_frame(self.tmpSaveImage)
            
            if QRres != None:
                im = QImage(QRImage.data, QRImage.shape[1], QRImage.shape[0], QImage.Format_RGB888)
            else:
                im = QImage(self.tmpSaveImage.data, self.tmpSaveImage.shape[1], self.tmpSaveImage.shape[0], QImage.Format_RGB888)
                QRres = "NO READ"
                
            #detectImage = ImagePro.gender_Detect_frame(self.tmpSaveImage2)
            #detectImage = face.detect_face(self.tmpSaveImage2)
            #detectImage = ImagePro.face_Detection(self.tmpSaveImage2)
            detectImage, site = ImagePro.teacherble_detection(self.tmpSaveImage2)
            
            im2 = QImage(detectImage.data, detectImage.shape[1], detectImage.shape[0], QImage.Format_RGB888)
            
           
            
            pixmap = QPixmap(im)
            pixmap2 = QPixmap(im2)
            self.lblVideo.setPixmap(pixmap)
            self.lblVideo2.setPixmap(pixmap2)

            
        elif self.QRCODEUSE == 1 and self.DetectUSE != 1:
            
            QRImage, QRres = ImagePro.read_frame(self.tmpSaveImage)
            
            if QRres != None:
                im = QImage(QRImage.data, QRImage.shape[1], QRImage.shape[0], QImage.Format_RGB888)
            else:
                im = QImage(self.tmpSaveImage.data, self.tmpSaveImage.shape[1], self.tmpSaveImage.shape[0], QImage.Format_RGB888)
                QRres = "NO READ"
                
            im2 = QImage(self.tmpSaveImage2.data, self.tmpSaveImage2.shape[1], self.tmpSaveImage2.shape[0], QImage.Format_RGB888)
                
            pixmap = QPixmap(im)
            pixmap2 = QPixmap(im2)
            self.lblVideo.setPixmap(pixmap)
            self.lblVideo2.setPixmap(pixmap2)
            self.Write_LOG("IMAGE GRAB")
            self.Write_LOG("QR CODE : " + QRres)

        elif self.QRCODEUSE != 1 and self.DetectUSE == 1:
            
            im = QImage(self.tmpSaveImage.data, self.tmpSaveImage.shape[1], self.tmpSaveImage.shape[0], QImage.Format_RGB888)
            #detectImage = ImagePro.gender_Detect_frame(self.tmpSaveImage2)
            #detectImage = face.detect_face(self.tmpSaveImage2)
            #detectImage = ImagePro.face_Detection(self.tmpSaveImage2)
            detectImage, site = ImagePro.teacherble_detection(self.tmpSaveImage2)
            im2 = QImage(detectImage.data, detectImage.shape[1], detectImage.shape[0], QImage.Format_RGB888)
            
            pixmap = QPixmap(im)
            pixmap2 = QPixmap(im2)
            self.lblVideo.setPixmap(pixmap)
            self.lblVideo2.setPixmap(pixmap2)
            
        else:
            
            im = QImage(self.tmpSaveImage.data, self.tmpSaveImage.shape[1], self.tmpSaveImage.shape[0], QImage.Format_RGB888)
            im2 = QImage(self.tmpSaveImage2.data, self.tmpSaveImage2.shape[1], self.tmpSaveImage2.shape[0], QImage.Format_RGB888)
            
            pixmap = QPixmap(im)
            pixmap2 = QPixmap(im2)
            self.lblVideo.setPixmap(pixmap)
            self.lblVideo2.setPixmap(pixmap2)
            
        
        
        #뎁스이미지 처리
        d1 = cv2.cvtColor(self.tmpSaveImage, cv2.COLOR_BGR2GRAY)
        d2 = cv2.cvtColor(self.tmpSaveImage2, cv2.COLOR_BGR2GRAY)
            
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(d1,d2)

        scaledDisparity = disparity - np.min(disparity)
        if np.max(scaledDisparity) > 0:
            scaledDisparity = scaledDisparity * (255/np.max(scaledDisparity))
        scaledDisparity = scaledDisparity.astype(np.uint8)
        displayHeight, displayWidth = scaledDisparity.shape
        channel = 1
        bytesPerLine = channel * displayWidth
        scaledDisparity = ImagePro.circle_Image(scaledDisparity,int(displayWidth/2)-100,int(displayHeight/2),10,255,255,0,-1)
        qtImg = QImage(scaledDisparity, displayWidth, displayHeight, bytesPerLine, QImage.Format_Indexed8)
            
        pixmap3 = QPixmap(qtImg)
        self.lblVideo3.setPixmap(pixmap3)
        
        self.MonitorImage = cv2.resize(self.MonitorImage,dsize=(self.lbl_MonitorVideo.width()-10,self.lbl_MonitorVideo.height()-10),interpolation=cv2.INTER_AREA)
        moimg = QImage(self.MonitorImage.data, self.MonitorImage.shape[1], self.MonitorImage.shape[0], QImage.Format_RGB888)
        
        pixmap4 = QPixmap(moimg)
        self.lbl_MonitorVideo.setPixmap(pixmap4)
        
        self.Write_LOG("IMAGE GRAB")
            
    #비디오 촬영 쓰레드 On/off
    def Image_Video(self):
        
        if self.VideoMode:
            
            
            self.btn_VideoStart.setText("Video Start")
            self.ThreadVideo.videoThread = False
            self.VideoMode = False
            self.ThreadVideo.stop()
            self.ThreadVideo2.stop()
            self.DepthThreadVideo.stop()
            self.ThreadMonitorVideo.stop()
            self.Write_LOG("VIDEO STOP")
            
            if self.VideoSave:
                self.ThreadVideoSave.start()
                self.ThreadVideoSave.videoSaveThread = True
                
        else:
            
            
            self.btn_VideoStart.setText("Video Stop")
            self.ThreadVideo.videoThread = True
            self.VideoMode = True
            self.ThreadVideo.start()
            self.ThreadVideo2.start()
            self.DepthThreadVideo.start()
            self.ThreadMonitorVideo.start()
            
            self.Write_LOG("VIDEO START")
            
            if self.VideoSave:
                self.ThreadVideoSave.stop()
                self.ThreadVideoSave.videoSaveThread = False
    
    #비디오 쓰레드 화면 영상 출력(표시)           
    @pyqtSlot(np.ndarray)
    def setImage(self, image):
        
        im = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap(im)
        self.lblVideo.setPixmap(pixmap)
    
    @pyqtSlot(np.ndarray)    
    def setImage2(self, image):
        
        im = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap(im)
        self.lblVideo2.setPixmap(pixmap)
        
    @pyqtSlot(QImage)    
    def setImage3(self, image):
        
        pixmap = QPixmap(image)
        self.lblVideo3.setPixmap(pixmap)
        
    @pyqtSlot(QImage)    
    def setImage4(self, image):
        
        pixmap = QPixmap(image)
        self.lbl_MonitorVideo.setPixmap(pixmap)
    
    @pyqtSlot(QImage)
    def setSlamImage(self, image):
        
        pixmap = QPixmap(image)
        self.lblSlam.setPixmap(pixmap)
            
    def convertQImageToMat(self,incomingImage):
        '''  Converts a QImage into an opencv MAT format  '''

        incomingImage = incomingImage.convertToFormat(4)

        width = incomingImage.width()
        height = incomingImage.height()

        ptr = incomingImage.bits()
        ptr.setsize(incomingImage.byteCount())
        arr = np.array(ptr).reshape(height, width, 4)  #  Copies the data
        return arr
    
    @pyqtSlot(str)
    def set_DEPTH_text(self, str):
        
        self.txt_depth.setText(str)

    
    #동영상 로드
    def Image_VideoLoad(self):

        if self.ThreadVideoLoad.videoLoadThread:
            self.ThreadVideoLoad.videoLoadThread = False
            self.ThreadVideoLoad.stop()
            
        else:
            
            self.lblVideo.clear()
            FileOpen = QFileDialog.getOpenFileName(self, 'Open file', './')
            FileName = FileOpen[0]
            self.LoadVideo = FileName
            self.ThreadVideoLoad.videoLoadThread = True
            self.ThreadVideoLoad.start()
            
    #동영상 로드 쓰레드 시작하면 동작
    @pyqtSlot(QImage)
    def Image_LoadVideoStream(self,image):
        
        pixmap = QPixmap(image)
        self.lblVideo.setPixmap(pixmap)
        
    #슬램 그랩
    def btn_SLAMGrab_clicked(self):
        
        img = self.LidarImage
        img = cv2.resize(img,dsize=(self.lblSlam.width()-10,self.lblSlam.height()),interpolation=cv2.INTER_AREA)
        self.tmpSlamSaveImage = img
        im = QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(im)
        self.lblSlam.setPixmap(pixmap)
        self.lblSlam.update()
        self.Write_LOG("SLAM IMAGE Grab")
        
        # rospy.wait_for_service('/static_map') # Wait for the service /static_map to be running
        # get_map_service = rospy.ServiceProxy('/static_map', GetMap) # Create the connection to the service
        # get_map = GetMapRequest() # Create an object of type GetMapRequest
        # result = get_map_service(get_map) # Call the service
        # print (result) # Print the result given by the service called
        # self.slamMap = np.asarray(result, dtype=np.int8).reshape(result.height, result.width)
        # print (self.slamMap)
        
        
        # np_array = np.array(result)
        # impg = cv2.imdecode(np_array,1)
        #print(np_array)
        
        
    
    #슬램 영상 저장 수정 필요
    def chk_SlamSave_clicked(self):
        
        if self.chk_SlamSave.isChecked():
            self.chk_SlamSave.setChecked(True)
            self.SlamSave = True
            
        else:
            self.chk_SlamSave.setChecked(False)
            self.SlamSave = False
    
    #슬램 영상 스타트 수정 필요
    def btn_SLAMVideo_clicked(self):

        if self.ThreadSlam.SLAMThread:
            
            self.btn_SLAMVideo.setText("SLAM START")
            self.ThreadSlam.SLAMThread = False
            self.ThreadSlam.stop()
            
        else:

            self.btn_SLAMVideo.setText("SLAM STOP")
            self.ThreadSlam.SLAMThread = True
            self.ThreadSlam.start()
            
        self.Write_LOG("SLAM START")
        
        

    #슬램 이미지 저장
    def SLAM_Save(self):
            
        camera.SLAM_Image_Save(self.tmpSlamSaveImage)
    
    #슬램 이미지 로드
    def SLAM_Load(self):
        FileOpen = QFileDialog.getOpenFileName(self, 'Open file', './')
        FileName = FileOpen[0]

        tmpImage = camera.cam_Image_Load(FileName)
        
        img = cv2.resize(tmpImage,dsize=(self.lblSlam.width(),self.lblSlam.height()),interpolation=cv2.INTER_AREA)
        h,w,c = img.shape
        qImg = QtGui.QImage(img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qImg)
        self.lblSlam.setPixmap(pixmap)
        self.lblSlam.update()
        
        self.Write_LOG("SLAM IMAGE LOAD")
        
    def NaviWindow_Load(self):
        
        #self.hide()
        Naviwindow = Navi(self)
        #Naviwindow.exec()
        #Naviwindow.show()
        
    #오퍼레이션 자동 시작
    def btn_start_clicked(self):
        
        self.SelfMode.data = 0 #오토모드
        self.Selfmsg = False #qr초기화
        self.selModepub.publish(self.SelfMode)
        self.selpub.publish(self.Selfmsg)
        self.Write_LOG("Auto Mode Start")
        
        self.OperationMode = True
        
        self.btn_start.setStyleSheet("background-Color : yellow")
        self.btn_stop.setStyleSheet("background-Color : white")
        self.btn_Manual.setStyleSheet("background-Color : white")
        self.Write_LOG("Manual Mode OFF")
        
        self.AutorunProcess(0)

    
    #오퍼레이션 자동 정지
    def btn_stop_clicked(self):
        
        self.SelfMode.data = 1 #오토스탑
        self.Selfmsg = False #qr초기화
        self.selModepub.publish(self.SelfMode)
        self.selpub.publish(self.Selfmsg)
        self.Write_LOG("Auto Mode Stop")
        
        self.OperationMode = True
        
        self.btn_start.setStyleSheet("background-Color : white")
        self.btn_stop.setStyleSheet("background-Color : white")
        self.btn_Manual.setStyleSheet("background-Color : yellow")
        self.Write_LOG("Manual Mode ON")
        
        self.AutorunProcess(1)

    #오퍼레이션 수동 조작 온오프
    def btn_Manual_clicked(self):
        
        if self.OperationMode:

            self.OperationMode = False
            self.btn_Manual.setStyleSheet("background-Color : white")
            self.Write_LOG("Manual Mode OFF")

        else:

            self.OperationMode = True
            self.btn_Manual.setStyleSheet("background-Color : yellow")
            self.Write_LOG("Manual Mode ON")

    
    #스카웃 수동 조작 id = 버튼 그룹 버튼별 아이디
    #마우스 조작
    def btn_Move_Pressed(self,id):

        if self.OperationMode:
            
            if id == 1: #straight
                scout.Tmsg.linear.x = 0.2
                scout.Tmsg.angular.z = 0
            elif id == 2: #left
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = 0.5
            elif id == 3: #break
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = 0
            elif id == 4: #right
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = -0.5
            elif id == 5: #back
                scout.Tmsg.linear.x = -0.2
                scout.Tmsg.angular.z = 0

            scout.Movepub.publish(scout.Tmsg)
            # scout.Tmsg.linear.x = 0
            # scout.Tmsg.angular.z = 0
        
        else:
            pass
    
    #마우스 조작
    #키 떼면 멈춤
    def btn_Move_Realsed(self):

        if self.OperationMode:
            scout.Tmsg.linear.x = 0
            scout.Tmsg.angular.z = 0
            scout.Movepub.publish(scout.Tmsg)
            
        else:
            scout.Tmsg.linear.x = 0
            scout.Tmsg.angular.z = 0
            scout.Movepub.publish(scout.Tmsg)
    
    #키보드 조작
    def keyPressEvent(self, e):

        if self.OperationMode:
            if e.key() == Qt.Key_Up: #straight
                scout.Tmsg.linear.x = 0.2
                scout.Tmsg.angular.z = 0
                self.btn_straight.setStyleSheet("background-Color : yellow")
            elif e.key() == Qt.Key_Left:
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = 0.5
                self.btn_left.setStyleSheet("background-Color : yellow")
            elif e.key() == Qt.Key_B: #break
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = 0
                self.btn_break.setStyleSheet("background-Color : yellow")
            elif e.key() == Qt.Key_Right:
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = -0.2
                self.btn_right.setStyleSheet("background-Color : yellow")
            elif e.key() == Qt.Key_Down: #back
                scout.Tmsg.linear.x = -0.8
                scout.Tmsg.angular.z = 0
                self.btn_back.setStyleSheet("background-Color : yellow")

            scout.Movepub.publish(scout.Tmsg)
            # scout.Tmsg.linear.x = 0
            # scout.Tmsg.angular.z = 0

        else:
            pass
    
    #키보드 조작
    def keyReleaseEvent(self, e):
        
        #and not e.isAutoRepeat();
        
        if self.OperationMode:
            
            if (e.key() == Qt.Key_Up or e.key() == Qt.Key_Left or e.key() == Qt.Key_B or e.key() == Qt.Key_Down) and not e.isAutoRepeat():
                scout.Tmsg.linear.x = 0
                scout.Tmsg.angular.z = 0
                scout.Movepub.publish(scout.Tmsg)
                self.btn_straight.setStyleSheet("background-Color : white")
                self.btn_left.setStyleSheet("background-Color : white")
                self.btn_break.setStyleSheet("background-Color : white")
                self.btn_right.setStyleSheet("background-Color : white")
                self.btn_back.setStyleSheet("background-Color : white")
            
        else:
            pass
    
    
    #스카웃 조명 조작
    def btn_Light_Clicked(self,id):

        if id == 1: #조명 온
            scout.ScoutLightmsg.front_mode = 1
            scout.ScoutLightmsg.enable_cmd_light_control = True
            self.btn_light_on.setStyleSheet("background-Color : yellow")
            self.btn_Light_off.setStyleSheet("background-Color : white")
            self.btn_Light_flash.setStyleSheet("background-Color : white")
            self.Write_LOG("LIGHT NO")
            

        elif id == 2: #조명 오프
            scout.ScoutLightmsg.front_mode = 0
            scout.ScoutLightmsg.enable_cmd_light_control = True
            self.btn_light_on.setStyleSheet("background-Color : white")
            self.btn_Light_off.setStyleSheet("background-Color : yellow")
            self.btn_Light_flash.setStyleSheet("background-Color : white")
            self.Write_LOG("LIGHT NC")

        elif id == 3: #점멸등
            scout.ScoutLightmsg.front_mode = 2
            scout.ScoutLightmsg.enable_cmd_light_control = True
            self.btn_light_on.setStyleSheet("background-Color : white")
            self.btn_Light_off.setStyleSheet("background-Color : white")
            self.btn_Light_flash.setStyleSheet("background-Color : yellow")
            self.Write_LOG("LIGHT BL")

        else:
            pass

        scout.ScoutLightpub.publish(self.ScoutLightmsg)
        
    #Autorun 함수
    def AutorunProcess(self,mode):
        
        self.operClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.opersrv = robotservice()
        
        
        if mode == 0: #auto
        
            self.ThreadVideo.videoThread = True
            self.ThreadVideo2.videoThread = True
            self.ThreadSlam.SLAMThread = True
            self.ThreadMonitorVideo.videoThread = True
            
            self.ThreadVideo.start()
            self.ThreadVideo2.start()
            self.DepthThreadVideo.start()
            self.ThreadMonitorVideo.start()
            self.ThreadSlam.start()
            
            try:
            
                reqsrv = robotserviceRequest()
                reqsrv.stamp = rospy.Time.now()
                reqsrv.service_number = 1 #auto
            
            except:
                
                print("Service Conect Check")
            
            if self.VideoSave:
                self.ThreadVideoSave.stop()
                self.ThreadVideoSave.videoSaveThread = False
        
        else: #stop

            self.ThreadVideo.videoThread = False
            self.ThreadVideo2.videoThread = False
            self.ThreadSlam.SLAMThread = False
            self.ThreadMonitorVideo.videoThread = False
            
            self.ThreadVideo.stop()
            self.ThreadVideo2.stop()
            self.DepthThreadVideo.stop()
            self.ThreadMonitorVideo.stop()
            self.ThreadSlam.stop()
            
            try:
            
                reqsrv = robotserviceRequest()
                reqsrv.stamp = rospy.Time.now()
                reqsrv.service_number = 2 #auto stop
                
            except:
                
                print("Service Conect Check")
            
            
            if self.VideoSave:
                self.ThreadVideoSave.start()
                self.ThreadVideoSave.videoSaveThread = True
                
        res = self.operClient(reqsrv)
        print(res)
    
    #로그 저장 함수
    def Write_LOG(self,msg):
        
        date = File.get_Today()
        dir1 = GlobalVariable.logpath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.logpath + "/" + date
        File.make_foloder(dir2)

        logpath = dir2 + "/log.txt"

        writelog.log_Set_Path(logpath)

        writelog.log_Write(msg)

        t = File.get_TodayTime()
        self.txt_Log.append(t + "_" + msg)
        
       
    #제드 이미지 서브스크라이버 콜백 함수
    #데이터 리스트 처리 후 cv2로 변환 후 QImage로 변환
    def ZED_Image_Callback(self,data): #왼쪽캠
        #print(data.data)
        imageArray = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        #print(imageArray)
        im_gray = cv2.cvtColor(imageArray, cv2.COLOR_BGR2RGB)
        #im_gray = cv2.resize(im_gray,dsize=(self.lblVideo.width()-10,self.lblVideo.height()), interpolation=cv2.INTER_AREA)
        self.tmpSaveImage = im_gray
        self.tmpSaveVideo_ImageArray.append(self.tmpSaveImage)
        
        #self.zed_rate.sleep()
        
    def ZED_Image_Callback2(self,data2): #오른쪽캠
        
        imageArray2 = np.frombuffer(data2.data, dtype=np.uint8).reshape(data2.height, data2.width, -1)
        im_gray2 = cv2.cvtColor(imageArray2, cv2.COLOR_BGR2RGB)
        #im_gray2 = cv2.resize(im_gray2,dsize=(self.lblVideo.width(),self.lblVideo.height()), interpolation=cv2.INTER_AREA)
        self.tmpSaveImage2 = im_gray2
        self.tmpSaveVideo_ImageArray2.append(self.tmpSaveImage2)
        
        #self.zed_rate.sleep()
        
    def ZED_Confidence_Image_Callback(self, msg):
        
        # imageArray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # im_gray3 = cv2.cvtColor(imageArray, cv2.COLOR_RGB2GRAY)
        
        # #im_gray3 = cv2.resize(im_gray3,dsize=(self.lblVideo.width(),self.lblVideo.height()), interpolation=cv2.INTER_AREA)
        
        # self.tmpConImage = im_gray3
        # self.tmpSaveVideo_ImageArray3.append(self.tmpConImage)
        pass
        
    def ZED_pathMap_Callback(self,msg):
        
        pass
    
    def ZED_pathOdom_Callback(self,msg):
        
        pass
    
    def ZED_Depth_Callback(self,msg):
        
        self.tmpDepth = str(msg.data)
        
    def Monitor_Callback(self,msg):
        
        imageArray = np.array(msg.data)
        #print(imageArray)
        #imageArray = np.array(imageArray, dtype=np.uint8).reshape(288,-1,3)
        imageArray = np.frombuffer(msg.data, dtype=np.uint8).reshape(288,-1, 3)
        #print(imageArray)
        self.MonitorImage = imageArray
        self.tmpSaveVideo_ImageArray3.append(self.MonitorImage)
        #image = imim.fromarray(imageArray.astype(np.uint8))
        #print(image)
        #img = cv2.cvtColor(imageArray, cv2.COLOR_BGR2RGB)
        #img = cv2.resize(img,dsize=(self.lbl_MonitorVideo.width()-10,self.lbl_MonitorVideo.height()),interpolation=cv2.INTER_AREA)
        
        # image = cv2.cvtColor(imageArray, cv2.COLOR_BGR2RGB)
        #cv2.imwrite('test.jpg',imageArray)
        
        
        

        
    
    def Odom_Callback(self, msg):
        
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        
        self.dictOdom = dict(theta = round(z,5), x = round(x,3), y = round(y,3))
        
    def OFastSlam_Callback(self,msg):
        
        
        imageArray = np.frombuffer(msg.data, dtype=np.uint8).reshape(1600, 1650, -1)
        #imageArray = np.frombuffer(msg.griddata, dtype=np.uint8).reshape(1600, 1650, -1)
        self.LidarImage = imageArray
        
        # print(msg.griddata)
        # print('--------------------------------------------------------')
        
        #image = np.array(msg.griddata, dtype=np.uint8).reshape(1600, 1650, -3)
        #print(image)
        #self.LidarImage = image
        # image = image.astype(np.uint8)
        # print(image)
        # image = np.array(image).reshape(1600, 1650, -1)
        # print(image)
        
        
        

    def Lidar_Scan_Callback(self,msg):
       
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs
        
        gl_range = list(msg.ranges)
        
        
        #print(msg.ranges)
        #print(len(gl_range))
        
        for i in range(len(gl_range)):
            # if gl_range[i] == 0.0 or gl_range[i] == "" or gl_range[i] == None:
            #     gl_range[i] = 0.00
            # else:
            gl_range[i] = round(gl_range[i],2)

        #print(gl_range)
        self.dictRange = dict(range = gl_range)
        tmpdict = dict(self.dictRange, **self.dictOdom)
        
        stampdict = {str(secs)+"."+ str(nsecs): tmpdict}
        self.tmpstampdic[str(secs)+"."+ str(nsecs)] = tmpdict
        
        mapdict = {'map': stampdict}
        self.tmpslamMapdic = {'map': self.tmpstampdic}
        
        self.slamMap = mapdict
        self.slamMaps = self.tmpslamMapdic
        
        #print(self.slamMap)
        #print(self.slamMaps)
        
        
        with open('./JSON/Currentmapdata.json', 'w') as outfile:
            json.dump(self.slamMap, outfile)
            
        with open('./JSON/Stackmapdata.json', 'w') as outfile:
            json.dump(self.tmpslamMapdic, outfile)
        
        
        
    #여기서부터는 수정필요 슬램관련
        
    def map_metadata_Callback(self, msg):
        
        pass
        
        
    def map_occupancy_Callback(self, msg):
        
        # np_data = np.array([self._color_converter(e) for e in msg.data])
        # reshaped = np.flipud(np.reshape(np_data, (msg.info.height, msg.info.width)))
        # imageio.imwrite("itest.png", reshaped, format='png', optimize=True, quantize=4)
        # imageArray = np.frombuffer(msg.data, dtype=np.uint32).reshape(msg.height, msg.width, -1)
        # print(msg.data)
        # self.tmpMap = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # print(self.slamMap)
        
        np_data = np.array([color_converter(e) for e in msg.data])
        reshaped = np.flipud(np.reshape(np_data, (msg.info.height, msg.info.width)))
        image = imim.fromarray(reshaped.astype(np.uint8))
        print(image)
        #plt.imsave('./Output/reshaped.png',image, cmap='gray')
       


    def _color_converter(value):
        if value == -1:
            return 0xcd / 0xff
        return (100 - value) / 100.0
    
    
        #return ogMap


#처음 프로그램 시작시 로딩화면
class splash(QWidget):

    def __init__(self):
        super(splash, self).__init__()

        dir = os.path.dirname(os.path.realpath(__file__))
        uidir = os.path.join(dir, 'splash.ui')
        loadUi(uidir, self)

        self.setWindowTitle('Robot System')
        self.resize(500,100)
        self.setFixedSize(500,100)
        self.Initialize()
        self.Center()
        self.show()

        self.timer.start(200, self)
        

    def Initialize(self):

        self.timer = QBasicTimer()
        self.step = 0
        self.progress = QProgressBar()
        self.lbl_title = QLabel("TEAM NO.2 ROBOT SYSTEM")
        self.splashLayout = QVBoxLayout()

        self.setLayout(self.splashLayout)
        self.splashLayout.addWidget(self.lbl_title)
        self.splashLayout.addWidget(self.progress)

    def Center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def timerEvent(self, e):
        if self.step >= 100:
            self.timer.stop()
            self.close()
            del self.timer
            
            global main, Naviwindow 
            main = robotsystem()
            
            return
        
        elif self.step == 10:
            tmp = self.lbl_title.text() + " SETTING LOADING "
            self.lbl_title.setText(tmp)

        elif self.step > 10:

            tmp = self.lbl_title.text() + ".."
            self.lbl_title.setText(tmp)
        
        self.step = self.step + 10
        self.progress.setValue(self.step)
        
class loginform(QDialog):
    
    def __init__(self):
        
        super(loginform, self).__init__()
        #/home/pkc/djangoServer/project/db.sqlite3
        self.logindb = sqlite('/home/pkc/djangoServer/project/db.sqlite3')
        
        self.UI_init()
        self.show()
    
    def UI_init(self):
        
        dir = os.path.dirname(os.path.realpath(__file__))
        uidir = os.path.join(dir, 'login.ui')
        loadUi(uidir, self)
        
        self.resize(350,100)
        self.setFixedSize(350,100)
        self.Center()
        self.setWindowTitle("Admin Login")
        
        self.MainLayOut = QVBoxLayout()
        self.setLayout(self.MainLayOut)
        
        self.MainLayOutHigh = QHBoxLayout()
        
        self.MainLayOutLow = QHBoxLayout()
        
        self.MainLayOut.addLayout(self.MainLayOutHigh)
        self.MainLayOut.addLayout(self.MainLayOutLow)
        
        self.txtLayout = QGridLayout()
        self.btnLayout = QVBoxLayout()
        self.txtlayOut2 = QHBoxLayout()
                        
        self.MainLayOutHigh.addLayout(self.txtLayout,stretch=3)
        self.MainLayOutHigh.addLayout(self.btnLayout,stretch=1)
        
        self.MainLayOutLow.addLayout(self.txtlayOut2)
        
        self.lblID = QLabel(" ID : ")
        self.lblID.setAlignment(QtCore.Qt.AlignRight)
        self.lblPwd = QLabel(" Password : ")
        self.lblPwd.setAlignment(QtCore.Qt.AlignRight)
        
        self.txtID = QLineEdit()
        self.txtPwd = QLineEdit()
        self.txtPwd.setEchoMode(QLineEdit.Password)
                   
        self.txtLayout.addWidget(self.lblID,0,0)
        self.txtLayout.addWidget(self.lblPwd,1,0)
        self.txtLayout.addWidget(self.txtID,0,1)
        self.txtLayout.addWidget(self.txtPwd,1,1)
        
        self.btnLogin = QPushButton("LOG IN")
        self.btnLogin.clicked.connect(self.btn_Login_clicked)
        
        self.btnLayout.addWidget(self.btnLogin)
        self.lblWarnning = QLabel()
        self.txtlayOut2.addWidget(self.lblWarnning)
        
        
    def Center(self): 
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
    def btn_Login_clicked(self):
        
        if self.txtID.text() != '' and self.txtPwd.text() != '':
            self.logindb.sql_excute("Select username, password FROM auth_user WHERE username='admin'")
            rows = self.logindb.cur.fetchall()
            tmprow = list(rows[0])
            #print(tmprow[1])
            
            if tmprow[0] == self.txtID.text() and self.txtPwd.text() == '8976':
                
                self.lblWarnning.setText("Log IN Success")
                
                self.close()
                global sp
                sp = splash()
            
            else:
                
                self.lblWarnning.setText("warning : Admin ID & Password Check")

        else:
            
            self.lblWarnning.setText("warning : Admin ID & Password Check")
        
def color_converter(value):
    if value == -1:
        return 0xcd / 0xff
    return (100 - value) / 100.0


#동영상 쓰레드
class VideoWorker(QThread):
    
    changePixmap = pyqtSignal(np.ndarray)
    
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.videoThread = False
        
        self.DestinationClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.Destinationopersrv = robotservice()
        self.Destinationreqsrv = robotserviceRequest()
        
    def run(self):
        
        
            self.videoThread = True
            
            while self.videoThread:
                
                tmpImage = self.parent.tmpSaveImage
                #cvim = self.parent.tmpSaveImage
                #pixmap = QPixmap(tmpImage)
                if self.parent.QRCODEUSE:
                    tmpImage, qr = ImagePro.read_frame(tmpImage)
                    
                    try:
                        if qr == 'A' or qr == 'a':
                            
                            self.Destinationreqsrv.service_number = 4
                            res = self.DestinationClient(self.Destinationreqsrv)
                            print(res)
                            
                        elif qr == 'B' or qr == 'b':
                            
                            self.Destinationreqsrv.service_number = 5
                            res = self.DestinationClient(self.Destinationreqsrv)
                            print(res)
                            
                        elif qr == 'C' or qr == 'c':
                            
                            self.Destinationreqsrv.service_number = 6
                            res = self.DestinationClient(self.Destinationreqsrv)
                            print(res)
                            
                        elif qr == 'D' or qr == 'd':
                            
                            self.Destinationreqsrv.service_number = 7
                            res = self.DestinationClient(self.Destinationreqsrv)
                            print(res)
                            
                        else:
                        
                            pass
                    except :
                        pass        
                        
                    
                    # if ImagePro.result:
                    #     # self.parent.Selfmsg.stamp = rospy.Time.now()
                    #     # self.parent.Selfmsg.data = 1
                    #     self.parent.Selfmsg = True
                    #     self.parent.selpub.publish(self.parent.Selfmsg)
                    # else:
                    #     #self.parent.Selfmsg.stamp = rospy.Time.now()
                    #     # self.parent.Selfmsg.data = 0
                    #     self.parent.Selfmsg = False
                    #     self.parent.selpub.publish(self.parent.Selfmsg)
                        
                self.changePixmap.emit(tmpImage)
                #self.sleep(2)
                #time.sleep(0.1)
        
            
    def stop(self):
        
        self.videoThread = False
        self.quit()
        self.wait(1000)
        
#동영상 쓰레드2
class VideoWorker2(QThread):
    
    changePixmap = pyqtSignal(np.ndarray)
    
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.videoThread = False
        
        self.DestinationClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.Destinationopersrv = robotservice()
        self.Destinationreqsrv = robotserviceRequest()
        
    def run(self):
        
        self.videoThread = True
        
        while self.videoThread:
            
            tmpImage = self.parent.tmpSaveImage2
            
            if self.parent.DetectUSE:
                
                #ImagePro.gender_Detect_frame(tmpImage)
                #ImagePro.face_Detection(tmpImage)
                #face.detect_face(tmpImage)
                tmpImage, site = ImagePro.teacherble_detection(tmpImage)
                
                try:
                    if site == 'A':
            
                        self.Destinationreqsrv.service_number = 4
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                        
                    elif site == 'B':
                        self.Destinationreqsrv.service_number = 5
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    elif site == 'C':
                        self.Destinationreqsrv.service_number = 6
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    elif site == 'D':
                        self.Destinationreqsrv.service_number = 7
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    else:
                        pass
                except:
                    
                    self.changePixmap.emit(tmpImage)
                
            self.changePixmap.emit(tmpImage)
            #self.sleep(2)
            #print("2")
            #time.sleep(0.1)
            

    def stop(self):
        
        self.videoThread = False
        self.quit()
        self.wait(1000)
        
#Depth 쓰레드
class DepthVideoWorker(QThread):
    
    changePixmap = pyqtSignal(QImage)
    
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.DepthThread = False
        
    def run(self):
        
        self.DepthThread = True
        
        while self.DepthThread:
            
            tmpImage1 = self.parent.tmpSaveImage
            tmpImage2 = self.parent.tmpSaveImage2
            
            d1 = cv2.cvtColor(tmpImage1, cv2.COLOR_BGR2GRAY)
            d2 = cv2.cvtColor(tmpImage2, cv2.COLOR_BGR2GRAY)
                
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(d1,d2)

            scaledDisparity = disparity - np.min(disparity)
            if np.max(scaledDisparity) > 0:
                scaledDisparity = scaledDisparity * (255/np.max(scaledDisparity))
            scaledDisparity = scaledDisparity.astype(np.uint8)
            displayHeight, displayWidth = scaledDisparity.shape
            channel = 1
            bytesPerLine = channel * displayWidth
            scaledDisparity = ImagePro.circle_Image(scaledDisparity,int(displayWidth/2)-100,int(displayHeight/2),10,255,255,0,-1)
            qtImg = QImage(scaledDisparity, displayWidth, displayHeight, bytesPerLine, QImage.Format_Indexed8)
                    
            self.changePixmap.emit(qtImg)
            
            #time.sleep(0.1)
            

    def stop(self):
        
        self.DepthThread = False
        self.quit()
        self.wait(1000)


#동영상 저장 쓰레드
class VideoSaveWorker(QThread):
    
    #imagearray = pyqtSignal(QImage)
    
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.videoSaveThread = False
        
    def run(self):
        
        self.videoSaveThread = True
        
        i = 1
        
        date = File.get_Today()
        dir1 = GlobalVariable.videoPath
        File.make_foloder(dir1)
        dir2 = GlobalVariable.videoPath + "/" + date
        File.make_foloder(dir2)
            
        while True:
                
            Videopath = dir2 + "/" + str(i)+ ".avi"
                
            if not os.path.exists(Videopath):
                img_arry = []
                for i in range(len(self.parent.tmpSaveVideo_ImageArray3)):
                    image = cv2.cvtColor(self.parent.tmpSaveVideo_ImageArray3[i],cv2.COLOR_BGR2RGB)
                    #img = cv2.imread(image)
                    height, width, layers = image.shape
                    size = (width,height)
                    img_arry.append(image)
                
                fps = 30
                #out = cv2.VideoWriter(Videopath,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
                out = cv2.VideoWriter(Videopath,cv2.VideoWriter_fourcc(*'DIVX'), 30, size)
                for i in range(len(img_arry)):
                    # writing to a image array
                    out.write(img_arry[i])
               
                break
            else:
                i += 1
       
    def stop(self):
        
        #camera.videoStart = False
        self.videoSaveThread = False
        self.quit()
        self.wait(1000)


#동영상 로드 쓰레드
class VideoLoadWorker(QThread):
    
    updateImage = pyqtSignal(QImage)
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.videoLoadThread = False
        
    def run(self):
        
        self.videoLoadThread = True
        cap = cv2.VideoCapture(self.parent.LoadVideo)

        # 프레임 너비/높이, 초당 프레임 수 확인
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH) # 또는 cap.get(3)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) # 또는 cap.get(4)
        fps = cap.get(cv2.CAP_PROP_FPS) # 또는 cap.get(5)
    
        while cap.isOpened(): 
            ret, frame = cap.read()
            
            if not ret:
                
                break

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            tmpImage = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            self.updateImage.emit(tmpImage)
            time.sleep(0.3)
        
        self.stop()
        self.videoLoadThread = False

    def stop(self):
        
        self.videoLoadThread = False
        self.quit()
        self.wait(1000)
        
#모니터링 스레드        
class MonitorWorker(QThread):
    
    changeMonitor = pyqtSignal(QImage)
    
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.videoThread = False
        
    def run(self):
        
        self.videoThread = True
        
        while self.videoThread:
            
            tmpImage = self.parent.MonitorImage
            #print(tmpImage)
            MonitorImage = cv2.resize(tmpImage,dsize=(self.parent.lbl_MonitorVideo.width()-10,self.parent.lbl_MonitorVideo.height()-10),interpolation=cv2.INTER_AREA)
            moimg = QImage(MonitorImage.data, MonitorImage.shape[1], MonitorImage.shape[0], QImage.Format_RGB888)
            
            self.changeMonitor.emit(moimg)
            #self.sleep(2)
            #time.sleep(0.1)
            
    def stop(self):
        
        self.videoThread = False
        self.quit()
        self.wait(1000)

#슬램 영상쓰레드
class SLAMWorker(QThread):
    
    updateMap = pyqtSignal(QImage)
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.SLAMThread = False
        
    def run(self):
        
        while self.SLAMThread:
       
            
            img = self.parent.LidarImage
            img = cv2.resize(img,dsize=(self.parent.lblSlam.width()-10,self.parent.lblSlam.height()),interpolation=cv2.INTER_AREA)
            #self.tmpSlamSaveImage = img
            im = QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888)
            self.updateMap.emit(im)
        

    def stop(self):
        self.SLAMThread = False
        self.quit()
        self.wait(1000)
        
#뎁스 쓰레드
class DEPTHMWorker(QThread):
    
    updateText = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent
        self.DEPTHThread = False
        
    def run(self):
       
        while self.DEPTHThread:
            depth = self.parent.tmpDepth[0:5]
            self.updateText.emit(depth)
            time.sleep(0.1)
        

    def stop(self):
        
        self.DEPTHThread = False
        self.quit()
        self.wait(1000)


if __name__ == "__main__":
    
    ROS_DEPTH = QProcess()
    print("Launching depthnode")
    program = 'roslaunch robotsystem depthnode.launch'
    ROS_DEPTH.start(program)
    
    rospy.init_node("MainDisplay") #메인폼 노드선언
    #rospy.wait_for_service('robot_service_server')
    
    app = QApplication(sys.argv)
    
    ll = loginform() # loginform
    #sp = splash() #처음로딩화면 시작
    writelog = log() #로그 클래스 선언
    camera = Cam() #카메라 클래스 선언
    scout = scout_mini() #cls_car.py 스카웃 클래스 선언
    slam = slamdata() #슬램 클래스 선언
    ImagePro = imageprocessing()
    # args = get_args()
    # depth = args.depth
    # width = args.widthreshaped
    sys.exit(app.exec_())
    #app.exec_()
        




