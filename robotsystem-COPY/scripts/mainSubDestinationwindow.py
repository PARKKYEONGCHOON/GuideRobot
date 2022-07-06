#!/usr/bin/python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore, QtGui
from PyQt5 import uic
from python_qt_binding import loadUi

import os
import rospy

from robotsystem.srv import *
from std_msgs.msg import Bool

#from mainwindow import *

form_class2 = uic.loadUiType("navi.ui")[0]

class Navi(QDialog,form_class2):
    
    # def __init__(self):
    #     super().__init__()
    def __init__(self,parent):
        
        super().__init__(parent)
        
        self.DestinationClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.Destinationopersrv = robotservice()
        self.Destinationreqsrv = robotserviceRequest()
        
        self.UI_init()
        self.show()
        
        
    def UI_init(self):
        
        #QT Creator UI File Load
        dir = os.path.dirname(os.path.realpath(__file__))
        uidir = os.path.join(dir, 'navi.ui')
        loadUi(uidir, self)
        
        self.resize(300,300)
        self.setFixedSize(300,300)
        self.Center()
        self.setWindowTitle("Robot System Navigation")
        
        self.MainLayOut = QGridLayout()
        self.setLayout(self.MainLayOut)
        
        self.btnNaviGroup = QButtonGroup()
        self.btnNaviGroup.setExclusive(False)
        self.btnNaviGroup.buttonClicked[int].connect(self.btn_Navi_Clicked)
        
        # self.lblempty1 = QLabel()
        # self.lblempty2 = QLabel()
        # self.lblempty3 = QLabel()
        # self.lblempty4 = QLabel()
        # self.lblempty5 = QLabel()
        
        self.btn_Asite = QPushButton("A Site")
        self.btn_Bsite = QPushButton("B Site")
        self.btn_Csite = QPushButton("C Site")
        self.btn_Dsite = QPushButton("D Site")

        self.btnNaviGroup.addButton(self.btn_Asite,1)
        self.btnNaviGroup.addButton(self.btn_Bsite,2)
        self.btnNaviGroup.addButton(self.btn_Csite,3)
        self.btnNaviGroup.addButton(self.btn_Dsite,4)
        
        self.MainLayOut.addWidget(self.btn_Asite,0,0)
        self.MainLayOut.addWidget(self.btn_Bsite,0,1)
        self.MainLayOut.addWidget(self.btn_Csite,1,0)
        self.MainLayOut.addWidget(self.btn_Dsite,1,1)
        
        # self.MainLayOut.addWidget(self.btn_Asite,0,0)
        # self.MainLayOut.addWidget(self.lblempty1,0,1)
        # self.MainLayOut.addWidget(self.btn_Bsite,0,2)
        # self.MainLayOut.addWidget(self.lblempty2,1,0)
        # self.MainLayOut.addWidget(self.lblempty3,1,1)
        # self.MainLayOut.addWidget(self.lblempty4,1,2)
        # self.MainLayOut.addWidget(self.btn_Csite,2,0)
        # self.MainLayOut.addWidget(self.lblempty5,2,1)
        # self.MainLayOut.addWidget(self.btn_Dsite,2,2)
        
        
    def Center(self): 
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
            
    def btn_Navi_Clicked(self,id):
        
        #self.Destinationreqsrv.stamp = rospy.Time.now()
        self.Destinationreqsrv.service_number = 3 + id # A~D Site
        
        res = self.DestinationClient(self.Destinationreqsrv)
        print(res)
        
        # if id == 1:
            
        #     self.reqsrv.service_number = 4 #A Site
        
        # elif id == 2:
            
        #     self.reqsrv.service_number = 5 
        
        # elif id == 3:
            
        #     self.reqsrv.service_number = 6 
        
        # elif id == 4:
            
        #     self.reqsrv.service_number = 7 
        

