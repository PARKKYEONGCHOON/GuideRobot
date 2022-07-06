#!/usr/bin/python3
# -*- coding: utf-8 -*-

from unicodedata import name
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore, QtGui
from PyQt5 import uic
from python_qt_binding import loadUi
from cls_sqllite import *

import mainwindow

import os
import rospy
import sys
import hashlib
from Crypto.Cipher import AES
import base64

diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)


form_class2 = uic.loadUiType("login.ui")[0]

class loginform(QDialog,form_class2):
    
    def __init__(self):
        
        super().__init__()
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
                
                aa = mainwindow.splash()
            
            else:
                
                self.lblWarnning.setText("warning : Admin ID & Password Check")

        else:
            
            self.lblWarnning.setText("warning : Admin ID & Password Check")
        
    
    
    
# if __name__ == "__main__":
        
#     app = QApplication(sys.argv)
        
#     ddd = loginform()
        
#     sys.exit(app.exec_())
    
        
        
        