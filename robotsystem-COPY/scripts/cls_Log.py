#!/usr/bin/python3
# -*- coding: utf-8 -*-

#로그 클래스

from encodings import utf_8
import os
from datetime import datetime
import time


class log: 
    def __init__(self):
        
        self.logPath = 'temporary.log'
        
    def log_Set_Path(self,path): #로그 저장 경로 셋
        self.logPath = path
    
    def log_Exist(self,path): # 저장 경로 존재 여부
        return os.path.exists(path)
       
    def log_Write(self,Message): #파일 열고 메세지 쓰기
        
        logfile = open(self.logPath, 'a',encoding='utf-8')
        logfile.write("LOG : " + time.strftime("%Y-%m-%d-%X") + " __ " + Message +'\n')
        logfile.close()