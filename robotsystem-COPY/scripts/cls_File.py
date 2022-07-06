#!/usr/bin/python3
# -*- coding: utf-8 -*-


#파일 클래스
#저장 디렉토리 생성 및 날짜 확인
import os
import time
import datetime

diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)

class File():
    def __init__(self):
        pass
    
    def get_Today():
        now = time.localtime()
        s = "%04d-%02d-%02d" % (now.tm_year, now.tm_mon, now.tm_mday)
        return s

    def make_foloder(foloderName):
        
        if not os.path.isdir(foloderName):
            os.mkdir(foloderName)
            
    def get_TodayTime():
        now = time.localtime()
        
        s = "%04d-%02d-%02d_%02d:%02d:%02d" % (now.tm_year, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec)
        
        return s

            