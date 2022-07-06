#!/usr/bin/python3
# -*- coding: utf-8 -*-


#전역변수 클래스

from cls_INI import *

diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)

class GlobalVariable:

    rootPath = diretory
    
    iniFile = Inifile()
    iniFile.InIfile_Set_Path('Setting.ini')

    
    #폼사이즈 XY, 이미지 비디오 경로, 슬램 경로, 로그 경로, 카메라 넘버
    FormSizeX = Inifile.InIfile_ReadValue(iniFile,'Setting','FormSizeX') 
    FormSizeY = Inifile.InIfile_ReadValue(iniFile,'Setting','FormSizeY')
    imagePath = Inifile.InIfile_ReadValue(iniFile,'Setting','imagePath')
    videoPath = Inifile.InIfile_ReadValue(iniFile,'Setting','videoPath')
    slamimagepath = Inifile.InIfile_ReadValue(iniFile,'Setting','slamimagepath')
    slamvideopath = Inifile.InIfile_ReadValue(iniFile,'Setting','slamvideopath')
    tempImagePath = Inifile.InIfile_ReadValue(iniFile,'Setting','tempImagePath')
    logpath = Inifile.InIfile_ReadValue(iniFile,'Setting','logpath')
    CameraNum = 0