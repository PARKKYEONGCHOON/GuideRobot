#!/usr/bin/python3
# -*- coding: utf-8 -*-


#INI 파일 클래스

import configparser
import os

class Inifile:
    
    def __init__(self):
        
        self.config = configparser.ConfigParser()
        self.Inipath = 'temporary.ini'
        
    def InIfile_Set_Path(self,path): # 파일 저장 경로 셋
        self.Inipath = path
    
    def InIfile_Exist(self,path): # 파일 저장경로 존재여부
        return os.path.exists(path)
    
    def InIfile_ReadValue(self,Section,Key): #INI 파일 Read
        
        self.config.read(self.Inipath, encoding='utf-8')
        
        if Section in self.config.sections(): #섹션 존재여부
            
            if Key in self.config[Section].keys(): #키 존재 여부
                
                return self.config[Section][Key]
    
    def InIfile_ReadValue_Path(self,Section,Key,Path): #INI 파일 READ 경로까지 체크
        
        if os.path.exists(Path):
            
            self.config.read(self.Inipath, encoding='utf-8')
        
            if Section in self.config.sections(): #섹션 존재여부
            
                if Key in self.config[Section].keys(): #키 존재 여부
                
                    return self.config[Section][Key]
                
    
    def InIfile_WriteValue(self,Section,Key,Value): #INI 파일 Write
        
        self.config.read(self.Inipath, encoding='utf-8')
        
        if Section in self.config.sections(): #섹션 존재여부
            
            if Key in self.config[Section].keys(): #키 존재 여부
                
                self.config.set(Section,Key,Value)
                
            else:
                
                self.config[Section][Key] = Value  
                
        else:
            
            self.config.add_section(Section) #섹션 추가
            self.config[Section][Key] = Value
            
            
        
        with open(self.Inipath, 'w+') as configfile:
                    self.config.write(configfile)
            
    
        