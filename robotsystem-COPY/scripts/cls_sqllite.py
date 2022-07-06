#!/usr/bin/python3
# -*- coding: utf-8 -*-


# SQLlite 클래스
# 수정필요

import sqlite3


class sqlite:

    def __init__(self,dbname):
        
        try:
        
            self.conn = sqlite3.connect(dbname) #DB 파일 열고 커넥트
            self.cur = self.conn.cursor()
            
        except:
            
            print("Data Base Connect Fail")

    def sql_excute(self,sql): #쿼리 날리고 저장하고 닫고
        
        try:

            self.cur.execute(sql)
            #self.conn.commit()
            #self.conn.close()
            
        except:
            
            print("SQL EXCUTE Fail")



        
        