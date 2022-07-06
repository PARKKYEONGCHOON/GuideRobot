#!/usr/bin/python3
# -*- coding: utf-8 -*-

from matplotlib.pyplot import cla
import rospy
import json
import websocket
import _thread
import time
import rel
import sys
from turtlesim.msg import Pose
from robotsystem.srv import *

# ws = websocket.WebSocket()
# ws.connect("ws://127.0.0.1:8000/ws/robot/guy/")

# ws.connect("ws://localhost:8000/ws/robot/guy/")
# ws.connect("ws://192.168.1.102:30000/ws/robot/guy/")

# ws://127.0.0.1:8000/ws/robot/guy/
class djangoClient:
    
    def __init__(self):
        
        self.DestinationClient = rospy.ServiceProxy("robot_service_server", robotservice)
        self.Destinationopersrv = robotservice()
        self.Destinationreqsrv = robotserviceRequest()
        
        self.SocketClientInit()
        
    def SocketClientInit(self):
        
        websocket.enableTrace(True)
        if len(sys.argv) < 2:
            host = "ws://127.0.0.1:8000/ws/testcarapp/default/"
            #host = "ws://127.0.0.1:8000/ws/testcarapp/LiveChat/"
            
        else:
            host = sys.argv[1]
        ws = websocket.WebSocketApp(host,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)
        ws.on_open = self.on_open
        ws.run_forever()

    def on_message(self,ws, message):
        print("============Message==============")
        #print(message)
        #print(len(message))
        #print(message[13])
        tmpmsgdic = json.loads(message)
        if "message" in tmpmsgdic:
            
            tmpcmd = tmpmsgdic['message']
            print(tmpcmd)
            
            if tmpcmd[0:4] == 'cmd-':
                
                #print(1)
                if tmpcmd[4:8] == 'dir-':
                    
                    #print(2)
                    
                    if tmpcmd[8] == 'a':
                        
                        print("A Site")
                        self.Destinationreqsrv.service_number = 4 # A~D Site
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                        
                    elif tmpcmd[8] == 'b':
                        
                        print("B Site")
                        self.Destinationreqsrv.service_number = 5 # A~D Site
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    
                    elif tmpcmd[8] == 'c':
                        
                        print("C Site")
                        self.Destinationreqsrv.service_number = 6 # A~D Site
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    
                    elif tmpcmd[8] == 'd':
                        
                        print("D Site")
                        self.Destinationreqsrv.service_number = 7 # A~D Site
                        res = self.DestinationClient(self.Destinationreqsrv)
                        print(res)
                    
                    else:
                    
                        pass    
        

    def on_error(self,ws, error):
        print("============Error==============")
        print(error)

    def on_close(self,ws, close_status_code, close_msg):
        print("### closed ###")

    def on_open(self,ws):
        print("Opened connection")


# def callback(data):
#     global ws
#     rospy.loginfo(rospy.get_caller_id() + 'x: %s', data.x)
#     position = {
#         'x': data.x,
#         'y': data.y,
#         'theta': data.theta
#     }
#     message = json.dumps({ 'message' : position })
#     ws.send('%s' % message)


# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('djangoNode', anonymous=True)
#     rospy.Subscriber('turtle1/pose', Pose, callback)

#     SocketClientInit()
    
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
    



if __name__ == '__main__':
    
    rospy.init_node('djangoNode', anonymous=True)
    
    djangoCli = djangoClient()
    
    rospy.spin()
    
    
    
    
    