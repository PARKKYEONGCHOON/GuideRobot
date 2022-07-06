#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, Transform, Quaternion, Vector3
from scout_msgs.msg import ScoutLightCmd


class scout_mini:
    
    def __init__(self):
        
         #스카웃 메세지, 퍼블리셔
        self.Tmsg = Twist()
        self.ScoutLightmsg = ScoutLightCmd()
        
        self.Movepub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.ScoutLightpub = rospy.Publisher("/scout_light_control",ScoutLightCmd,queue_size=1)
        
    
  
        