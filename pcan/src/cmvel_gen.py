# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 15:21:26 2017

@author: rohit
"""

from geometry_msgs.msg import Twist
import rospy
import sys, select, termios, tty
import pickle
import math
import numpy as np
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _=select.select([sys.stdin], [], [], 0.1)
    if rlist:
       key = sys.stdin.read(1)
    else:
       key='q'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
settings = termios.tcgetattr(sys.stdin)
rospy.init_node("dekhlebro",anonymous=False)
pb=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
f=open("reverse_turn_vel.save","rb")
ls=pickle.load(f)
cnt=0
while(1):
    k=getKey()
    ls[cnt].angular.z=0
    ls[cnt].linear.x=ls[cnt].linear.x*((-1)**cnt)
    if abs(ls[cnt].angular.z)<=0.001:
           steer_d=0
    else:
           radius = ls[cnt].linear.x / ls[cnt].angular.z
           steer_d=math.atan(1.958/radius)
    print "nxt setpoint",ls[cnt].linear.x*18/5
    if k=='a':
        #ls[cnt].angular.z=0
        
        pb.publish(ls[cnt])
        cnt=cnt+1
    if k=='d':
        pb.publish(ls[cnt])
    if k=='s':
        break
    
