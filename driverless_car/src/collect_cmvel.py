# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 14:42:22 2017

@author: rohit
"""

from geometry_msgs.msg import Twist
import rospy
import sys, select, termios, tty
import pickle
def callback(msg):
    global st
    st.append(msg)
    print"yo"
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
st=[]
rospy.init_node("dekhlebro",anonymous=False)
rospy.Subscriber("cmd_vel",Twist,callback)
f=open("forward_back_vel.save","wb")
while(1):
    k=getKey()
    if k=='a':
        print st
        pickle.dump(st,f)
        f.close()
        break
    

