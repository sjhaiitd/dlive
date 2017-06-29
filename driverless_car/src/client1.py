#!/usr/bin/python           
#!/usr/bin/python
#from __future__ import print_function
#PKG_NAME = 'driverless_car'
#import roslib; roslib.load_manifest(PKG_NAME)
import cv2
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import rospy
import numpy as np
import itertools
import socket
import sys
from decimal import *
from std_msgs.msg import String
rospy.init_node("vo_client", anonymous=True)
s = socket.socket()         # Create a socket object
#host = socket.gethostname() # Get local machine name
host= '10.10.10.12'
port = 22338      # Reserve a port for your service.
c=0;
s.connect((host, port))
#plt.ion()
vo_pub=rospy.Publisher("vo_pos",String,queue_size=100)

#output=open('data1.txt','wb')
while(1):
    x=s.recv(1000)
    #x="$VO ,x=12 ,y=13"
    vo_pub.publish(x)
s.close    	                 # Close the socket when done
