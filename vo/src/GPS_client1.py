#!/usr/bin/python          

import socket               # Import socket module
import sys
import rospy
from std_msgs.msg import String
s = socket.socket()         # Create a socket object
#host = socket.gethostname() # Get local machine name
host= '10.10.10.20'
port = 12363       # Reserve a port for your service.
c=0;
s.connect((host, port))
rospy.init_node('GPS_client', anonymous=True)
pub=rospy.Publisher('GPS_talker', String, queue_size=1000 )
while(1):
    #c=c+1
    #print c
    x=s.recv(1000)
    print x
    pub.publish(x)

