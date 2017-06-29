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
#import rospy
#from std_msgs.msg import String
s=socket.socket()
host='10.10.10.20'
port=12357
c=0;
factor=10000
rospy.init_node('GPS_client', anonymous=True)
img=cv2.imread("/home/rohit/scratch/src/driverless_car/maps/img2.jpg",cv2.CV_LOAD_IMAGE_COLOR)
min_lat=28.5306000
min_lon=77.1618000
data_hai=False
p0=geodesy.utm.fromLatLong(min_lat,min_lon).toPoint()
cv2.namedWindow("Display",0)
s.connect((host,port))
while(1):
	data=s.recv(1000)
	#print data
	y=str(data)
        #print y
	a=y.find('$GPS')
	b=y.find('/GPS')
	x=y[a:b]
	print x
	if('$GPS' in y):
		commas_gps_data=[pos for pos,char in enumerate(x) if char==',']
		#x=y[(commas_gps_data[0]+1):commas_gps_data[2]]
		lat=Decimal(x[(commas_gps_data[0]+5):commas_gps_data[1]])
		lon=Decimal(x[(commas_gps_data[1]+6):])
		lat1=float(int(lat*factor))/factor
                lon1=float(int(lon*factor))/factor
                data_hai=True
		print lat1,lon1
        if(data_hai==True):
    		p=geodesy.utm.fromLatLong(lat1,lon1).toPoint()
                print "min",p0.x,p0.y
                print "real",p.x,p.y
                print img.shape
    		cv2.circle(img,(int(p.x-p0.x),int(p.y-p0.y)),5,(0,0,255),-1)
		#print "lovalu"
        data_hai=False
        display_img=cv2.resize(img,(int(img.shape[1]/10),int(img.shape[0]/10)))
	
        cv2.imshow("Display",display_img)
        if(cv2.waitKey(1)==27):
    	        break
