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
import pickle
from decimal import *
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
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
def callback(x):
	data=str(x)
	#a=data.find("$VO")
	global x_vo,y_vo,vo_flag
	if ("$VO" in data):
		commas=[pos for pos,char in enumerate(data) if char==',']
                print data
                if data[commas[0]+3]=='-':
                   x_vo=float(data[commas[0]+4:commas[1]])*-1
                else:
		   x_vo=float(data[commas[0]+3:commas[1]])
               
                if data[commas[1]+3]=='-':
		   y_vo=float(data[commas[1]+4:])*-1
                else:
                   y_vo=float(data[commas[1]+3:])
		vo_flag=True
                print x_vo
                print y_vo
s=socket.socket()
host='10.10.10.20'
port=12357
c=0;
factor=10000
x_vo=0
y_vo=0
vo_flag=False
rospy.init_node('GPS_client', anonymous=True)
img=cv2.imread("/home/rohit/scratch/src/driverless_car/maps/img2.jpg",cv2.IMREAD_COLOR)
min_lat=28.5306000
min_lon=77.1618000
data_hai=False

p0=geodesy.utm.fromLatLong(min_lat,min_lon).toPoint()
cv2.namedWindow("Display",0)
rospy.Subscriber("vo_pos",String,callback)
s.connect((host,port))
gps_flag=False
opa=[]
while(1):
	data=s.recv(1000)
	print data
	#data="dsds"
	y=str(data)
        #print y
	a=y.find('$GPS')
	b=y.find('/GPS')
	x=y[a:b]
	#print x
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
         	flag_1=1
                print img.shape
    		cv2.circle(img,(int(p.x-p0.x),int(p.y-p0.y)),1,(0,0,255),-1)
		#print "lovalu"
                if(gps_flag==False):
                   zx=p.x-p0.x
                   zy=p.y-p0.y
                   gps_flag=True
         
        
        if vo_flag==True and gps_flag==True:
                vo_flag==False
                #print "x from spec",x_vo
                #print "y from spec",y_vo
                #print "imgx",zx+x_vo
                #print "imgy",zy+y_vo
                print"aya"
                opa.append([x_vo,y_vo,zx+x_vo,zy+y_vo])
                if zx+x_vo>=0 and zx+x_vo< 7164 and zy+y_vo>=0 and zy+y_vo< 3358:
                   cv2.circle(img,(int(zx+x_vo),int(zy+y_vo)),1,(255,0,0),-1)
        #display_img=cv2.resize(img,(int(img.shape[1]/10),int(img.shape[0]/10)))
	
        cv2.imshow("Display",img)
        cv2.waitKey(1)
    	key=getKey()
        if key=='a':
                break    
f=open("raj1.txt","w")
f.write(str(opa))
f.close()
j=open("kera1.save","wb")
pickle.dump(opa,j)
j.close()
