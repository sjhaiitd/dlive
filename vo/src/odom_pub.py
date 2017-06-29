#!/usr/bin/env python
import rospy
#from encoders.msg import encoder_msg
from std_msgs.msg import String
import numpy as np
import math
from math import cos,sin,pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3
import time
global pos_x,pos_y,th,vx,vy,vth,y_ini,del_max
from sensor_msgs.msg import Imu
flag=0
y_ini=0

def callback(data):
    #rospy.loginfo( data)
    #y=data
    global pos_x,pos_y,th,vx,vy,vth,y_ini,del_max,flag,t_ini
    #flag=0
    print flag
    if(flag==0):
	flag=1
	pos_x=0.0
	pos_y=0.0
	th=0.0
	vx=0.4
	vy=0.0
	vth=0
	y_ini=0
	del_max=50
	
    
    x=str(data)
    print x
    print "..........."
    buffer_GPS=[]
    buffer_IMU=[]
    if('$GPS' in x):
        commas_gps_data=[pos for pos,char in enumerate(x) if char==',']
        lat=x[(commas_gps_data[0]+5):commas_gps_data[1]]
        lon=x[(commas_gps_data[1]+6):]
        #buffer_GPS.append([float(int(lat*10000))/10000,float(int(lon*10000))/10000,(rospy.Time.now()).to_sec()]) 
        print "GPS_data \n",[lat,lon]
    if('$IMU' in x):
    	commas_imu_data=[pos for pos,char in enumerate(x) if char==',']
        r=float(x[(commas_imu_data[0]+3):commas_imu_data[1]])
        p=float(x[(commas_imu_data[1]+3):commas_imu_data[2]])
        y=float(x[(commas_imu_data[2]+3):])
    	#buffer_IMU.append([float(r),float(p),float(y),(rospy.Time.now()).to_sec()])
        print "IMU_data\n",[r,p,y]
    
    ##publishing odometry data
    
  
    if(int(y)<0):
        if(int(y)>-90):
            y2=y/90*65
        
    if (int(y)<-90) :
        y1=(y-(-90))/90*85
        y2=-65+y1
    
    if (int(y)>0):
        if(int(y)<90):
            y2=y/90*104
        
    if (int(y)>90 ):
        y1=(y-90)/90*106
        y2=104+y1
    
    print "changed _y  ", y2
    if (y2<0):
        y2=360-abs(y2)
    print "new_yaw ", y2

    if y_ini==0:
        y_ini=y2
	t_ini=time.time()
	#print "////////////////////sensor_msgs/Imu
#////////////////////////////////////////////////////////////////////////////////////////"
    else:
        del_th=y2-y_ini
        if(abs(del_th)>del_max):
            if(del_th>0):
                del_th=-(360-y2)-y_ini
            else:
                del_th=y2-(360-y_ini)
        y_ini=y2        
        
	t_now=time.time()
	dt=t_now-t_ini	
	vth=del_th/dt
	print "omega ",vth
	delta_x=(vx*cos(th))*dt
	delta_y=int((vx*sin(th))*dt)
	delta_th=vth*dt
	pos_x+=delta_x
	pos_y+=delta_y
	th+=delta_th
	current_time = rospy.Time.now()
	print "X, Y, theta",[pos_x,pos_y,th]

	imu_msg=Imu()
    imu_msg.header.stamp=current_time
    imu_msg.header.frame_id="imu"
    imu_msg.orientation=Quaternion(odom_quat[0],odom_quat[1],odom_quat[2],odom_quat[3])
    imu_msg.angular_velocity=Vector3(0,0,vth)
    imu_msg.linear_acceleration=Vector3(0,0,0)
    imu_msg.orientation_covariance[0]=0.01
    imu_msg.orientation_covariance[4]=0.01
    imu_msg.orientation_covariance[8]=999
    imu_msg.angular_velocity_covariance[0]=0.01
    imu_msg.angular_velocity_covariance[4]=0.01
    imu_msg.angular_velocity_covariance[8]=999
    imu_msg.linear_acceleration_covariance[0]=100
    imu_msg.linear_acceleration_covariance[0]=100
    imu_msg.linear_acceleration_covariance[0]=100

    imu_topic.publish(imu_msg)

th=0	
#y=encoder_msg()
z=String()

odom_pub=rospy.Publisher("odom",Odometry,queue_size=50)
odom_broadcaster=tf.TransformBroadcaster()
        
rospy.init_node('listener', anonymous=True)
#rospy.Subscriber("enc_talker", encoder_msg, callback)
rospy.Subscriber("GPS_talker",String, callback)
rospy.Subscriber("IMU_talker",String, callback)
imu_topic=rospy.Publisher("imu_data",Imu,queue_size=1000)
rospy.spin()
 
     
        
      

