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
import geodesy.utm
from numpy.linalg import inv

y_ini=0
position=np.array([0.0,0.0,0.0])   #kalman state [x,y,orientation]
position_GPS=np.array([0.0,0.0,0.0])
velocity=[0.0,0.0,0.0]   #velocity=[vx,vy,vth]
accel=[0.0,0.0,0.0]      #accel=[ax,ay,ath]
flag_GPS=0         #used to check if GPS data is available before publishing odometry data

##kalman parameters
A=np.array([[1.0,1.0,0.5],[0.0,1.0,1.0],[0.0,0.0,1.0]])  ## X=A*X_ini+k*U   R=measurement noise  P=covariance matrix   H=measurement vector 
X=np.array([0.0,0.0,1.0])                    ##X=state vector
P=np.array([[0.05,0.0,0.0],[0.0,100.0,0.0],[0.0,0.0,100.0]]) ##covariance matrix
R=np.array([1.0])                        ##measurement error   
H=np.array([1.0,0.0,0.0])                    ##measurement matrix
Z=np.array([0.0])                        ##to store the measurement of GPS (x,y)
covar=[P,P,P] nm                                                                                
I=np.eye(3.0)
buffer_POS=[]
buffer_IMU=[]
buffer_GPS=[]
buffer_GPS_utm=[]
position_vo_buffer=[]
del_pos_max=100000
pos_max=10000000000
del_max=50
def filter1(X,Z,P,dt):      #kalman filter implementation
	A=np.array([[1.0,dt,0.5*dt*dt],[0.0,1.0,dt],[0.0,0.0,1.0]])
	X=A*X
	P = A * P * A.transpose()
	y = Z.transpose() - (H * X)
	S = H * P * H.transpose() + R
	det=np.linalg.det(S)
	print "determinant", det
	if(det!=0):
		K = P * H.transpose() * inv(S)
		print "determinant 0 hai"
	else:
		K=1
	X = X + (K * y)
	P = (I - (K * H)) * P 
	print 'x= ',X
	
	print 'P= ',P
	while(1):
		a=1
	#return[X,P]

def callback(data):
	global del_max,buffer_POS,buffer_IMU,buffer_GPS,buffer_GPS_utm,position_vo_buffer,del_pos_max,pos_max,flag_accel,flag_vel,position,position_GPS,accel,velocity,flag_GPS,A,X,P,R,H,Z,covar    
	x=str(data)
	print "..........."
	if('$GPS' in x):
		commas_gps_data=[pos for pos,char in enumerate(x) if char==',']
		b=x.find('/GPS')
		lat=float(x[(commas_gps_data[0]+5):commas_gps_data[1]])
		lat=float(int(lat*10000))/10000
		lon=float(x[(commas_gps_data[1]+6):b])
		lon=float(int(lon*10000))/10000
		if(int(lat!=0) & int(lon!=0)):
			buffer_GPS.append([float(int(lat*10000))/10000,float(int(lon*10000))/10000,time.time()]) 
			GPS_utm=geodesy.utm.fromLatLong(lat,lon).toPoint()
			buffer_GPS_utm.append([GPS_utm.x,GPS_utm.y])
			flag_GPS=1
		#buffer_GPS=buffer_GPS[-2:]
			print "GPS_data \n",buffer_GPS_utm[-1]

##    if('$IMU' in x):
##        commas_imu_data=[pos for pos,char in enumerate(x) if char==',']
##        r=float(x[(commas_imu_data[0]+3):commas_imu_data[1]])
##        p=float(x[(commas_imu_data[1]+3):commas_imu_data[2]])
##        y=float(x[(commas_imu_data[2]+3):])
##    print "original_yaw", y
##    if(int(y<0)&int(y>=-90)):
##            #if(int(y)>=-90):
##        y2=y/90*65
##        
##    if (int(y)<-90) :
##        y1=(y-(-90))/90*85
##        y2=-65+y1
##
##    if (int(y>=0))&(int(y<90)):
##        #if(int(y)<90):
##        y2=y/90*104
##    
##    if (int(y)>90 ):
##        y1=(y-90)/90*106
##        y2=104+y1
##
##    print "changed _y  ", y2
##    if (y2<0):
##        y2=360-abs(y2)
##    print "new_yaw ", y2
##    buffer_IMU.append([float(r),float(p),float(y2),time.time()])
##    buffer_IMU=buffer_IMU[-2:]
##    if(len(buffer_IMU)<2):
##        #t_ini=time.time()
		#print "IMU_data\n",buffer_IMU

	if('$VO' in x):
		#print x
		commas=[pos for pos,char in enumerate(x) if char==',']
		if(x[commas[0]+3]=='-'):
			vo_x=-1*float(x[commas[0]+4:commas[1]])
		else:
			vo_x=float(x[commas[0]+3:commas[1]])
		if(x[commas[1]+3]=='-'):            
			vo_y=float(x[commas[1]+4:commas[2]])*(-1)
			print "vo_y",x[commas[1]+4:commas[2]]
		else:           
			vo_y=float(x[commas[1]+3:commas[2]])
		if(x[commas[2]+3]=='-'):
			vo_o=-1*float(x[commas[2]+4:])
		else:
			vo_o=float(x[commas[2]+3:])
		position_vo_buffer.append([vo_x,vo_y,vo_o,time.time()])
		#print (position_vo_buffer[-1])
	# if('$POS' in x):
	# 	pos_index=x.find('$POS')
	# 	#print x
	# 	position=float(x[(pos_index+5):])
	# 	buffer_POS.append([position,time.time()])
	# 	buffer_POS=buffer_POS[-2:]
	if(flag_GPS==1):
	   
	##    if(len(buffer_IMU)>=2):
	##  del_th=buffer_IMU[-1][2]-buffer_IMU[-2][2]
	##  t_now=time.time()
	##  dt=t_now-t_ini
	##  t_ini=t_now
	##  if(abs(del_th)>del_max):
	##      if(del_th>0):
	##                  del_th=-(360-buffer_IMU[-1][2])-buffer_IMU[-2][2]  ##omega negative
	##      else:
	##          del_th=buffer_IMU[-1][2]+(360-buffer_IMU[-2][2])   ##omega positive
	##  if(abs(del_th)<0.01):
	##      del_th=0
	##  vth=del_th/dt
	##  th+=del_th
	##    if(len(buffer_POS)>=2):
	##  del_pos=buffer_POS[-1][0]-buffer_POS[-2][0]
	##  dt=buffer_POS[-1][1]-buffer_POS[-2][1]
	##  if(del_pos>del_pos):
	##      if(del_pos<0):
	##          del_pos=buffer_POS[-1][0]+(pos_max-buffer_POS[-2][0])  ##vx positive
	##      else:
	##          del_pos=-(pos_max-buffer_POS[-1][0])-buffer_POS[-2][0]  #-ve vx
	##  vx=del_pos/dt   
		
		if(len(buffer_GPS)>=2):
			delta_x_utm=buffer_GPS_utm[-1][0]-buffer_GPS_utm[-2][0]
			delta_y_utm=buffer_GPS_utm[-1][1]-buffer_GPS_utm[-2][1]
			if(int(delta_x_utm==0) | int(delta_y_utm==0)):
				delta_x_utm=0.00001
				delta_y_utm=0.00001
			orient=math.atan2(delta_y_utm,delta_x_utm)
			position_GPS+=np.array([delta_x_utm,delta_y_utm,orient])
			dt=buffer_GPS[-1][2]-buffer_GPS[-2][2]
			for i in range(3):
				X=np.array([position[i],velocity[i],accel[i]])
				Z=np.array([position_GPS[i]])
				filter1(X,Z,covar[i],dt)
				print "filter................",filter_result
				position=X
				covar[i]=P
		   		#print "position,,,,,,,,,,,,,,,,,,,,,,,,,,",position
		#delta_x=(vx*cos(th))*(buffer_POS[-1][1]-buffer_POS[-2][1])
		#delta_y=int((vx*sin(th))*(buffer_POS[-1][1]-buffer_POS[-2][1]))
		#position[0]+=delta_x
		#position[1]+=delta_y
		if(len(position_vo_buffer)>2):
				dt=position_vo_buffer[-1][3]-position_vo_buffer[-2][3]
				del_x=position_vo_buffer[-1][0]-position_vo_buffer[-2][0]
				del_y=position_vo_buffer[-1][1]-position_vo_buffer[-2][1]
				del_th=position_vo_buffer[-1][2]-position_vo_buffer[-2][2]
				position[0]+=del_x
				position[1]+=del_y
				position[2]+=del_th
				velocity.append([del_x/dt,del_y/dt,del_th/dt,time.time()])
				if(len(velocity)>=2):

					dt=velocity[-1][3]-velocity[-2][3]
					print "drdr",dt
					accel_x=(velocity[-1][0]-velocity[-2][0])/dt
					accel_y=(velocity[-1][1]-velocity[-2][1])/dt
					accel_th=(velocity[-1][2]-velocity[-2][2])/dt
					accel=[accel_x,accel_y,accel_th]
		
		current_time = rospy.Time.now()
		print "X, Y, theta",[position[0],position[1],position[2]]
		while(1):
			a=1
		odom_quat=tf.transformations.quaternion_from_euler(0,0,position[2])
		odom_broadcaster.sendTransform((position[0],position[1],0),odom_quat,current_time,"base_link","odom")
		odom_msg=Odometry()
		odom_msg.header.stamp=current_time
		odom_msg.header.frame_id="odom"
		odom_msg.pose.pose=Pose(Point(position[0],position[1],0),Quaternion(*odom_quat))
		odom_msg.child_frame_id="base_link"
		odom_msg.twist.twist=Twist(Vector3(0,0,0),Vector3(0,0,0))
		odom_pub.publish(odom_msg)
	


odom_pub=rospy.Publisher("odom",Odometry,queue_size=50)
odom_broadcaster=tf.TransformBroadcaster()        
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("enc_talker", String, callback)
rospy.Subscriber("GPS_talker",String, callback)
rospy.Subscriber("IMU_talker",String, callback)
rospy.Subscriber("vo_pub",String, callback)
rospy.spin()
 
	 
		
	  
