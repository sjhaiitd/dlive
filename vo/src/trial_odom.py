#!/usr/bin/env python
import rospy
#from encoders.msg import encoder_msg
from std_msgs.msg import String
import numpy as np
from math import cos,sin,pi,sqrt
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3
import time
import geodesy.utm
from numpy.linalg import inv
import pickle
from sensor_msgs.msg import Imu,NavSatFix
import utm
y_ini=0
position=np.asmatrix([0.0,0.0,0.0])   #kalman state [x,y,orientation]
position2=np.asmatrix([0.0,0.0,0.0]) 
position_GPS=np.asmatrix([0.0,0.0])
velocity=[]   #velocity=[vx,vy,vth]
accel=[]      #accel=[ax,ay,ath]

flag_GPS=0         #used to check if GPS data is available before publishing odometry data

##kalman parameters
A=np.asmatrix([[1.0,1.0,0.5],[0.0,1.0,1.0],[0.0,0.0,1.0]])  ## X=A*X_ini+k*U   R=measurement noise  P=covariance matrix  H=measurement vector
X=np.asmatrix([0.0,0.0,1.0])                    ##X=state vector
P=np.asmatrix([[0.05,0.0,0.0],[0.0,0.05,0.0],[0.0,0.0,0.05]]) ##covariance matrix
#X1=np.asmatrix([0.0,0.0,1.0])                   
I=np.eye(3.0)
buffer_POS=[]
buffer_IMU=[]
buffer_GPS=[]
buffer_GPS_utm=[]
GPS_s=[]
GPS_s1=[]
my_list=[]
position_vo_buffer=[]
del_pos_max=100000           ##X=state vector
#P1=np.asmatrix([[0.05,0.0,0.0],[0.0,100.0,0.0],[0.0,0.0,100.0]]) ##covariance matrix
R=np.asmatrix([1.0])                        ##measurement error   
H=np.asmatrix([1.0,0.0,0.0])                    ##measurement matrix
Z=np.asmatrix([0.0])                        ##to store the measurement of GPS (x,y)
covar=[P,P,P]                                                                               
I=np.eye(3.0)
buffer_POS=[]
buffer_IMU=[]
buffer_GPS=[]
buffer_GPS_utm=[]
GPS_s=[]
GPS_s1=[]
my_list=[]
position_vo_buffer=[]
del_pos_max=100000
pos_max=10000000000
del_max=50
#output=open('data5.save','wb')
f=open('GPS_smooth3.save','w')
g=open('odom3.save','w')
flag_GPS=0
flag_VO=0
start=False

def smooth1():
 	global GPS_s1,var_x,var_y,start,position_GPS
 	if(len(GPS_s1)==0):
		if(len(buffer_GPS_utm)>=2):
 			mean_ini_x=buffer_GPS_utm[-2][0]
 			mean_ini_y=buffer_GPS_utm[-2][1]
 			var_x=0.0625
 			var_y=0.0625
 			start=True
 	else:
 		mean_ini_x=GPS_s1[-1][0]
 		mean_ini_y=GPS_s1[-1][1]
 		start=True
 	if(len(my_list)>2 and start==True):
 		move_x=my_list[-1][0]-my_list[-2][0]
 		var_mov_x=0.03
 		#mean_ini_x=buffer_GPS_utm[-2][0]
 		#var_x=0.0625
 		mean_e_x=mean_ini_x+move_x
 		var_e_x=var_x+var_mov_x
 		mean_meas_x=buffer_GPS_utm[-1][0]
 		var_meas_x=0.0625
 		mean_u_x=(var_e_x*mean_meas_x+var_meas_x*mean_e_x)/(var_meas_x+var_e_x)
 		var_u_x=var_e_x*var_meas_x/(var_e_x+var_meas_x)
 		move_y=my_list[-1][1]-my_list[-2][1]
 		var_mov_y=0.03
 		#mean_ini_y=buffer_GPS_utm[-2][0]
 		#var_y=0.0625
 		mean_e_y=mean_ini_y+move_y
 		var_e_y=var_y+var_mov_y
 		mean_meas_y=buffer_GPS_utm[-1][1]
 		var_meas_y=0.0625
 		mean_u_y=(var_e_y*mean_meas_y+var_meas_y*mean_e_y)/(var_meas_y+var_e_y)
 		var_u_y=var_e_y*var_meas_y/(var_e_y+var_meas_y)
 		GPS_s1.append([mean_u_x,mean_u_y,time.time()])
 		lat_lon=utm.to_latlon(mean_u_x,mean_u_y,43,'R')
 		current_time=rospy.Time.now()
 		gps_msg=NavSatFix()
 		gps_msg.header.frame_id="GPS"
 		gps_msg.header.stamp=current_time
 		gps_msg.latitude=lat_lon[0]
 		gps_msg.longitude=lat_lon[1]
 		gps_msg.altitude=0
 		gps_pub.publish(gps_msg)

		#print "GPS_s1",GPS_s1[-1]
		
 		var_x=var_u_x
 		var_y=var_u_y
 		if(len(GPS_s1)>=2 and orient==True):
			#flag_GPS=0
			delta_x_utm=GPS_s1[-1][0]-GPS_s1[-2][0]
			delta_y_utm=GPS_s1[-1][1]-GPS_s1[-2][1]
			position_GPS+=np.asmatrix([delta_x_utm,delta_y_utm])
			print "\nposition_GPS\n", position_GPS
			pickle.dump([position_GPS[0,0],position_GPS[0,1]],f)
	


def filter1(X,Z,P,dt):      #kalman filter implementation
	A=np.asmatrix([[1.0,dt,0.5*dt*dt],[0.0,1.0,dt],[0.0,0.0,1.0]])
	#print "X initial\n",X
	X=np.dot(A,X)
	P = np.dot(np.dot(A , P) , A.transpose())
	y = Z.transpose() - np.dot(H,X)
	S = np.dot(np.dot(H , P), H.transpose()) + R
	det=np.linalg.det(S)
	#print "S",S
	#print "determinant", det
	if(det!=0):
		K = P * H.transpose() * inv(S)	
	else:
		K=np.asmatrix([1,1,1]).transpose()
	X = X + np.dot(K , y)
	P = np.dot((I - np.dot(K , H)) , P) 
	print 'x= \n',X
	print 'P= \n',P
	return[X,P]




def callback_VO(data):
	
	#print data
	global position_vo_buffer,flag_VO
	flag_VO=1
	x=str(data)
	#pickle.dump(x,output)	
	commas=[pos for pos,char in enumerate(x) if char==',']
	if(x[commas[0]+3]=='-'):
		vo_x=-1*float(x[commas[0]+4:commas[1]])
	else:
		vo_x=float(x[commas[0]+3:commas[1]])
	if(x[commas[1]+3]=='-'):            
		vo_y=float(x[commas[1]+4:commas[2]])*(-1)
		#print "vo_y",x[commas[1]+4:commas[2]]
	else:           
		vo_y=float(x[commas[1]+3:commas[2]])
	if(x[commas[2]+3]=='-'):
		vo_o=-1*float(x[commas[2]+4:])
	else:
		vo_o=float(x[commas[2]+3:])
	position_vo_buffer.append([vo_x,vo_y,vo_o,time.time()])
	print x
	#odom()
	current_time=rospy.Time.now()
	vo_quat=tf.transformations.quaternion_from_euler(0,0,vo_o)

	vo_msg=Odometry()
	vo_msg.header.stamp=current_time
	vo_msg.header.frame_id="odom"
	vo_msg.pose.pose=Pose(Point(vo_x,vo_y,0),Quaternion(*vo_quat))
	vo_msg.child_frame_id="base_link"
	#vo_msg.twist.twist=Twist(Vector3(vx,vy,0),Vector3(0,0,vth))
	odom_pub.publish(vo_msg)

def callback_GPS(data):
	global flag_GPS,buffer_GPS,buffer_GPS_utm,flag_GPS,my_list
	#flag_GPS=1
	x=str(data)
	#pickle.dump(x,output)
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
		if(len(position_vo_buffer)!=0):
			my_list.append([position_vo_buffer[-1][0],position_vo_buffer[-1][1]])
		smooth1()
		flag_GPS=1

def callback_IMU(data):
	x=str(data)
	if('$IMU' in x):
		commas_imu_data=[pos for pos,char in enumerate(x) if char==',']
		r=float(x[(commas_imu_data[0]+3):commas_imu_data[1]])
		p=float(x[(commas_imu_data[1]+3):commas_imu_data[2]])
		y=float(x[(commas_imu_data[2]+3):])
		#print [r,p,y]
		imu_msg=Imu()
		current_time=rospy.Time.now()
		imu_quat=tf.transformations.quaternion_from_euler(0,0,y)
		imu_msg.header.frame_id="Imu"
		imu_msg.header.stamp=current_time
		imu_msg.orientation=imu_quat
		imu_msg.angular_velocity.x=0
		imu_msg.angular_velocity.y=0
		imu_msg.angular_velocity.z=0
		imu_pub.publish(imu_msg)

odom_pub=rospy.Publisher("odom1",Odometry,queue_size=50)
gps_pub=rospy.Publisher("GPS_topic",NavSatFix,queue_size=50)
imu_pub=rospy.Publisher("IMU_topic",Imu,queue_size=50)
odom_broadcaster=tf.TransformBroadcaster()        
rospy.init_node('listener', anonymous=True)
#rospy.Subscriber("enc_talker", String, callback)
rospy.Subscriber("GPS_talker",String, callback_GPS)
rospy.Subscriber("IMU_talker",String, callback_IMU)
rospy.Subscriber("vo_pub",String, callback_VO)

orient=False
# while(1):
# 	try:
# 		if flag_GPS==1 :
# 			#print "len(GPS_s1)",len(GPS_s1)
# 			# if(len(GPS_s1)>=2 and orient==False )and np.sqrt(((GPS_s1[-1][0]-GPS_s1[-2][0])**2)+((GPS_s1[-1][0]-GPS_s1[-2][0])**2))>1:
# 			# 	orient=True
# 			# 	theta=np.arctan2(GPS_s1[-1][1]-GPS_s1[-2][1],GPS_s1[-1][0]-GPS_s1[-2][0])
# 			# 	#translate=[p_buffer_GPS_utm[-2,0]-p0.x,p_buffer_GPS_utm[-2,1]-p0.y]
# 			# 	transform=np.zeros((4,4))
# 			# 	transform[3,3]=1
# 			# 	transform[2,2]=1
# 			# 	transform[0,3]=0
# 			# 	transform[1,3]=0
# 			# 	transform[0,0]=cos(theta)
# 			# 	transform[0,1]=-sin(theta)
# 			# 	transform[1,0]=sin(theta)
# 			# 	transform[1,1]=cos(theta)
# 			# 	print "\ntransform\n",transform
# 			# 	print "\ntheta\n",theta
			

# 			# if(len(position_vo_buffer)>=2 ):#and int(flag_VO==1)):
# 			# 	flag_VO=0
# 			# 	#print "after getting in flag_VO",flag_VO
# 			# 	#print "vo_buffer_len",len(position_vo_buffer)
# 			# 	dt=position_vo_buffer[-1][3]-position_vo_buffer[-2][3]
# 			# 	if(dt==0.0):
# 			# 		dt=0.000000000001
# 			# 	del_x=position_vo_buffer[-1][0]-position_vo_buffer[-2][0]
# 			# 	del_y=position_vo_buffer[-1][1]-position_vo_buffer[-2][1]
# 			# 	del_th=position_vo_buffer[-1][2]-position_vo_buffer[-2][2]

# 			# 	#print "\nposition after transformation\n",position2
# 			# 	position+=np.asmatrix([del_x,del_y,del_th])
# 			# 	#print "\nposition1\n",position
# 			# 	#print "position ",position
# 			# 	#print "dt", dt
# 			# 	#print "position_ from vo", position
# 				# if(dt==0.0):
# 				# 	dt=0.000000000001
# 				# velocity.append([del_x/dt,del_y/dt,del_th/dt,time.time()])
# 				# if(len(velocity)>=2):
# 				# 	dt=velocity[-1][3]-velocity[-2][3]
# 				# 	#print "drdr",dt
# 				# 	accel_x=(velocity[-1][0]-velocity[-2][0])/dt
# 				# 	accel_y=(velocity[-1][1]-velocity[-2][1])/dt
# 				# 	accel_th=(velocity[-1][2]-velocity[-2][2])/dt
# 				# 	accel=[accel_x,accel_y,accel_th]
# 				# 	velocity=velocity[-1:]


				

# 			# if(len(GPS_s1)>=2 and flag_GPS==1):
# 			# 	flag_GPS=0
# 			# 	delta_x_utm=GPS_s1[-1][0]-GPS_s1[-2][0]
# 			# 	delta_y_utm=GPS_s1[-1][1]-GPS_s1[-2][1]
# 			# 	position_GPS+=np.asmatrix([delta_x_utm,delta_y_utm])
# 			# 	print "\nposition_GPS\n", position_GPS
# 				#pickle.dump([position_GPS[0,0],position_GPS[0,1]],f)
# 				#print "GPS smooth",GPS_s1[-1]
# 				#print "meas",position_GPS
# 				#dt=GPS_s1[-1][2]-GPS_s1[-2][2]
# 				# if(len(accel)==3):
# 				# 	for i in range(2):
# 				# 		X=np.asmatrix([position[0,i],velocity[-1][i],accel[i]])
# 				# 		Z=np.asmatrix([position_GPS[0,i]])
# 				# 		filter_result=filter1(X.transpose(),Z,covar[i],dt)
# 				# 		position[0,i]=filter_result[0][0,0]
# 				# 		covar[i]=filter_result[1]
					
# 			#print "\nposition\n",position
# 			#pickle.dump([position[0,0],position[0,1]],g)
# 	except KeyboardInterrupt:
# 		output.close()
# 		break

rospy.spin()
f.close()
g.close()






















