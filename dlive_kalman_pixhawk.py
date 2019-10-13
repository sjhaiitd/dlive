#!/usr/bin/env python
import math as m
import rospy 
import time
import numpy as np
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import NavSatFix,Imu,MagneticField
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from e2o.msg import e2o_ctrl, e2o_status
import os

class state():
	def _init_(self,LOOP_FREQUENCY):
		self.iLat = 0
		self.latitude = 0
		self.gps_latitude = 0
		self.lastLat = 0
		self.iLon = 0
		self.longitude = 0
		self.gps_longitude = 0
		self.lastLon = 0
		self.X = 0
		self.last_X = 0
		self.gps_X = 0
		self.past_X = 0
		self.Y = 0
		self.last_Y = 0
		self.gps_Y = 0
		self.past_Y = 0
		self.PosError_X = 0
		self.PosError_Y = 0
		self.position_reset = True
		self.past_PosError_X = self.PosError_X
		self.past_PosError_Y = self.PosError_Y
		self.VelError = 0
		self.past_VelError = 0
		self.heading = 0
		self.Velocity = 0
		self.last_cosmh = 0
		self.last_sinmh = 0
		self.Acceleration = 0
		self.last_Velocity = 0
		self.AccBias = 0
		self.Hdop = 10000
		self.MIN_GPS_HDOP = 2.0
		self.GPS_Velocity = 0
		self.GPS_sAcc = 100
		self.GPS_V_x = 0
		self.GPS_V_y = 0
		self.tick_gps = False
		self.tick_velocity = False
		self.dt = 0.02
		self.ACCEL_VARIANCE = 5e-3 #CHANGED
		self.CIRCULAR_VELOCITY_ERROR = 10 #CHANGED
		self.DEG2METER = 111392.84
		self.METER2DEG = 1/self.DEG2METER
		self.DEG2RADIAN = 1/57.3
		self.RADIAN2DEG = 57.3
		self.odom_Velocity = 0
		self.odom_tick = False
		self.odom_Velocity_error = 1
		self.GRAVITY = 9.81
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
		self.odom_broadcaster = tf.TransformBroadcaster()
		self.roll = 0
		self.pitch = 0
		self.Ax = 0
		self.Ay = 0
		self.Az = 0
		self.Gx = 0
		self.Gy = 0
		self.Gz = 0
		self.min_gps_speed = 2#CHANGED
		self.last_odom_speed = int(-1)#CHANGED
		self.head_error = 1e-2
		self.declination = 0
 
	def magnitude(self,x1,x2,y1,y2):
		return m.sqrt((x1-x2)**2 + (y1-y2)**2)

	def gps_fix_velocity_callback(self,data):
		self.GPS_V_x = data.twist.twist.linear.x
		self.GPS_V_y = data.twist.twist.linear.y
		self.GPS_Velocity = self.GPS_V_x*m.cos(self.heading) + self.GPS_V_y*m.sin(self.heading)
		self.GPS_sAcc = data.twist.covariance[0]
		self.tick_velocity = True

	def gps_fix_callback(self,data):
		self.latitude = data.latitude*1e-7
		self.gps_latitude = self.latitude
		self.longitude = data.longitude*1e-7
		self.gps_longitude = self.longitude
		self.Hdop = data.position_covariance[0]*1e2
		self.tick_gps = True

	def odom_velocity_callback(self,data):
		self.odom_Velocity = float(data.Speed)/3.6 
		self.odom_Velocity_error = 1e2 #CHANGED		
		if( m.fabs(self.odom_Velocity-self.Velocity) < (1/3.6) or self.odom_Velocity==0):#CHANGED. This is to prevent the system from getting biased towards the car's speed
			self.odom_Velocity = self.Velocity
			self.odom_Velocity_error = 0
		self.odom_tick = True

	def Fuse_gps_position(self):
		temp_X = self.gps_X #unused
		temp_Y = self.gps_Y
		self.gps_X = ((self.gps_longitude - self.iLon)*self.DEG2METER)
		self.gps_Y = ((self.gps_latitude - self.iLat)*self.DEG2METER)

		separation = self.magnitude(self.gps_X,self.X,self.gps_Y,self.Y)#CHANGED 
		if(separation > 2 or self.Hdop>self.MIN_GPS_HDOP or self.Velocity<self.min_gps_speed/4): 
			self.position_reset = True 
			self.Hdop = 1e7 
		
		PosGain_X = self.PosError_X/(self.PosError_X + self.Hdop)
		PosGain_Y = self.PosError_Y/(self.PosError_Y + self.Hdop)

		self.X = self.gps_X*PosGain_X + (1-PosGain_X)*self.X
		self.Y = self.gps_Y*PosGain_Y + (1-PosGain_Y)*self.Y

		self.PosError_X *= (1-PosGain_X)
		self.PosError_Y *= (1-PosGain_Y)

	def estimate_declination(self):
		gps_heading = m.atan2(self.GPS_V_y,self.GPS_V_x)
		heading_error = self.heading - gps_heading
		self.head_error += 0.01*self.dt
		if(heading_error>= m.pi): #CHANGED
			heading_error -= 2*m.pi
		if(heading_error<= -m.pi):
			heading_error += 2*m.pi
		if(self.GPS_Velocity>1 and m.fabs(self.Gz)<0.2):
			error = self.GPS_sAcc*10/self.GPS_Velocity
			gain = (self.head_error/(self.head_error+error))
			self.declination -= heading_error*gain
			self.head_error *= (1-gain)
			# self.min_gps_speed += 0.1*self.dt
			# if(self.min_gps_speed>3):
			# 	self.min_gps_speed = 3


	def Fuse_odom(self): #wheelspeed callback
		gain = self.VelError/(self.VelError + self.odom_Velocity_error)
		self.Velocity = gain*self.odom_Velocity + (1-gain)*self.Velocity
		self.VelError *= (1-gain)

	def Fuse_gps_velocity(self):
		gain = self.VelError/(self.VelError + self.GPS_sAcc)
		self.Velocity = gain*self.GPS_Velocity + (1-gain)*self.Velocity
		self.VelError *= (1-gain)
		self.estimate_declination()

	def callback_imu(self,data):
		self.Ax = data.linear_acceleration.x
		self.Ay = data.linear_acceleration.y
		self.Az = data.linear_acceleration.z
		self.Gx = data.angular_velocity.x
		self.Gy = data.angular_velocity.y
		self.Gz = data.angular_velocity.z

		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.pitch, self.roll, self.heading) = euler_from_quaternion(orientation_list) #TODO : confirm this
		# Exploting non-holonomic constraints
		self.heading += self.declination
		if(m.fabs(self.Gz)>10*self.DEG2RADIAN):
			Gz2 = self.Gz**2
			radius = -float(self.Ay/Gz2)
			if(radius!=0):#CHANGED
				self.Ax += (self.Ay*(1/radius))
			V_mes = -self.Ay/self.Gz
			mes_error = max(m.fabs(self.Ax),1.0)*self.CIRCULAR_VELOCITY_ERROR/(min(m.fabs(Gz2),1.0))
			self.Velocity += self.Ax*self.dt
			self.VelError += self.dt*self.ACCEL_VARIANCE*m.cos(self.pitch)

			gain = self.VelError/(mes_error + self.VelError)
			self.Velocity = (1-gain)*self.Velocity + gain*V_mes
			self.VelError *= (1-gain)
		
		else:
			self.Velocity += self.Ax*self.dt
			self.VelError += self.dt*self.ACCEL_VARIANCE*m.cos(self.pitch)

		if(self.odom_tick):
			self.Fuse_odom()
			self.odom_tick = False

		if(self.tick_velocity):
			self.Fuse_gps_velocity()
			self.tick_velocity = False

		cosmh = m.cos(self.heading) #CHANGED
		sinmh = m.sin(self.heading) #CHANGED

		dS_x = self.Velocity*self.dt
		dSError = self.VelError*self.dt
		dTheta = self.head_error
		self.X += dS_x*cosmh
		self.Y += dS_x*sinmh
		self.PosError_X += m.fabs(dSError*cosmh - dS_x*sinmh*dTheta)
		self.PosError_Y += m.fabs(dSError*sinmh + dS_x*cosmh*dTheta)

		if(self.tick_gps and not self.position_reset):
			self.Fuse_gps_position()
			self.tick_gps = False

		if(self.position_reset):
			self.lastLat = self.gps_latitude
			self.lastLon = self.gps_longitude 
			self.gps_X = self.X 
			self.gps_Y = self.Y 
			self.iLat = self.gps_latitude - (self.Y*self.METER2DEG) #lat lon reported by gps matches with the past value of position.
			self.iLon = self.gps_longitude - (self.X*self.METER2DEG) 
			self.position_reset = False #prevent this code block from being re-executed

		self.longitude = self.iLon + self.X*self.METER2DEG #these are filtered lat lon coordinates
		self.latitude = self.iLat + self.Y*self.METER2DEG

		self.Publish_data()

	def Publish_data(self):
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "map" #?
		odom_quat = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.heading)
		odom.pose.pose = Pose(Point(self.X, self.Y, 0.), Quaternion(*odom_quat))
		odom.child_frame_id = "odom"
		vx = self.Velocity*m.cos(self.heading)
		vy = self.Velocity*m.sin(self.heading)
		odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(self.Gx, self.Gy, self.Gz))
		odom.pose.covariance = np.zeros(36)
		odom.twist.covariance = np.zeros(36)
		odom.pose.covariance[0] = self.PosError_X
		odom.pose.covariance[7] = self.PosError_Y
		odom.twist.covariance[0] = self.VelError*m.cos(self.heading)
		odom.twist.covariance[7] = self.VelError*m.sin(self.heading)
		self.odom_pub.publish(odom)
		## broadcast transform!
		self.odom_broadcaster.sendTransform((self.X, self.Y, 0),odom_quat,rospy.Time.now(),"base_link","odom")
		self.odom_broadcaster.sendTransform((0, 0, 0),(0,0,0,1),rospy.Time.now(),"odom","map")



if __name__ == '__main__':
	se = state()
	se._init_(100)
	rospy.init_node('KF', anonymous=True)
	rospy.Subscriber("gps/pixhawk", NavSatFix, se.gps_fix_callback)#First parameter is the name of the topic
	rospy.Subscriber("twist/pixhawk",TwistWithCovarianceStamped, se.gps_fix_velocity_callback)
	rospy.Subscriber("imu/pixhawk",Imu,se.callback_imu) #CHANGED
	rospy.Subscriber("e2ostatus",e2o_status,se.odom_velocity_callback)
	# rospy.Subscriber("imu/mag",MagneticField,se.mag_callback)
	rospy.spin()