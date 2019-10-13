#!/usr/bin/env python
import time
import sys, os
import numpy as np
#source : https://gist.github.com/vo/9331349
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import NavSatFix,Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymavlink import mavutil
import math as m 
# import traceback

imu_quat = None
A = np.zeros(3)
G = np.zeros(3)
imu_pub = rospy.Publisher("imu/pixhawk", Imu, queue_size=10)
twist_pub = rospy.Publisher("twist/pixhawk", TwistWithCovarianceStamped, queue_size=10)
navsat_pub = rospy.Publisher("gps/pixhawk",NavSatFix,queue_size=10)
speed_variance = 1000
position_variance = 1000

def handle_heartbeat(msg):
	mode = mavutil.mode_string_v10(msg)
	is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
	is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
	channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
			msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
	hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
	# print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
	# print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

def handle_attitude(msg):
	global imu_quat
	global G
	global A
	global imu_pub
	attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
	yaw = -msg.yaw
	yaw += m.pi/2
	if(yaw>m.pi):
		yaw -= 2*m.pi
	if(yaw< -m.pi):
		yaw += 2*m.pi

	G = np.array([msg.rollspeed,msg.pitchspeed,-msg.yawspeed])
	A[0] -= 9.89*m.sin(msg.pitch)
	if(m.fabs(A[0])<0.1):
		A[0] = 0
	A[0] = round(A[0],2)
	A[1] += 9.89*m.sin(msg.roll)
	if(m.fabs(A[1])<0.1):
		A[1] = 0
	A[1] = round(A[1],2)

	imu = Imu()
	imu.header.stamp = rospy.Time.now()
	imu.header.frame_id = "base_link"
	imu_quat = quaternion_from_euler(msg.roll, msg.pitch, yaw)
	imu.orientation = Quaternion(*imu_quat)
	imu.linear_acceleration.x = -A[0]
	imu.linear_acceleration.y = -A[1]
	imu.linear_acceleration.z = -A[2]
	imu.angular_velocity.x = G[0]
	imu.angular_velocity.y = G[1]
	imu.angular_velocity.z = G[2]
	imu_pub.publish(imu)
	return


def handle_acc(msg):
	global A
	A = np.array([float(msg.xacc)*0.01,float(msg.yacc)*0.01,float(msg.zacc)*0.01])

def handle_gps(msg):
	global speed_variance
	global position_variance
	global twist_pub
	TWCS = TwistWithCovarianceStamped()
	TWCS.header.stamp = rospy.Time.now()
	TWCS.header.frame_id = "odom"
	TWCS.twist.twist.linear.x = msg.vy*0.01
	TWCS.twist.twist.linear.y = msg.vx*0.01
	TWCS.twist.covariance[0] = speed_variance
	twist_pub.publish(TWCS)

	NSF = NavSatFix()
	NSF.header.stamp = rospy.Time.now()
	NSF.header.frame_id = "odom"
	NSF.latitude = msg.lat*1e-7
	NSF.longitude = msg.lon*1e-7
	NSF.position_covariance[0] = position_variance
	navsat_pub.publish(NSF)

def handle_report(msg):
	global position_variance
	global speed_variance
	speed_variance = msg.velocity_variance
	position_variance = msg.pos_horiz_variance

def read_loop(m):
	now = time.time()

	while not rospy.is_shutdown():
		# grab a mavlink message
		msg = m.recv_match(blocking=False)
		try:
			# handle the message based on its type
			msg_type = msg.get_type()
			# print(msg_type)
			if msg_type == "BAD_DATA":
				if mavutil.all_printable(msg.data):
					sys.stdout.write(msg.data)
					sys.stdout.flush()
			elif msg_type == "RC_CHANNELS_RAW": 
				handle_rc_raw(msg)
			elif msg_type == "HEARTBEAT":
				handle_heartbeat(msg)
			elif msg_type == "ATTITUDE":
				handle_attitude(msg)
			elif msg_type == "GLOBAL_POSITION_INT":
				handle_gps(msg)
			elif msg_type == "EKF_STATUS_REPORT":
				handle_report(msg)
			elif msg_type == 'RAW_IMU':
				handle_acc(msg)

		except KeyboardInterrupt:
			break
		# except Exception as e:
		# 	print(traceback.format_exc())
		except:
			pass
		
def main():
	rate = 50
	device = '/dev/ttyACM0'
	baudrate = 115200


	# create a mavlink serial instance
	try:
		master = mavutil.mavlink_connection(device, baud=baudrate)
	except:
		master = mavutil.mavlink_connection('/dev/ttyACM1', baud=baudrate)


	# wait for the heartbeat msg to find the system ID
	master.wait_heartbeat()

	# request data to be sent at the given rate
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
	# enter the data loop
	read_loop(master)


if __name__ == '__main__':
	rospy.init_node('pixhawk')
	main()