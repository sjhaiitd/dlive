#!/usr/bin/env python
import rospy

from PCANBasic import *
import time                    ## Time-related library
import threading               ## Threading-based Timer library
from pcan.msg import Twist1
import numpy as np
from math import ceil
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pickle
rise_time=-100
settling_time=-100
overshoot=0
ss_error=-100
e_prior=0
es=0
first=True
bprev=0
bpprev=0
count=0
firstst=True
firstocl=True
firstss=True
cnt=0
now=0
set_po=[]
car_sp=[]
prev_time=0.0
first_time=0.0
control=False

m_CanRead = True
m_IsFD = False
f=open("logfile.save","wb")
m_PcanHandle = PCAN_USBBUS1

#initialize e2OCAN messages
e2OCAN_RNDB = TPCANMsg()
e2OCAN_LAMP = TPCANMsg()
e2OCAN_STEER = TPCANMsg()
e2OCAN_BRAKE = TPCANMsg()
e2OCAN_ACCEL = TPCANMsg()
e2OCAN_WIPHORN = TPCANMsg()

for i in range(8):
	e2OCAN_RNDB.DATA[i] = 0x00
	e2OCAN_LAMP.DATA[i] = 0x00
	e2OCAN_STEER.DATA[i] = 0x00
	e2OCAN_BRAKE.DATA[i] = 0x00
	e2OCAN_ACCEL.DATA[i] = 0x00
	e2OCAN_WIPHORN.DATA[i] = 0x00

e2OCAN_RNDB.ID = 0x778
e2OCAN_RNDB.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_RNDB.LEN = 8

e2OCAN_LAMP.ID = 0x776
e2OCAN_LAMP.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_LAMP.LEN = 8

e2OCAN_STEER.ID = 0x774
e2OCAN_STEER.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_STEER.LEN = 8

e2OCAN_BRAKE.ID = 0x772
e2OCAN_BRAKE.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_BRAKE.LEN = 8

e2OCAN_ACCEL.ID = 0x770
e2OCAN_ACCEL.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_ACCEL.LEN = 8

e2OCAN_WIPHORN.ID = 0x76D
e2OCAN_WIPHORN.MSGTYPE = PCAN_MESSAGE_STANDARD
e2OCAN_WIPHORN.LEN = 8

RNDB_NEUTRAL = 0x14
RNDB_REVERSE = 0x0C
RNDB_DRIVE = 0x24
RNDB_BOOST = 0x84
max_steer_angle_radian=5
min_steer_angle_radian=-5
LAMP_OFF = 0x09
LAMP_RIND = 0x19
LAMP_LIND = 0x89
LAMP_HAZARD = 0x99
LAMP_LBEAM = 0x0B
LAMP_HBEAM = 0x0D
LAMP_BKLIGHT_B7 = 0x09
LAMP_BKLIGHT_B6_ON = 0xE0
LAMP_BKLIGHT_B6_OFF = 0x20

STEER_ZERO = 0x01
STEER_LEFT = 0xA2
STEER_RIGHT = 0xA1

BRAKE_ZERO = 0x01
BRAKE_FULL = 0xC9

ACCEL_ZERO = 0x01
ACCEL_FULL = 0xC9

WIPER_OFF = 0x20
WIPER_INT = 0x60
WIPER_HIGH = 0xA0
HORN_ON = 0x18
HORN_OFF = 0x08

m_objPCANBasic = PCANBasic()
print("PCAN_CHANNEL_CONDITION (Before) = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))

Status = m_objPCANBasic.Initialize(m_PcanHandle, PCAN_BAUD_500K, 0, 0, 0)
print(Status)
i=0
print "i :", i
print("PCAN_DEVICE_NUMBER = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_DEVICE_NUMBER)))
print("PCAN_API_VERSION= " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_API_VERSION)))
print("PCAN_HARDWARE_NAME = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_HARDWARE_NAME)))
print("PCAN_CHANNEL_CONDITION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
print("PCAN_CHANNEL_VERSION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_VERSION)))
print("PCAN_BITRATE_INFO = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BITRATE_INFO)))
print("PCAN_BUSSPEED_NOMINAL = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BUSSPEED_NOMINAL)))
print("PCAN_RECEIVE_STATUS = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_RECEIVE_STATUS)))


e2OCAN_LAMP.DATA[6] = 0x20
e2OCAN_STEER.DATA[6] = 0xC0

e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
e2OCAN_LAMP.DATA[7] = LAMP_HAZARD
e2OCAN_STEER.DATA[7] = STEER_ZERO
e2OCAN_BRAKE.DATA[7] = BRAKE_ZERO
e2OCAN_ACCEL.DATA[7] = ACCEL_ZERO
e2OCAN_WIPHORN.DATA[7] = WIPER_OFF | HORN_OFF


def ReadMessages():
        global carspeed1
        global setpoint1
	global steer_feedback
	global count_odom 
	global count_cmd
	
	try:
		stsResult = PCAN_ERROR_OK
		while (not (stsResult & PCAN_ERROR_QRCVEMPTY)):
			result = m_objPCANBasic.Read(m_PcanHandle)
			stsResult = result[0]
			if result[0] == PCAN_ERROR_OK:
				newMsg = result[1]
				msgTimeStamp = result[2]
				#msgTimeStamp.value = (msgTimeStamp.micros + 1000 * msgTimeStamp.millis + 0x100000000 * 1000 * msgTimeStamp.millis_overflow)

				#print(format(newMsg.ID,'04x'))
				MsgID = format(newMsg.ID,'04x')
				DATA7 = format(newMsg.DATA[7],'02x')
				DATA6 = format(newMsg.DATA[6],'02x')
				if (MsgID == '076c'):
					count_odom=count_odom+1
                                        count_cmd=count_cmd+1
                            
					#print "Vehicle speed status"
					#print "\nSpeed of Car : ", int(DATA7,16)
				        carspeed1 = int(DATA7,16)
                                        mode_select()

				        setpoint= Float64(setpoint1)                                    
                                        carspeed=Float64(carspeed1)
                          
                                        pub2.publish(setpoint)						
					pub1.publish(carspeed)
                                  
					#Logger= [carspeed , setpoint]
					
					#f=open("test.txt","a")
                                         
					#pickle.dump(Logger,f)
					
						
				#elif (MsgID == '076e'):
					#print "\n\nWiper & Horn Status"
					#if(DATA7 == 'a0'):
 	                                 #print "\nWiper : ON \nHorn : ON"
					#elif(DATA7 == '20'):
						#print "\nWiper : OFF \nHorn : ON"
					#elif(DATA7 == '80'):
						#print "\nWiper : ON \nHorn : OFF"
					#elif(DATA7 == '00'):
						#print "\nWiper : OFF \nHorn : OFF"
				#elif (MsgID == '0771'):
					#print "\n\nThrottle Status"
					#Throttle = DATA7
					#print "\nThrottel Percentage of Car : ", int(Throttle,16)/2
				#elif (MsgID == '0773'):
					#print "Braking Status"
					#Brake = DATA7
					#print "\nBraking Percentage of Car : ", int(Brake,16)/2
				elif (MsgID == '0775'):
					#print "\n\nSteering angle & Steering Direction Status"
					Steer = DATA7
					Steer_bin = int(Steer,16)
                                        Steer_angle = (Steer_bin>>2)
					#print Steer_bin
                                       
					if((Steer_bin&3)==2):
                                                Steer_angle=Steer_angle*1
						#print "\nSteering Direction : Clockwise"
					elif((Steer_bin&3)==1):
                                                Steer_angle=Steer_angle*(-1)
						#print "\nSteering Direction : AntiClockwise"
                                        #print Steer_angle
					steer_feedback=Steer_angle
					#print "\n Steering Angle : ",Steer_angle
					#Steering Angle yet to be done

				#elif (MsgID == '0777'):
					#print "\n\nTurn Indicators & HL Beam"
					#if(DATA7[0] == '8'):
						#print "\nIndicator Status : Left Indicator"
					#elif(DATA7[0] == '1'):
						#print "\nIndicator Status : Right Indicator"
					#elif(DATA7[0] == '9'):
						#print "\nIndicator Status : Hazard Indicator"
					#elif(DATA7[0] == '0'):
						#print "\nIndicator Status : OFF"

					#if(DATA7[1] == '4'):
						#print "\nHead Lights : ON"
					#elif(DATA7[1] == '0'):
						#print "\nHead Lights : OFF"
				elif (MsgID == '0779'):
					#print "\n\nDriving Mode & PRNDL Status"

					#if(DATA7 == '40'):
						#print "\nDriving Mode : Manual"
					#elif(DATA7 == '80'):
						#print "\nDriving Mode : Autonomous"
					#elif(DATA7 == 'c0'):
						#print "\nDriving Mode : Homing in Progress"
					#else:
						#print "\nDriving Mode : Invalid"

					if(DATA6 == '08'):
                                                drive_mode=2
						#print "\nGear Status : Reverse"
					elif(DATA6 == '10'):
                                                drive_mode=0
						#print "\nGear Status : Neutral"
					elif(DATA6 == '20'):
                                                drive_mode=1
                                  
						#print "\nGear Status : Drive"
					#elif(DATA6 == '80'):
						#print "\nGear Status : Boost"
					#else:
						#print "\nGear Status : Invalid"


				#print(msgTimeStamp.value)

				#for i in range(newMsg.LEN):
				#    print(format(newMsg.DATA[i],'02x'))
	except KeyboardInterrupt:
		tmrRead.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

def WriteMessages():
	try:
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_RNDB)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_LAMP)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_STEER)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_BRAKE)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_ACCEL)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_WIPHORN)
	except KeyboardInterrupt:
		tmrWrite.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

###*****************************************************************
### Timer class
###*****************************************************************
class TimerRepeater(object):
	"""
	A simple timer implementation that repeats itself
	"""
	# Constructor
	def __init__(self, name, interval, target, isUi, args=[], kwargs={}):
		"""
		Creates a timer.
		Parameters:
			name        name of the thread
			interval    interval in second between execution of target
			target      function that is called every 'interval' seconds
			args        non keyword-argument list for target function
			kwargs      keyword-argument list for target function
		"""
		# define thread and stopping thread event
		self._name = name
		self._thread = None
		self._event = None
		self._isUi = isUi
		# initialize target and its arguments
		self._target = target
		self._args = args
		self._kwargs = kwargs
		# initialize timer
		self._interval = interval
		self._bStarted = False

	# Runs the thread that emulates the timer
	def _run(self):
		"""
		Runs the thread that emulates the timer.
		Returns:
			None
		"""
		while not self._event.wait(self._interval):
			if self._isUi:
				# launch target in the context of the main loop
				root.after(1, self._target,*self._args, **self._kwargs)
			else:
				self._target(*self._args, **self._kwargs)

	# Starts the timer
	def start(self):
		# avoid multiple start calls
		if (self._thread == None):
			self._event = threading.Event()
			self._thread = threading.Thread(None, self._run, self._name)
			self._thread.start()

	# Stops the timer
	def stop(self):
		if (self._thread != None):
			self._event.set()
			self._thread = None


def callback(msg):
        global gsteer
        global vx
        global prev_vx
	final = int(round(msg.data)) 
	print "global",final
        if neutral==False:
           if final>50:
              final=50
	   if final>= 0:
              print "positive:",final
	      e2OCAN_ACCEL.DATA[7]= (final<<1) | 1
	      e2OCAN_BRAKE.DATA[7]= BRAKE_ZERO
		#print final.data
	   else:
              print "negative:",final
              if(prev_vx*vx>=0 and prev_vx-vx>2):
                 e2OCAN_ACCEL.DATA[7]= ACCEL_ZERO
                 e2OCAN_BRAKE.DATA[7]= ((abs(final/2))<<1) | 1
              else:
                 e2OCAN_ACCEL.DATA[7]= ACCEL_ZERO
                 e2OCAN_BRAKE.DATA[7]= BRAKE_ZERO
		# 
        #print "st:",gsteer
        print gsteer
        if gsteer>=0:
           e2OCAN_STEER.DATA[7] = ((abs(gsteer)<<2)| 2)
        else:
           e2OCAN_STEER.DATA[7] = ((abs(gsteer)<<2)| 1)
def cmd_vel_callback(vel):
        global setpoint1
        global gsteer
        global prev_vx
        global vx
        global cml
        global count_cmd
        global min_steer_angle_radian
        global max_steer_angle_radian
        print "setpoint",setpoint1
        prev_vx=vx
        count_cmd=0
        cml=True
        vx=int(round(vel.linear.x*18/5))
        print "vx",vx
        if abs(vel.angular.z)<=0.001:
           steer_d=0
        else:
           radius = vel.linear.x / vel.angular.z
           steer_d=math.atan(1.958 / radius)
           steer_d=steer_d*180/np.pi
        #print "steer_d",steer_d
        steer=max(min_steer_angle_radian,min(max_steer_angle_radian,steer_d))
       
        gsteer=steer
        gsteer=int(round(gsteer))
        #print "str:",gsteer
        setpoint1=abs(vx) 
        #print "set",setpoint1
        
def mode_select():
        global setpoint1
        global gsteer
        global prev_vx
        global change
        global fr
        global vx
	global cml
        #print "current",vx
        #print "prev",prev_vx
        #print cml
        #print vx*prev_vx
        if cml==True:
           if vx*prev_vx<0 and change==False:
              if count_cmd<15:
                 setpoin1=0
              else:
                 e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
                 neutral=True
                 change=True
           elif change==True:
              setpoint1=0
              change=False
              fr=True
              cml=False
              if vx>=0 :
                  e2OCAN_RNDB.DATA[6] = RNDB_DRIVE
              else:
                  e2OCAN_RNDB.DATA[6] = RNDB_REVERSE
        else:
           setpoint1=abs(vx)


def odom_send_callback(data):
        global carspeed1
        global steer_feedback
        global count_odom
        global vx
        count_odom=0
        if drive_mode==1:
           carspeed1=carspeed1*1
        elif drive_mode==2:
           carspeed1=carspeed1*(-1)
        data.twist.twist.linear.x=carspeed1
        data.twist.twist.angular.z=((vx*5/18)/1.958)*math.tan((steer_feedback*np.pi/180))
	#print "data _twist",data.twist.twist.linear.x
	#print "data _angular",data.twist.twist.angular.z
        odom_full.publish(data)
setpoint1=0
steer_feedback=0
gsteer=0
fr=True
carspeed1=0
prev_vx=0
vx=0
drive_mode=1
cml=False
change=False
e2OCAN_RNDB.DATA[6] = RNDB_DRIVE
e2OCAN_ACCEL.DATA[7] =ACCEL_ZERO
e2OCAN_BRAKE.DATA[7] = BRAKE_ZERO
neutral=False
angle=0
acc=0
deacc=0
prev=0
loop_count=0
quit_count=0
rospy.init_node('e2oMove')
sub = rospy.Subscriber('/control_effort', Float64, callback)
#sub1 = rospy.Subscriber('can_msg', Twist1, callback1)
velocity=rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
odom_without_speed=rospy.Subscriber('/odom1',Odometry,odom_send_callback)
odom_full=rospy.Publisher('/odom',Odometry,queue_size=1)
pub1 = rospy.Publisher('/state',Float64, queue_size=1)
pub2 = rospy.Publisher('/setpoint' , Float64,queue_size=1)
count_odom=0
count_cmd=0
tmrRead = TimerRepeater("tmrRead", 0.010, ReadMessages, False)
tmrWrite = TimerRepeater("tmrWrite", 0.010, WriteMessages, False)
tmrWrite.start()
tmrRead.start()
rospy.spin()
#o1yPjTaQ
