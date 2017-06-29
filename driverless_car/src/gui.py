# -*- coding: utf-8 -*-
"""
Created on Sat Jun 24 21:32:16 2017

@author: rohit
"""

import sys
from PyQt4.QtCore import* 
from PyQt4.QtGui import* 
from PyQt4 import QtGui
import rospy
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import actionlib
import numpy as np
rospy.init_node('gui', anonymous=True)
listener = tf.TransformListener()
from math import*
import math
init_first=False
fake=[]
all_val=[]
jk=False
#try:     
    #now = rospy.Time.now()
    #listener.waitForTransform("/map", "/odom", now, rospy.Duration(1))
    #[trans,rot] = listener.lookupTransform("/map", "/odom", now)
#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #print "a"
trans=[250,250,0]
rot=[0,0,0,1]
af=tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
theta=af[2]
transform=np.zeros((4,4))
transform[3,3]=1
transform[2,2]=1
transform[0,3]=trans[0]
transform[1,3]=trans[1]
transform[0,0]=cos(theta)
transform[0,1]=-sin(theta)
transform[1,0]=sin(theta)
transform[1,1]=cos(theta)
decay_count=0
goal_x=0
goal_y=0
goal_set_flag=False
def callbackacc(data,args):
    args[0].setText(str(int(data.data)))
def callbacksetp(data,args):
    args[0].setText(str(int(data.data)))
def callbackcmd(data,args):
    args[0].setText(str(round(data.linear.x*18/5,0)))
    if data.angular.z<0.001:
        args[1].setText(str(0))
    else:
        args[1].setText(str(round(math.atan((1.958/data.linear.x)*data.angular.z)*180/np.pi,0)))
def callbackgoalstatus(data,args):
    global init_first
    if len(data.status_list)==1:
        if data.status_list[0].status==1 :
            args[0].setText("PENDING")
        elif data.status_list[0].status==3:
            args[0].setText("SUCCEEDED")
        elif data.status_list[0].status==4:
            args[0].setText("ABORTED")
        elif data.status_list[0].status==5:
            args[0].setText("REJECTED")
    elif len(data.status_list)>1:
        if data.status_list[1].status==1 :
            args[0].setText("PENDING") 
def callbackgoal(data,args):
    args[0].setText(str(round(data.goal.target_pose.pose.position.x,0)))
    args[1].setText(str(round(data.goal.target_pose.pose.position.y,0)))
    a=tf.transformations.euler_from_quaternion([data.goal.target_pose.pose.orientation.x,data.goal.target_pose.pose.orientation.y,data.goal.target_pose.pose.orientation.z,data.goal.target_pose.pose.orientation.w])
    fin=round((a[2]-theta)*180/np.pi,0)
    args[2].setText(str(fin))
def callbackodom(data,args):
    global listener,init_first,transform,decay_count,goalx,goaly,goal_set_flag
    decay_count+=1
    if decay_count>10:
        f=np.dot(transform,np.asarray([round(data.pose.pose.position.x,2),round(data.pose.pose.position.y,2),0,1]))
        args[0].setText(str(round(f[0],0)))
        args[1].setText(str(round(f[1],0)))
        a=tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        fin=round((a[2]-theta)*180/np.pi,0)
        args[2].setText(str(fin))
        args[6].setText(str(round(data.twist.twist.linear.x*18/5,0)))
        if data.twist.twist.angular.z<0.001:
            args[7].setText(str(0))
        else:
            args[7].setText(str(round(math.atan((1.958/data.twist.twist.linear.x)*data.twist.twist.angular.z)*180/np.pi,0)))
    
        if(init_first==False):
            init_first=True
            args[3].setText(str(round(f[0],0)))
            args[4].setText(str(round(f[1],0)))
            args[5].setText(str(fin))
        if goal_set_flag==True:
            args[11].setText(str(round(sqrt(((f[0]-goalx)**2)+((f[1]-goaly)**2)),0)))
        decay_count=0
def window():
    global all_val
    
    app = QtGui.QApplication(sys.argv)
    win = QtGui.QWidget()
    grid=QtGui.QGridLayout()
    g_odom=QtGui.QLabel()
    g_odom.setText("Odometry Data:")
    g_carspeed=QtGui.QLabel()
    g_carspeed.setText("Speed of Car:")
    g_setpoint=QtGui.QLabel()
    g_setpoint.setText("Desired Value:")
    g_controleffort=QtGui.QLabel()
    g_controleffort.setText("Acceleration:")
    g_initialp=QtGui.QLabel()
    g_initialp.setText("Initial Position:")
    g_cmdvel=QtGui.QLabel()
    g_cmdvel.setText("Velocity Command:")
    g_kp=QtGui.QLabel()
    g_kp.setText("kp:")
    g_ki=QtGui.QLabel()
    g_ki.setText("ki:")
    g_kd=QtGui.QLabel()
    g_kd.setText("kd:")
    g_goalstatus=QtGui.QLabel()
    g_goalstatus.setText("Goal Status:")
    g_logo=QtGui.QLabel()
    g_logo.setPixmap(QPixmap("dlive.png"))
    g_goalpos=QtGui.QLabel()
    g_goalpos.setText("Goal Position:")
    g_distfromgoal=QtGui.QLabel()
    g_distfromgoal.setText("Distance from Goal:")
    # label wala cheez hua avi tk
    #individual design av
    #initial position
    g_ipx=QtGui.QLabel()
    g_ipy=QtGui.QLabel()
    g_ipth=QtGui.QLabel()
    g_ipx.setText("x:")
    g_ipy.setText("y:")
    g_ipth.setText("yaw:")
    g_ipx_val=QtGui.QLineEdit()
    g_ipy_val=QtGui.QLineEdit()
    g_ipth_val=QtGui.QLineEdit()
    g_ipx_val.setReadOnly(True)
    g_ipy_val.setReadOnly(True)
    g_ipth_val.setReadOnly(True)
    hbox_ip=QtGui.QHBoxLayout()
    hbox_ip.addWidget(g_ipx)
    hbox_ip.addStretch()
    hbox_ip.addWidget(g_ipx_val)
    hbox_ip.addStretch()
    hbox_ip.addWidget(g_ipy)
    hbox_ip.addStretch()
    hbox_ip.addWidget(g_ipy_val)
    hbox_ip.addStretch()
    hbox_ip.addWidget(g_ipth)
    hbox_ip.addStretch()
    hbox_ip.addWidget(g_ipth_val)
    vbox_ip=QtGui.QVBoxLayout()
    vbox_ip.addWidget(g_logo)
    vbox_ip.addStretch()
    vbox_ip.addWidget(g_initialp)
    vbox_ip.addStretch()
    vbox_ip.addLayout(hbox_ip)
    
    ##############################
    #odom_data
    g_odomx=QtGui.QLabel()
    g_odomy=QtGui.QLabel()
    g_odomth=QtGui.QLabel()
    g_odomx.setText("x:")
    g_odomy.setText("y:")
    g_odomth.setText("yaw:")
    g_odomx_val=QtGui.QLineEdit()
    g_odomy_val=QtGui.QLineEdit()
    g_odomth_val=QtGui.QLineEdit()
    g_odomx_val.setReadOnly(True)
    g_odomx_val.setValidator(QDoubleValidator())
    g_odomy_val.setReadOnly(True)
    g_odomth_val.setReadOnly(True)
    g_odomth_val.setMaxLength(8)
    hbox_odom=QtGui.QHBoxLayout()
    hbox_odom.addWidget(g_odomx)
    hbox_odom.addStretch()
    hbox_odom.addWidget(g_odomx_val)
    hbox_odom.addStretch()
    hbox_odom.addWidget(g_odomy)
    hbox_odom.addStretch()
    hbox_odom.addWidget(g_odomy_val)
    hbox_odom.addStretch()
    hbox_odom.addWidget(g_odomth)
    hbox_odom.addStretch()
    hbox_odom.addWidget(g_odomth_val)
    vbox_odom=QtGui.QVBoxLayout()
    vbox_odom.addWidget(g_odom)
    vbox_odom.addStretch()
    vbox_odom.addLayout(hbox_odom)
    #############################
    vbox_glob=QtGui.QVBoxLayout()
    vbox_glob.addLayout(vbox_ip)
    vbox_glob.addStretch()
    vbox_glob.addLayout(vbox_odom)
    ####################################
    g_carspeed_val=QtGui.QLineEdit()
    g_steera=QtGui.QLabel("Steering Angle:")
    g_steera_val=QtGui.QLineEdit()
    g_steera_val.setReadOnly(True)
    g_carspeed_val.setReadOnly(True)
    hbox_carspeed=QtGui.QHBoxLayout()
    hbox_carspeed.addWidget(g_carspeed)
    hbox_carspeed.addStretch()
    hbox_carspeed.addWidget(g_carspeed_val)
    hbox_carspeed.addStretch()
    hbox_carspeed.addWidget(g_steera)
    hbox_carspeed.addStretch()
    hbox_carspeed.addWidget(g_steera_val)
    vbox_glob.addStretch()
    vbox_glob.addLayout(hbox_carspeed)
    #########################################
    g_setpoint_val=QtGui.QLineEdit()
    g_setpoint_val.setReadOnly(True)
    hbox_setpoint=QtGui.QHBoxLayout()
    hbox_setpoint.addWidget(g_setpoint)
    hbox_setpoint.addStretch()
    hbox_setpoint.addWidget(g_setpoint_val)
    vbox_glob.addStretch()
    vbox_glob.addLayout(hbox_setpoint)
    ###########################################
    g_controleffort_val=QtGui.QLineEdit()
    g_controleffort_val.setReadOnly(True)
    hbox_controleffort=QtGui.QHBoxLayout()
    hbox_controleffort.addWidget(g_controleffort)
    hbox_controleffort.addStretch()
    hbox_controleffort.addWidget(g_controleffort_val)
    vbox_glob.addStretch()
    vbox_glob.addLayout(hbox_controleffort)
    #############################################
    g_cmdx=QtGui.QLabel("Speed Input:")
    g_cmdsteer=QtGui.QLabel("Steering Angle:")
    g_cmdx_val=QtGui.QLineEdit()
    g_cmdsteer_val=QtGui.QLineEdit()
    g_cmdx_val.setReadOnly(True)
    g_cmdsteer_val.setReadOnly(True)
    hbox_cmd=QtGui.QHBoxLayout()
    hbox_cmd.addWidget(g_cmdx)
    hbox_cmd.addStretch()
    hbox_cmd.addWidget(g_cmdx_val)
    hbox_cmd.addStretch()
    hbox_cmd.addWidget(g_cmdsteer)
    hbox_cmd.addStretch()
    hbox_cmd.addWidget(g_cmdsteer_val)
    vbox_cmd=QtGui.QVBoxLayout()
    vbox_cmd.addWidget(g_cmdvel)
    vbox_cmd.addStretch()
    vbox_cmd.addLayout(hbox_cmd)
    vbox_glob1=QtGui.QVBoxLayout()
    vbox_glob1.addLayout(vbox_cmd)
    ######################################kp ki kd
    g_kp_val=QtGui.QLineEdit()
    g_ki_val=QtGui.QLineEdit()
    g_kd_val=QtGui.QLineEdit()
    g_kp_val.setReadOnly(True)
    g_ki_val.setReadOnly(True)
    g_kd_val.setReadOnly(True)
    hbox_pid=QtGui.QHBoxLayout()
    hbox_pid.addWidget(g_kp)
    hbox_pid.addStretch()
    hbox_pid.addWidget(g_kp_val)
    hbox_pid.addStretch()
    hbox_pid.addWidget(g_ki)
    hbox_pid.addStretch()
    hbox_pid.addWidget(g_ki_val)
    hbox_pid.addStretch()
    hbox_pid.addWidget(g_kd)
    hbox_pid.addStretch()
    hbox_pid.addWidget(g_kd_val)
    vbox_glob1.addStretch()
    vbox_glob1.addLayout(hbox_pid)
    #####################################goal pos
    g_goalx=QtGui.QLabel()
    g_goaly=QtGui.QLabel()
    g_goalth=QtGui.QLabel()
    g_goalx.setText("x:")
    g_goaly.setText("y:")
    g_goalth.setText("yaw:")
    g_goalx_val=QtGui.QLineEdit()
    g_goaly_val=QtGui.QLineEdit()
    g_goalth_val=QtGui.QLineEdit()
    g_goalx_val.setReadOnly(True)
    g_goaly_val.setReadOnly(True)
    g_goalth_val.setReadOnly(True)
    hbox_goal=QtGui.QHBoxLayout()
    hbox_goal.addWidget(g_goalx)
    hbox_goal.addStretch()
    hbox_goal.addWidget(g_goalx_val)
    hbox_goal.addStretch()
    hbox_goal.addWidget(g_goaly)
    hbox_goal.addStretch()
    hbox_goal.addWidget(g_goaly_val)
    hbox_goal.addStretch()
    hbox_goal.addWidget(g_goalth)
    hbox_goal.addStretch()
    hbox_goal.addWidget(g_goalth_val)
    vbox_goal=QtGui.QVBoxLayout()
    vbox_goal.addWidget(g_goalpos)
    vbox_goal.addStretch()
    vbox_goal.addLayout(hbox_goal)
    vbox_glob1.addStretch()
    vbox_glob1.addLayout(vbox_goal)
    ####################################distgoal
    g_dist_val=QtGui.QLineEdit()
    g_dist_val.setReadOnly(True)
    g_goalstat_val=QtGui.QLineEdit()
    g_goalstat_val.setReadOnly(True)
    hbox_dist=QtGui.QHBoxLayout()
    hbox_dist.addWidget(g_distfromgoal)
    hbox_dist.addStretch()
    hbox_dist.addWidget(g_dist_val)
    vbox_glob1.addStretch()
    vbox_glob1.addLayout(hbox_dist)
    hbox_gs=QtGui.QHBoxLayout()
    hbox_gs.addWidget(g_goalstatus)
    hbox_gs.addStretch()
    hbox_gs.addWidget(g_goalstat_val)
    vbox_glob1.addStretch()
    vbox_glob1.addLayout(hbox_gs)
    ##################################################
    enter_goal=QtGui.QPushButton(win)
    enter_goal.setText("Enter Goal")
    enter_goal.clicked.connect(goal_clicked)
    vbox_glob1.addStretch()
    reset=QtGui.QPushButton(win)
    reset.setText("Reset")
    reset.clicked.connect(reset_clicked)
    
    vbox_glob1.addWidget(enter_goal)
    vbox_glob1.addStretch()
    vbox_glob1.addWidget(reset)
    grid.addLayout(vbox_glob,0,0)
    grid.addLayout(vbox_glob1,0,5)
    rospy.Subscriber("/odom",Odometry,callbackodom,[g_odomx_val,g_odomy_val,g_odomth_val,g_ipx_val,g_ipy_val,g_ipth_val,g_carspeed_val,g_steera_val,g_kp_val,g_ki_val,g_kd_val,g_dist_val])
    rospy.Subscriber("/move_base/goal",MoveBaseActionGoal,callbackgoal,[g_goalx_val,g_goaly_val,g_goalth_val])
    rospy.Subscriber("/move_base/status",GoalStatusArray,callbackgoalstatus,[g_goalstat_val])
    rospy.Subscriber("/cmd_vel",Twist,callbackcmd,[g_cmdx_val,g_cmdsteer_val])
    rospy.Subscriber("/setpoint",Float64,callbacksetp,[g_setpoint_val])
    rospy.Subscriber("/control_effort",Float64,callbackacc,[g_controleffort_val])
    #rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,callbackinitial,[g_ipx_val,g_ipy_val,g_ipth_val])
    #hbox_glob=QtGui.QHBoxLayout()
   
    #hbox_glob.addLayout(vbox_glob)
    #hbox_glob.addStretch()
    
    #hbox_glob.addLayout(vbox_glob1)
    #full_set=[g_odom,g_carspeed,g_setpoint,g_controleffort, g_initialp,g_cmdvel,g_kp,g_ki,g_kd,g_goalstatus, g_goalpos,g_distfromgoal,g_distfromgoal]
            #grid.addWidget(QtGui.QPushButton("B"+str(i)+str(j)),i,j)
    all_val=[g_ipx_val,g_ipy_val,g_ipth_val,g_odomx_val,g_odomy_val,g_odomth_val,g_carspeed_val,g_steera_val,g_setpoint_val,g_controleffort_val,g_cmdx_val,g_cmdsteer_val]
    win.setLayout(grid)
    win.setGeometry(200,100,540,500)
    win.setWindowTitle("D-Live")
    win.show()
   
    sys.exit( app.exec_())
    

def goal_clicked():
    global fake
    win1=QtGui.QDialog()
    g_goalsetx=QtGui.QLabel("x:")
    g_goalsety=QtGui.QLabel("y:")
    g_goalsetth=QtGui.QLabel("yaw:")
    g_goalsetx_val=QtGui.QLineEdit()
    g_goalsety_val=QtGui.QLineEdit()
    g_goalsetth_val=QtGui.QLineEdit()
    
    hbox_goalset=QtGui.QHBoxLayout()
    hbox_goalset.addWidget(g_goalsetx)
    hbox_goalset.addStretch()
    hbox_goalset.addWidget(g_goalsetx_val)
    hbox_goalset.addStretch()
    hbox_goalset.addWidget(g_goalsety)
    hbox_goalset.addStretch()
    hbox_goalset.addWidget(g_goalsety_val)
    hbox_goalset.addStretch()
    hbox_goalset.addWidget(g_goalsetth)
    hbox_goalset.addStretch()
    hbox_goalset.addWidget(g_goalsetth_val)
    fake=[]
    fake.append([g_goalsetx_val])
    fake.append([g_goalsety_val])
    fake.append([g_goalsetth_val])
    enter_g=QtGui.QPushButton(win1)
    enter_g.setText("OK")
    enter_g.clicked.connect(goal_pub)
    vbox_g=QtGui.QVBoxLayout()
    vbox_g.addLayout(hbox_goalset)
    vbox_g.addStretch()
    vbox_g.addWidget(enter_g)
    win1.setGeometry(200,100,200,200)
    win1.setLayout(vbox_g)
    win1.setWindowTitle("Enter Goal")
    win1.setWindowModality(Qt.ApplicationModal)
    fake.append([win1])
    win1.exec_()

def reset_clicked():
    global all_val,init_first
    init_first=False
    for i in range(len(all_val)):
        all_val[i].clear()
        
def goal_pub(args):
    global goal_set_flag,goalx,goaly
    a=fake[0][0].text()
    b=fake[1][0].text()
    c=fake[2][0].text()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp=rospy.get_rostime() 
    goal.target_pose.pose.position.x = float(a)
    goal.target_pose.pose.position.y = float(b)
    goalx=float(a)
    goaly=float(b)
    y=float(c)
    quat =tf.transformations.quaternion_from_euler(0, 0, y)
    goal.target_pose.pose.orientation.w =quat[3]
    goal.target_pose.pose.orientation.x =quat[0]
    goal.target_pose.pose.orientation.y =quat[1]
    goal.target_pose.pose.orientation.z =quat[2]
    # Fill in the goal here
    goal_set_flag=True
    client.send_goal(goal)
    fake[3][0].close()
if __name__ == '__main__':
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    window()
