#! /usr/bin/env python
"""
Created on Sun Jun 18 18:46:35 2017

@author: rohit
"""
import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from driverless_car.msg import more
from driverless_car.msg import random
def callback(data):
    n=len(data.msg)
    print "aya"
    for i in range(1,n):
       data.msg[i].yaw=math.atan2((data.msg[i].y-data.msg[i-1].y),(data.msg[i].x-data.msg[i-1].x))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp=rospy.get_rostime()   
    goal.target_pose.pose.position.x = data.msg[n-1].x
    goal.target_pose.pose.position.y = data.msg[n-1].y
    y=data.msg[n-1].yaw
    quat =tf.transformations.quaternion_from_euler(0, 0, y)
    goal.target_pose.pose.orientation.w =quat[3]
    goal.target_pose.pose.orientation.x =quat[0]
    goal.target_pose.pose.orientation.y =quat[1]
    goal.target_pose.pose.orientation.z =quat[2]
    # Fill in the goal here
    client.send_goal(goal)
    print "gaya"
    while(1):
      pub.publish(data)
    
    a = client.wait_for_result()
    #rospy.Duration.from_sec(5.0)
    print a

rospy.init_node('waypoint_executer')
rospy.Subscriber('way_msg', more, callback)
pub=rospy.Publisher('contact',more,queue_size=10)
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()
rospy.spin()
    
