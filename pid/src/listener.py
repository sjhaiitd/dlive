#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def callback(number):
	if(number.data > 0):
		print "Throttle : ",number.data
	else:
		print "Brake : ",-number.data
		#rospy.loginfo(data)

def listener():
	rospy.init_node('listener', anonymous = True)
	mysub = rospy.Subscriber('/control_effort',Float64,callback)
	print mysub
	rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
