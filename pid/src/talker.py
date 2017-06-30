#!/usr/bin/env python
import math
import rospy
import numpy
from std_msgs.msg import Float64

def main():
    pub1 = rospy.Publisher('/setpoint', Float64, queue_size=10)
    rospy.init_node('circler', anonymous=True)
    pub2 = rospy.Publisher('/state',Float64, queue_size = 10)

    rate = rospy.Rate(10) # 2hz
    msga = Float64()
    msga = 1
    msgb = 0
			
    msg1 = 1
    #msg.angular.z = 3

    while not rospy.is_shutdown():
        #msg += 0.2
	msga+=1
	pub1.publish(2*msga)
	pub2.publish(msga)
	rate.sleep()
        #i = 0		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
