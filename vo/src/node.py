#!/usr/bin/python           

import socket               # Import socket module
import sys
import rospy
from encoders.msg import encoder_msg
from std_msgs.msg import String
def encoder_data():
	s = socket.socket()         # Create a socket object
	print 'start'
	host= '10.10.10.10'
	port = 6000        # Reserve a port for your service.
	rospy.init_node('encoder_client',anonymous=True)
	s.connect((host, port))
	pub=rospy.Publisher('enc_talker', encoder_msg, queue_size=1000 )
	#pub1=rospy.Publisher('bakchodi', String, queue_size=1000 )
	rate=rospy.Rate(10)
##additional part to be added

##additional part ends
	c=0
	while not rospy.is_shutdown():
		#print 'abcdef'
		enc_data = s.recv(1024)
    	        temp=9
    		#print temp
    		for i in range(9,len(enc_data)):
    			#print i
    			if(enc_data[i].isdigit()):
    				temp=temp+1
    			else:
    				break
    		s1=enc_data[9:temp]
		x=encoder_msg()
    		if(s1.isdigit()):
    			#print 'data= ', int(s1)
    			position=int(s1)
    			x.position=position
    			x.velocity=0
    			x.timestamp=0
			
        		#print "Position_Data %d", x	
   		pub.publish(x)
		#pub1.publish(bakchodi_msg)
		rospy.loginfo(x)
   		rate.sleep()
		c=c+1
		print c
    	#print s1
	s.close    	                 # Close the socket when done

if __name__ == '__main__':
    try:
        encoder_data()
    except rospy.ROSInterruptException:
        pass
