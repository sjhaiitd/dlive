#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Quaternion
import numpy as np
import socket
import sys, select, termios, tty
s=socket.socket()
port=22339
host='10.10.10.12'
# s.bind((host,port))
# s.listen(5)
# c1,addr=s.accept()


#class image_converter:

  #def __init__(self):


# def callback(data):
#     try:
#       cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError as e:
#       print(e)

#     (rows,cols,channels) = cv_image.shape
#     if cols > 60 and rows > 60 :
#       cv2.circle(cv_image, (50,50), 10, 255)

#     cv2.imshow("Image window", cv_image)
#     cv2.waitKey(3)


# def callback1(data):
#     try:
#       cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError as e:
#       print(e)

#     (rows,cols,channels) = cv_image.shape
#     if cols > 60 and rows > 60 :
#       cv2.circle(cv_image, (50,50), 10, 255)

#     cv2.imshow("Image window1", cv_image)
#     cv2.waitKey(3)
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _=select.select([sys.stdin], [], [], 0.1)
    if rlist:
       key = sys.stdin.read(1)
    else:
       key=0
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
def callback3(data):

    a=data.pose.pose.orientation.x
    b=data.pose.pose.orientation.y
    c=data.pose.pose.orientation.z
    d=data.pose.pose.orientation.w
    
    euler = tf.transformations.euler_from_quaternion([a,b,c,d])
    eul=np.asarray(euler)
    print eul*180/np.pi
    print data.pose.pose.position
    print data.pose.covariance
    #print '****************'
    soc_data='$VO ,x='+ str(data.pose.pose.position.x)+',y='+str(data.pose.pose.position.y)+',o='+str(eul[2]*180/np.pi)
    #print soc_data
    vo_pub.publish(soc_data)
    #c1.send(soc_data)
    # try:
    #   image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
    current_time=rospy.Time.now()
    odom_quat=tf.transformations.quaternion_from_euler(0,0,eul[2])
    odom_broadcaster.sendTransform((data.pose.pose.position.x,data.pose.pose.position.y,0),odom_quat,current_time,"base_link","odom")
    #odom_msg=Odometry()
    #dom_msg.header.stamp=current_time
    data.header.stamp=current_time
    #odom_msg.header.frame_id="odom"
    data.header.frame_id="odom"
    #odom_msg.pose.pose=Pose(Point(pos_x,pos_y,0),Quaternion(*odom_quat))
    data.child_frame_id="base_link"
    #odom_msg.twist.twist=Twist(Vector3(vx,vy,0),Vector3(0,0,vth))
    odom_pub.publish(data)

settings = termios.tcgetattr(sys.stdin)
rospy.init_node('image_converter', anonymous=True)
odom_pub=rospy.Publisher("odom1",Odometry,queue_size=50)
odom_broadcaster=tf.TransformBroadcaster()
vo_pub = rospy.Publisher("vo_pub",String,queue_size=10)
print "come here"
bridge = CvBridge()
vo_sub = rospy.Subscriber("vo",Odometry,callback3)
#image_sub1 = rospy.Subscriber("/rgb/image_rect_color",Image,callback1)
while(1):
    key=getKey()
    if key=='z':
       break


