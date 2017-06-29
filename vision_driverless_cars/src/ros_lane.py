#!/usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from thresholder import Thresholder
from warper import Warper
from polyfitter import Polyfitter
from polydrawer import Polydrawer


class Image_process(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_subscribe = rospy.Subscriber("/rgb/image_rect_color", Image, self.image_callback)
        self.depth_subscribe = rospy.Subscriber("/depth/depth_registered", Image, self.depth_callback)
    


    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image.astype('uint8')
        except CvBridgeError as e:
            print (e)
        out.write(image)
        thresh = threshold_object.threshold(image)
        warp = warp_object.warp(thresh)
        # print image.shape
        cv2.imshow('Threshold', thresh)
        cv2.imshow('Image', image)
        cv2.imshow('Depth', depth)
        cv2.waitKey(1)

    def depth_callback(self, data):
        try:
            global depth
            depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # cv_image.astype('uint8')
        except CvBridgeError as e:
            print (e)
        # out.write(cv_image)
        # print depth.shape
        # cv2.imshow('Depth', depth)
        # cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_display', anonymous=True)
    threshold_object = Thresholder()
    warp_object = Warper()
    fitt_object = Polyfitter()
    draw_object = Polydrawer()
    # fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('test.avi', fourcc, 30.0, (1080, 720))
    image_object = Image_process()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.is_shutdown()
        print 'Shutting Down'
        out.release()
    cv2.destroyAllWindows()
