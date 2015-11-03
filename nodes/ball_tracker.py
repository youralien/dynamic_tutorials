#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import time

# dynamic reconfigure
from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import balltrackerConfig



class BallTracker(object):
    """ The BlobDetector is a Python object that encompasses a ROS node 
        that can process images from the camera and search for blobs within """
    def __init__(self, image_topic):
        """ Initialize the blob detector """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('camera_image')             # a window for the latest camera image
        cv2.namedWindow('hsv_camera_image')             # a window for the latest camera image

        self.srv = Server(balltrackerConfig, self.update_color_params)

        self.hsv = None

        cv2.setMouseCallback('hsv_camera_image', self.process_mouse_event)
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('tracking_window')
 
        # color params
        self.cp = {}
        names = ["blue_l", "blue_u", "green_l", "green_u", "red_l", "red_u"]
        for name in names:
            self.cp[name] = 0

        self.should_move = False

        self.center_x = None
        self.center_y = None

    def update_color_params(self, config, level):
        """ updates the color parameters based on dynamic reconfigure """
        # assign config to color parameters

        self.cp = config
        rospy.loginfo("""Reconfigure Request: {blue_l}, {blue_u},\ 
              {green_l}, {green_u}, {red_l}, {red_u} """.format(**config))
        return config

    def detect(self):
        """ Search for a blob in the last image found """
        if self.cv_image == None:
            return
        my_image = deepcopy(self.cv_image)
        
        thresholded = cv2.inRange(my_image,
                                  (self.cp["blue_l"],self.cp["green_l"],self.cp["red_l"]),
                                  (self.cp["blue_u"],self.cp["green_u"],self.cp["red_u"]))
        moments = cv2.moments(thresholded)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            cv2.circle(my_image,(int(self.center_x), int(self.center_y)), 5, (255,0,0))
        cv2.imshow('tracking_window', thresholded)
        cv2.imshow("camera_image", my_image)
        cv2.waitKey(20)


    def detect_hsv(self):
        """ Search for a blob in the last image found """
        if self.cv_image == None:
            return
        my_image = deepcopy(self.cv_image)
        self.hsv = cv2.cvtColor(my_image, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(self.hsv,
                                  (self.hue_lower,self.saturation_lower,self.value_lower),
                                  (self.hue_upper,self.saturation_upper,self.value_upper))
        moments = cv2.moments(thresholded)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            cv2.circle(my_image,(int(self.center_x), int(self.center_y)), 5, (255,0,0))
        cv2.imshow('hsv_tracking_window', thresholded)
        cv2.imshow("hsv_camera_image", my_image)
        cv2.waitKey(20)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.should_move = not self.should_move
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window, 'Color (h=%d,s=%d,v=%d)' % (self.hsv[y,x,0], self.hsv[y,x,1], self.hsv[y,x,2]), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(20)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # self.detect_hsv()
            self.detect()
            if self.should_move and self.center_x != None:
                offset_x = (self.center_x-320)/320.0
                print -0.5*offset_x
                if abs(offset_x) > 0.25:
                    linear_velocity = 0.0
                else:
                    linear_velocity = 0.2
                self.pub.publish(Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=-1.0*offset_x)))
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()