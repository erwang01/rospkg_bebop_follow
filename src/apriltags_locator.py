#!/usr/bin/env python

""" apriltags_locator.py
Node that listens to tag_detections and calculates the location of it in pixels
Optionally overlays a red circle over the april tag from camera_raw and outputs it as image_overlay

TODO make image option a paramter when the node is launched from a luanch file

Currently calibrated for -51 degrees tilt and 0 degrees pan on the Bebop 2. More calibration is needed.
"""

import rospy
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

class ApriltagsLocator():
    def __init__(self, image=False):
        # Creates node apriltag_locator
        #subscribes to tag_detections (location of apriltags relative to camera) and image_raw (video feed).
        #Publishes tag_location (location of the apriltags in pixels) and image_overlay (video feed with red circle over april tag)
        rospy.init_node("apriltag_locator")
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.process_apriltag)
        self.pub = rospy.Publisher("tag_location", Point, queue_size = 10)
        # Video feeds only active when specified in constructor
        if image:
            rospy.Subscriber("/bebop/image_raw", Image, self.image_overlay)
            self.img_pub = rospy.Publisher("image_overlay", Image, queue_size=10)
            self.bridge = CvBridge()
        self.xp = self.yp = 0

    # Run until node is stopped
    def run(self):
        rospy.spin()

    # Called when tag_detections from apriltag2_ros sends updates
    # Maps 3D apriltag location to pixel coordinates and publishes those coordinates to tag_location
    def process_apriltag(self, data):
        detections = data.detections
        # Only if the apriltag is detected
        if len(detections) > 0:
            pose = detections[0].pose.pose.pose
            position = pose.position
            f = 2.3
            sx = 300
            sy = 300
            cx = 370
            cy = 220
            z = f
            y = f*position.y/position.z
            x = f*position.x/position.z
            self.xp = int(sx*x+cx)
            self.yp = int(sy*y+cy)
            self.pub.publish(self.xp,self.yp,0)
        # Otherwise return origin
        else:
            self.pub.publish(0,0,0)

    # Called when new frame from the camera comes in
    # Overlays a red circle over the last known location of the apriltag
    def image_overlay(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.circle(cv_image, (self.xp, self.yp), 10, (0,0,255))
        overlay_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.img_pub.publish(overlay_image)

if __name__=="__main__":
    al = ApriltagsLocator(image=True)
    al.run()
