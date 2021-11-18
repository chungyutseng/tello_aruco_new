#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import time

def convert_color_image(ros_image):
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2.namedWindow("Color")
        cv2.imshow("Color", color_image)
        k = cv2.waitKey(10)
        if k % 256 == 32:
            time_name = str(time.time())
            img_name = time_name + ".png"
            cv2.imwrite(img_name, color_image)
            # imag_counter = imag_counter + 1
    except CvBridgeError as e:
        print(e)

def tellodetect():
    rospy.init_node("tello_aruco_test", anonymous=True)
    rospy.Subscriber("tello/raw_image", Image, callback=convert_color_image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        tellodetect()
    except rospy.ROSInterruptException:
        pass


# <node pkg="image_transport" name="image_compressed" type="republish" args="raw in:=image_raw compressed out:=image_raw" />
# <node pkg="image_transport" name="image_compressed1" type="republish" args="h264 in:=image_raw raw out:=raw_image" />