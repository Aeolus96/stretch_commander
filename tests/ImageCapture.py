#!/usr/bin/env python

import datetime
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

bridge = CvBridge()
cv_image = None
# Initialize ROS node


def image_callback(msg):

    global cv_image
    try:
        # Convert the ROS image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))
    
    if cv_image is not None:
        cv2.imshow('Captured Image', cv_image)
        cv2.waitKey(1)

def capture_images():
    print("Press Enter to capture an image. Press 'q' to quit.")

    while not rospy.is_shutdown():
        input("Press Enter Key")

        # Save the image with a serialized filename based on the timestamp
        timestamp = rospy.Time.now()
        image_filename = f'captured_image_{timestamp}.jpg'
        image_path = os.path.join(save_directory, image_filename)
        cv2.imwrite(image_path, cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        print(f"Image saved to {image_path}")

if __name__ == '__main__':
    rospy.init_node('image_capture_node', anonymous=True)

    # Subscribe to the image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    save_directory = '/home/terminus/Desktop/Images/'
    
    capture_images()