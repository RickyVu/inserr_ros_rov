#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class USBCamera:
    def __init__(self, cam_num, cam_format, v_width, v_height, v_fps):
        self.pub_cam = rospy.Publisher('/inserr/cam/stream', Image, queue_size=10)

        self.cam = cv.VideoCapture(cam_num, cv.CAP_V4L2)

        # Set the video format
        self.cam.set(cv.CAP_PROP_CONVERT_RGB, 0)
        fourcc = cv.VideoWriter_fourcc(*cam_format)
        self.cam.set(cv.CAP_PROP_FOURCC, fourcc)
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, v_width)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, v_height)
        self.cam.set(cv.CAP_PROP_FPS, v_fps)

        self.bridge = CvBridge()

    def run(self):
        ret, image = self.cam.read()
        if not ret:
            rospy.logerr("Failed to capture frame")
            return

        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.pub_cam.publish(msg)



    def cleaup(self):
        self.cam.release()

if __name__ == "__main__":
    try:
        usb_camera = USBCamera(0, 'MJPG', 640, 480, 30)

        rospy.init_node('usb_camera', anonymous=True)
        rate_param = rospy.get_param('~rate', 30.0)
        rate = rospy.Rate(rate_param)

        while not rospy.is_shutdown():
            usb_camera.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        usb_camera.cleaup()