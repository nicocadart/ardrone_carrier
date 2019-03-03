#!/usr/bin/env python
"""
@brief Convert bottom camera to corrected bottom_new fake camera.
@author N. Cadart
@date February 2019
"""
import rospy
from sensor_msgs.msg import CameraInfo, Image


def cam_info_cb(msg):
    msg.header.frame_id = 'ardrone_base_bottomcam_new'
    bottomcam_info_pub.publish(msg)


def image_cb(msg):
    msg.header.frame_id = 'ardrone_base_bottomcam_new'
    bottomcam_img_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('bottomcam_correcter')

    # init publishers and subscribers
    bottomcam_info_sub = rospy.Subscriber('/ardrone/bottom/camera_info', CameraInfo, cam_info_cb, queue_size=10)
    bottomcam_info_pub = rospy.Publisher('/ardrone/bottom_new/camera_info', CameraInfo, queue_size=10)

    bottomcam_img_sub = rospy.Subscriber('/ardrone/bottom/image_raw', Image, image_cb, queue_size=10)
    bottomcam_img_pub = rospy.Publisher('/ardrone/bottom_new/image_raw', Image, queue_size=10)

    rospy.loginfo("Bottom cam converter node ready.")
    rospy.spin()
    rospy.logwarn("Bottom cam converter node shutdown.")
