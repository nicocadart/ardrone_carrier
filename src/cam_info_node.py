#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

def reformat():

    rospy.init_node('reformat', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    sub = rospy.Subscriber('/ardrone/bottom/camera_info', CameraInfo, cam_info_cb)
    while not rospy.is_shutdown():
        rate.sleep()

def cam_info_cb(msg):
    pub = rospy.Publisher('/ardrone/bottom/new_camera_info', CameraInfo, queue_size=10)
    msg.header.frame_id = 'ardrone_base_bottomcam_new'
    pub.publish(msg)

if __name__ == '__main__':
    try:
        reformat()
    except rospy.ROSInterruptException:
        pass
