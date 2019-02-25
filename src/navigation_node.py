#!/usr/bin/python
# coding=utf-8
import numpy as np

import rospy
import tf2_ros as tf
from tf.transformations import euler_from_quaternion, quaternion_multiply
from tf2_geometry_msgs import PoseStamped, PointStamped
from ardrone_carrier.msg import NavigationGoal
from geometry_msgs.msg import Twist, Quaternion  # for sending commands to the drone

from pid import PID


FRAME_ARDRONE = 'ardrone_base_link'
FRAME_FIXED = 'odom'

LOOP_RATE = 30.  #  [Hz] rate of the ROS node loop

TARGET_THRESHOLD_POSE = 0.15  # [m]
TARGET_THRESHOLD_ORIENTATION = 10 * np.pi / 180.  # [rad]
TARGET_THRESHOLD_SPEED = 0.1  # [m/s]


def normalize_angle(rad):
    """
    Ensure that a given angle is in range [-pi; pi[
    """
    return ((rad + np.pi) % (2 * np.pi)) - np.pi


class ArdroneNav:
    """
    Class for Ardrone navigation: getting target position (and estimated one) and
    return velocity command to the drone with PID control
    """
    def __init__(self):
        """
        Init function (to be completed)
        """
        # Tf attributes
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.frame_drone = FRAME_ARDRONE
        self.frame_target = ''  # will be defined in /pose_goal msgs
        self.frame_fixed = FRAME_FIXED

        # ROS publishers/subscribers
        self.pub_command = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # drone speed command
        self.pub_rviz_target_pose = rospy.Publisher('/rviz/pose_goal', PoseStamped, queue_size=1)
        self.sub_target_pos = rospy.Subscriber('/pose_goal', NavigationGoal, self.pose_goal_callback)

        # ----- Init control flags and target -----

        # target pose to reach
        self.target_pose = None  # PoseStamped()

        # if no angle target is given, only compute command for position
        self.no_angle_control = False

        # ----- Init PID controllers -----

        # saturation of commands
        self.cmd_constrain = {'trans_x': 1.,
                              'trans_y': 1.,
                              'trans_z': 1.,
                              'rot_z': 0.7}

        # init PID gains
        self.pid_gains = {'trans_x': {'P': 0.2, 'I': 0.01, 'D': 0.03},
                          'trans_y': {'P': 0.2, 'I': 0.01, 'D': 0.03},
                          'trans_z': {'P': 3.,  'I': 0.01, 'D': 0.05},
                          'rot_z':   {'P': 0.,  'I': 0., 'D': 0.}}

        # init PID controllers
        self.pid = {}
        for pid_name in self.pid_gains:
            self.pid[pid_name] = PID(self.pid_gains[pid_name]['P'],
                                     self.pid_gains[pid_name]['I'],
                                     self.pid_gains[pid_name]['D'],
                                     1. / LOOP_RATE)

        # activate saturation if constrains are defined
        for pid_name in self.pid:
            cmd_cstr = self.cmd_constrain[pid_name]
            if cmd_cstr != 0.0:
                self.pid[pid_name].activate_command_saturation(-cmd_cstr, cmd_cstr)
                print("Saturation activated for PID '%s'" % pid_name)

        # wait for drone pose to be available
        rospy.loginfo("Waiting for drone TF to be published...")
        while (not rospy.is_shutdown() and 
               not self.tf_buffer.can_transform(self.frame_fixed, self.frame_drone, rospy.Time(0), rospy.Duration(1.))):
            pass

        rospy.loginfo("Navigation successfully initialized and ready.")


    def run(self):
        """ Main loop """
        # loop at given rate
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            # update command only if active goal
            if self.target_pose:
                self.update()
            rate.sleep()


    def update(self):
        """
        Compute velocity command through PID
        """
        # compute error from drone current pose to target
        error = self.compute_error()

        # set empty command
        command = {'trans_x': 0.,
                    'trans_y': 0.,
                    'trans_z': 0.,
                    'rot_z':   0.}

        # compute PID command
        for pid_name in self.pid_gains:
            if (pid_name.split('_')[0] == 'trans') or (not self.no_angle_control):
                command[pid_name] = self.pid[pid_name].compute_command(error[pid_name])

        # set drone velocity command
        cmd_vel = Twist()

        # if drone has reached target, send one null command to enable hover mode
        if np.sqrt(error['trans_x'] ** 2 + error['trans_y'] ** 2 + error['trans_z'] ** 2) < TARGET_THRESHOLD_POSE \
            and np.abs(error['rot_z']) < TARGET_THRESHOLD_ORIENTATION:
            rospy.loginfo('Target reached !')
            self.target_pose = None

        # otherwise, update order with computed PID commands
        else:
            # TODO : check units (m/s or mm/s, rad/s or deg/s)
            cmd_vel.linear.x = command['trans_x']
            cmd_vel.linear.y = command['trans_y']
            cmd_vel.linear.z = command['trans_z']
            cmd_vel.angular.z = command['rot_z']

        # publish the command msg to drone
        self.pub_command.publish(cmd_vel)


    def compute_error(self):
        """
        With target expressed in target frame, compute real time error with ardrone position, with tf
        """
        # Express target pose in drone frame, which gives the error
        target_pose = self.target_pose
        target_pose.header.stamp = rospy.Time(0)
        error_pose = self.tf_buffer.transform(self.target_pose, self.frame_drone)

        # Transform quaternion into Euler (RPY) angle
        if self.no_angle_control:
            rotation = [0.0, 0.0, 0.0]
        else:
            quaternion = error_pose.pose.orientation
            rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
            rotation = [normalize_angle(angle) for angle in rotation]

        # Compute orientation and positio2n errors into a structure for PID control
        error = {'trans_x': error_pose.pose.position.x,
                 'trans_y': error_pose.pose.position.y,
                 'trans_z': error_pose.pose.position.z,
                 'rot_z': rotation[2]}

        return error


    def pose_goal_callback(self, msg_pose):
        """
        Get target position in specific frame.
        We assume that target is already expressed in the control frame
        :param msg_pose: NavigationGoal msg
        """
        # Get position of target in target_frame
        self.target_pose = PoseStamped()
        self.target_pose.pose = msg_pose.pose
        self.target_pose.header = msg_pose.header
        self.frame_target = msg_pose.header.frame_id

        # Check if we ask for angle command
        if ([self.target_pose.pose.orientation.x,
             self.target_pose.pose.orientation.y,
             self.target_pose.pose.orientation.z,
             self.target_pose.pose.orientation.w] == [0.0, 0.0, 0.0, 0.0]):
            self.no_angle_control = True
            self.target_pose.pose.orientation.w = 1.0

        # According to mode, define target pose in referential
        if msg_pose.mode == NavigationGoal.ABSOLUTE:
            rospy.loginfo('New ABSOLUTE target received.')

        elif msg_pose.mode == NavigationGoal.RELATIVE:
            rospy.loginfo('New RELATIVE target received.')

            # TODO: reset angle PIDs
            # get position of drone in target frame (as transform between origins is the same)
            transform = self.tf_buffer.lookup_transform(self.frame_target,
                                                        self.frame_drone,
                                                        rospy.Time(0))

            self.target_pose.pose.position.x += transform.transform.translation.x
            self.target_pose.pose.position.y += transform.transform.translation.y
            self.target_pose.pose.position.z += transform.transform.translation.z

            # TODO: check composition of quaternion
            xyzw_array = lambda o: [o.x, o.y, o.z, o.w]
            self.target_pose.pose.orientation = Quaternion(*quaternion_multiply(xyzw_array(self.target_pose.pose.orientation),
                                                                                xyzw_array(transform.transform.rotation)))

        else:
            rospy.logerr('Invvalid target recievd (unknown mode')

        # convert target pose into a fixed frame to avoid buffering its transform for a long time
        self.target_pose = self.tf_buffer.transform(self.target_pose, self.frame_fixed)

        # publish target pose to display it with rviz
        self.pub_rviz_target_pose.publish(self.target_pose)


if __name__ == '__main__':
    rospy.init_node('navigation')
    try:
        navigation_node = ArdroneNav()
        navigation_node.run()
    except rospy.ROSInterruptException:
        print("Shutting down drone navigation node...")
