#!/usr/bin/env python
from __future__ import division, print_function

import rospy
import tf2_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

from ardrone_carrier.msg import NavigationGoal, ArdroneCommand


# states of the drone
STATE_OFF      = ArdroneCommand.OFF      # wait for new order
STATE_STANDBY  = ArdroneCommand.STANDBY  # takeoff and wait
STATE_REACHING = ArdroneCommand.REACH    # fly to approximate location where target is
STATE_FINDING  = ArdroneCommand.FIND     # find target near a given location
STATE_TRACKING = ArdroneCommand.TRACK    # follow target
STATE_LANDING  = ArdroneCommand.LAND     # land on target

# frames ids
BUNDLE_ID = 8  # id of the master marker in the bundle
# FRAME_TARGET = "/marker_{}".format(BUNDLE_ID)  # target bundle to follow/land on
# FRAME_DRONE = "/ardrone_base_link"  # drone

# time parameters
LOOP_RATE = 10.  # [Hz] rate of the ROS node loop
BUNDLE_DETECTION_TIMEOUT = 1.  # [s] if a bundle detection is older than this, we go back to FINDING state


class FlightManager:
    def __init__(self):
        # ROS subscribers and publishers
        self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=1)
        self.nav_pub = rospy.Publisher("/pose_goal", NavigationGoal, queue_size=1)
        self.command_sub = rospy.Subscriber("/command", ArdroneCommand, self._command_callback, queue_size=5)
        self.bundle_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._bundle_callback, queue_size=5)

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # command and current state of the drone
        self.command = STATE_OFF
        self.state = STATE_OFF
        # target pose (approximate location of bundle)
        self.target_pose = PoseStamped()
        self.target_pose_precision = 0.
        self.target_pose_received = False
        # detected bundle pose
        self.bundle_pose = PoseStamped()
        self.bundle_pose_received = False


    def run(self):
        # loop at given rate
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():

            # call right controller according to current state
            if self.state == STATE_OFF:
                pass
            elif self.state == STATE_STANDBY:
                pass
            elif self.state == STATE_REACHING:
                self.reaching_loop()
            elif self.state == STATE_FINDING:
                self.finding_loop()
            elif self.state == STATE_TRACKING:
                self.tracking_loop()
            elif self.state == STATE_LANDING:
                self.landing_loop()
            else:
                rospy.logerr("Unknown flight state : {}".format(self.state))

            rate.sleep()

    # =======================  States loops =======================

    def reaching_loop(self):
        """
        Fly to approximate target where bundle is supposed to be located.
        """
        # if drone has arrived to target pose, find the bundle
        # TODO
        pass


    def finding_loop(self):
        """
        Drone has to fly around and find the marker/bundle.
        """
        # if new bundle detection has been received, change to TRACKING state
        if self.bundle_pose_received:
            self.state = STATE_TRACKING
            rospy.loginfo("FINDING --> TRACKING")
            return

        # otherwise, fly around
        # TODO


    def tracking_loop(self):
        """
        Drone has to follow a specific bundle or marker published by ar_track_alvar
        """
        # if new bundle detection has been received, send its pose to navigation node
        if self.bundle_pose_received:
            nav_target = NavigationGoal()
            nav_target.header = self.bundle_pose.header
            nav_target.pose = self.bundle_pose.pose
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.nav_pub.publish(nav_target)
            self.bundle_pose_received = False

        # if last bundle detection is too old, we have probably lost the target : we have to find it again
        if (rospy.Time.now() - self.bundle_pose.header.stamp).to_sec() > BUNDLE_DETECTION_TIMEOUT:
            self.state = STATE_FINDING
            rospy.logwarn("TRACKING --> FINDING")


    def landing_loop(self):
        """
        Drone has to land on bundle.
        """
        # TODO
        pass

    # =======================  ROS callbacks =======================

    def _bundle_callback(self, msg):
        """
        Update detected bundle pose.
        :param msg: AlvarMarkers msg
        """
        # loop though detections and check if the interesting bundle has been detected
        for bundle in msg.markers:
            if bundle.id == BUNDLE_ID:
                self.bundle_pose = bundle.pose
                self.bundle_pose.header = bundle.header
                self.bundle_pose_received = True
                return

    def _command_callback(self, msg):
        """
        Receive new order.
        :param msg: ArdroneCommand msg
        """
        # save command
        self.command = msg.command

        # send landing order
        if msg.command == ArdroneCommand.OFF:
            self.land_pub.publish()
            self.state = STATE_OFF
            rospy.logwarn(" --> OFF")

        # send take-off order and wait
        elif msg.command == ArdroneCommand.STANDBY:
            self.takeoff_pub.publish()
            self.state = STATE_STANDBY
            rospy.loginfo(" --> STANDBY")

        # move to specified location
        elif msg.command in {ArdroneCommand.REACH, ArdroneCommand.FIND, ArdroneCommand.TRACK}:
            self.target_pose.pose = msg.pose
            self.target_pose.header = msg.header
            self.target_pose_precision = msg.precision
            self.target_pose_received = True
            self.state = STATE_REACHING
            rospy.loginfo(" --> REACHING")

        # land on target
        elif msg.command == ArdroneCommand.LAND:
            self.state = STATE_LANDING
            rospy.loginfo(" --> LANDING")


if __name__ == '__main__':
    rospy.init_node('manager')
    try:
        flight_manager = FlightManager()
        flight_manager.run()
    except rospy.ROSInterruptException:
        print("Shutting down flight manager node...")
