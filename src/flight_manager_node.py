#!/usr/bin/env python
from __future__ import division, print_function
from enum import IntEnum
import numpy as np

import rospy
import tf2_ros
from std_msgs.msg import Empty
# from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_geometry_msgs import PoseStamped, PointStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ardrone_autonomy.msg import Navdata

from ardrone_carrier.msg import NavigationGoal, ArdroneCommand


# states of the drone
class STATE(IntEnum):
    OFF      = ArdroneCommand.OFF      # wait for new order
    STANDBY  = ArdroneCommand.STANDBY  # takeoff and wait
    REACHING = ArdroneCommand.REACH    # fly to approximate location where target is
    FINDING  = ArdroneCommand.FIND     # find target near a given location
    TRACKING = ArdroneCommand.TRACK    # follow target
    LANDING  = ArdroneCommand.LAND     # land on target


# frames ids
BUNDLE_ID = 3  # id of the master marker in the bundle
FRAME_TARGET = "marker_{}".format(BUNDLE_ID)  # target bundle to follow/land on
FRAME_DRONE = "ardrone_base_link"  # drone
FRAME_WORLD = "odom"  # base frame

# time parameters
LOOP_RATE = 10.  # [Hz] rate of the ROS node loop
BUNDLE_DETECTION_TIMEOUT = 1.  # [s] if a bundle detection is older than this, we go back to FINDING state

FLIGHT_ALTITUDE = 1.  # [m] general altitude of flight for the drone
FLIGHT_PRECISION = 0.20  # [m] tolerance to reach specified target


class FlightManager:
    def __init__(self):
        """ Object constructor. """
        # ROS subscribers and publishers
        self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=1)
        self.nav_pub = rospy.Publisher("/pose_goal", NavigationGoal, queue_size=1)
        self.command_sub = rospy.Subscriber("/command", ArdroneCommand, self._command_callback, queue_size=5)
        self.bundle_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._bundle_callback, queue_size=5)
        self.navdata_sub = rospy.Subscriber("/ardrone/navdata", Navdata, self._navdata_callback, queue_size=1)

        # TF management
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # command and current state of the drone
        self.command = ArdroneCommand.OFF
        self.state = STATE.OFF
        self._change_state(STATE.OFF, previous='')  # used only to display state and set LED animations
        self.drone_state = 0  # Unknown (see https://ardrone-autonomy.readthedocs.io/en/latest/reading.html)

        # target position (approximate location of bundle)
        self.target_point = PointStamped()
        self.target_point_precision = 0.
        self.target_point_received = False

        # detected bundle pose
        self.bundle_pose = PoseStamped()
        self.bundle_pose_received = False

        rospy.loginfo("Flight manager successfully initialized and ready.")

    def run(self):
        """ Main loop running until node is killed. """
        # loop at given rate
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():

            # call right controller according to current state
            if self.state == STATE.OFF:
                self.off_loop()
            elif self.state == STATE.STANDBY:
                self.standby_loop()
            elif self.state == STATE.REACHING:
                self.reaching_loop()
            elif self.state == STATE.FINDING:
                self.finding_loop()
            elif self.state == STATE.TRACKING:
                self.tracking_loop()
            elif self.state == STATE.LANDING:
                self.landing_loop()
            else:
                rospy.logerr("Unknown flight state : {}".format(self.state))

            rate.sleep()

    # =======================  States loops =======================

    def off_loop(self):
        """
        Land and wait.
        """
        # if drone is not landed, send landing order
        if self.drone_state not in {1, 2, 8}:
            self.land_pub.publish()

    def standby_loop(self):
        """
        Take-off and hover.
        """
        # if drone is landed, take-off
        if self.drone_state in {1, 2, 8}:
            self.takeoff_pub.publish()

        # if drone is hovering, go to next state if necessary
        elif self.command in {ArdroneCommand.REACH, ArdroneCommand.FIND, ArdroneCommand.TRACK}:
            self._change_state(STATE.REACHING)

    def reaching_loop(self):
        """
        Fly to approximate target where bundle is supposed to be located.
        """
        # if new target has been received, send its pose to navigation node
        if self.target_point_received:
            nav_target = NavigationGoal()
            nav_target.header = self.target_point.header
            nav_target.pose.position = self.target_point.point
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.nav_pub.publish(nav_target)
            self.target_point_received = False

        # compute distance to target
        distance_to_target = self.tf_buffer.transform(self.target_point, FRAME_DRONE).point
        error = np.array([distance_to_target.x, distance_to_target.y, distance_to_target.z])

        # if drone has arrived to target pose (within tolerance radius)
        if np.sqrt(np.sum(error ** 2)) < FLIGHT_PRECISION:
            # if command was only to reach target, reset order and standby
            if self.command == ArdroneCommand.REACH:
                self.command = ArdroneCommand.STANDBY
                self._change_state(STATE.STANDBY)
            # if command was to find or track target, move on to next state
            elif self.command in {ArdroneCommand.FIND, ArdroneCommand.TRACK}:
                self._change_state(STATE.FINDING)

    def finding_loop(self):
        """
        Drone has to fly around and find the marker/bundle.
        """
        # if new bundle detection has been received, target has been found !
        if self.bundle_pose_received:
            # if command was only to find target, reset order and standby
            if self.command == ArdroneCommand.FIND:
                self.command = ArdroneCommand.STANDBY
                self._change_state(STATE.STANDBY)
            # if command was to track target, move on to next state
            elif self.command == ArdroneCommand.TRACK:
                self._change_state(STATE.TRACKING)

        # otherwise, fly around to find target
        else:
            # TODO
            pass

    def tracking_loop(self):
        """
        Drone has to follow a specific bundle or marker published by ar_track_alvar
        """
        # if new bundle detection has been received, send its pose to navigation node
        # TODO : modify flight height of the bundle
        if self.bundle_pose_received:
            nav_target = NavigationGoal()
            nav_target.header = self.bundle_pose.header
            nav_target.pose = self.bundle_pose.pose
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.nav_pub.publish(nav_target)
            self.bundle_pose_received = False

        # if last bundle detection is too old, we have probably lost the target : we have to find it again
        if (rospy.Time.now() - self.bundle_pose.header.stamp).to_sec() > BUNDLE_DETECTION_TIMEOUT:
            self._change_state(STATE.FINDING, warn=True)

        # if order to land on target has been received, change state
        elif self.command == ArdroneCommand.LAND:
            self._change_state(STATE.LANDING)

    def landing_loop(self):
        """
        Drone has to land on bundle.
        """
        # continue tracking target, decrease altitude smoothly, then land on target
        # TODO

        # if drone is landing or has landed, go to OFF state
        if self.drone_state in {1, 2, 8}:
            rospy.loginfo("Landed on target.")
            self._change_state(STATE.OFF)

        # if last bundle detection is too old, we have probably lost the target : we have to find it again
        elif (rospy.Time.now() - self.bundle_pose.header.stamp).to_sec() > BUNDLE_DETECTION_TIMEOUT:
            self._change_state(STATE.FINDING, warn=True)

    # =======================    Utilities   =======================

    def _change_state(self, new_state, warn=False, previous=None):
        """
        Change current state of the flight manager.
        :param new_state: new state to set
        :param warn: if True, publish on ROS_STREAM_WARN, else ROS_STREAM_INFO
        :param previous: string, if not None, replace previous state in display
        """
        # display state change
        previous_state_str = self.state.name if previous is None else previous
        if warn:
            rospy.logwarn("{} --> {}".format(previous_state_str, new_state.name))
        else:
            rospy.loginfo("{} --> {}".format(previous_state_str, new_state.name))

        # change state
        self.state = new_state

        # set LED animations depending on new state
        # TODO

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

    def _navdata_callback(self, msg):
        """
        Get drone state.
        :param msg:  Navdata msg
        """
        self.drone_state = msg.state
        if msg.batteryPercent < 15.:
            rospy.logwarn("Low battery : {}%".format(msg.batteryPercent))

    def _command_callback(self, msg):
        """
        Receive new order.
        :param msg: ArdroneCommand msg
        """
        # check command validity
        try:
            _ = STATE(abs(msg.command))
        except ValueError:
            rospy.logerr("Received unkown command : {}".format(msg.command))
            return

        # save command
        self.command = msg.command

        # send landing order
        if msg.command == ArdroneCommand.OFF:
            self._change_state(STATE.OFF, warn=True, previous='')

        # send take-off order and wait
        elif msg.command == ArdroneCommand.STANDBY:
            self._change_state(STATE.STANDBY, previous='')

        # move to specified location
        elif msg.command in {ArdroneCommand.REACH, ArdroneCommand.FIND, ArdroneCommand.TRACK}:
            # save message
            self.target_point.header = msg.header
            self.target_point.point = msg.position
            self.target_point_precision = msg.precision
            # if frame_id is not defined, assume it is the world frame
            if not self.target_point.header.frame_id:
                self.target_point.header.frame_id = FRAME_WORLD
            # transform target position to world frame
            self.target_point = self.tf_buffer.transform(self.target_point, FRAME_WORLD)
            # fly at a specific altitude from target point
            self.target_point.point.z += FLIGHT_ALTITUDE
            # notify msg reception and change state
            self.target_point_received = True
            self._change_state(STATE.STANDBY, previous='')

        # land on target
        elif msg.command == ArdroneCommand.LAND:
            if self.state != STATE.TRACKING:
                rospy.logerr("No target currently tracked : cannot change to LANDING state.")

        # debug mode to force state change when sending the negative order
        elif msg.command < 0:
            self._change_state(STATE(-msg.command), warn=True, previous='debug')


if __name__ == '__main__':
    rospy.init_node('flight_manager')
    try:
        flight_manager = FlightManager()
        flight_manager.run()
    except rospy.ROSInterruptException:
        print("Shutting down flight manager node...")
