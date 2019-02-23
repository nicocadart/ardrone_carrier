#!/usr/bin/env python
from __future__ import division, print_function
from enum import IntEnum
import numpy as np

import rospy
import tf2_ros
from std_msgs.msg import Empty
# from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PoseStamped, PointStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import LedAnim, CamSelect

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
FRAME_TARGET = "ar_marker_{}".format(BUNDLE_ID)  # target bundle to follow/land on
FRAME_DRONE = "ardrone_base_link"  # drone
FRAME_WORLD = "odom"  # base frame

# time parameters
LOOP_RATE = 10.  # [Hz] rate of the ROS node loop
BUNDLE_DETECTION_TIMEOUT = 1.  # [s] if a bundle detection is older than this, we go back to FINDING state

BUNDLE_FINDING_DISTANCE_FACTOR = 0.10  # [m] how much we increase distance from approximate target position at each step

# Flight parameters
TAKEOFF_ALLOWED = True  # if False, no takeoff order will be sent (Test mode)
FLIGHT_ALTITUDE = 1.  # [m] general altitude of flight for the drone
FLIGHT_PRECISION = 0.20  # [m] tolerance to reach specified target

LANDING_FACTOR = 0.8  # at each iteration, the drone multiply its distance to target by this factor
LANDING_MIN_ALTITUDE = 0.20  # [m] below this altitude, the drone stops flying and tries to land


class FlightManager:
    def __init__(self):
        """ Object constructor. """
        rospy.loginfo("Waiting for 'ardrone_autonomy' node...")

        # ROS subscribers and publishers
        rospy.wait_for_service("/ardrone/setcamchannel")
        self.cam_srv = rospy.ServiceProxy("/ardrone/setcamchannel", CamSelect)
        self.led_srv = rospy.ServiceProxy("/ardrone/setledanimation", LedAnim)
        self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=1)
        self.nav_pub = rospy.Publisher("/pose_goal", NavigationGoal, queue_size=1)
        self.command_sub = rospy.Subscriber("/command", ArdroneCommand, self._command_callback, queue_size=1)
        self.bundle_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._bundle_callback, queue_size=5)
        self.navdata_sub = rospy.Subscriber("/ardrone/navdata", Navdata, self._navdata_callback, queue_size=5)

        # TF management
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # command and current state of the drone
        self.command = ArdroneCommand.OFF
        self.state = STATE.OFF
        self._change_state(STATE.OFF, previous='')  # used only to display state and set LED animations
        self.drone_state = 0  # Unknown (see https://ardrone-autonomy.readthedocs.io/en/latest/reading.html)

        # target position (approximate location of bundle, in world frame)
        self.target_point = PointStamped()
        self.target_point_precision = 0.
        self.target_point_received = False
        self.research_pose = PoseStamped()  # pose where we are actually looking for the target
        self.research_pose_count = 0  # number of intermediary pose we already visited

        # detected bundle pose
        self.bundle_pose = PoseStamped()
        self.bundle_pose_received = False

        # ensure bottom camera is selected
        self.cam_srv(1)

        # send null pose to navigation
        nav_target = NavigationGoal()
        nav_target.header.frame_id = FRAME_DRONE
        nav_target.mode = NavigationGoal.RELATIVE
        self.nav_pub.publish(nav_target)

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

        # land on exit
        self.land_pub.publish()

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
        elif self.command in {ArdroneCommand.REACH, ArdroneCommand.FIND, ArdroneCommand.TRACK, ArdroneCommand.LAND}:
            self._change_state(STATE.REACHING)

    def reaching_loop(self):
        """
        Fly to approximate target where bundle is supposed to be located.
        """
        # if new target has been received, send its pose to navigation node
        if self.target_point_received:
            self.target_point_received = False
            nav_target = NavigationGoal()
            nav_target.header = self.target_point.header
            nav_target.pose.position = self.target_point.point
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.nav_pub.publish(nav_target)

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
            elif self.command in {ArdroneCommand.FIND, ArdroneCommand.TRACK, ArdroneCommand.LAND}:
                self._change_state(STATE.FINDING)

    def finding_loop(self):
        """
        Drone has to fly around and find the marker/bundle.
        """
        # if new bundle detection has been received, target has been found !
        if self.bundle_pose_received:
            self.bundle_pose_received = False
            # send null pose to navigation
            nav_target = NavigationGoal()
            nav_target.header.frame_id = FRAME_DRONE
            nav_target.mode = NavigationGoal.RELATIVE
            self.nav_pub.publish(nav_target)
            # if command was only to find target, reset order and standby
            if self.command == ArdroneCommand.FIND:
                self.command = ArdroneCommand.STANDBY
                self._change_state(STATE.STANDBY)
            # if command was to track target, move on to next state
            elif self.command in {ArdroneCommand.TRACK, ArdroneCommand.LAND}:
                self._change_state(STATE.TRACKING)

        # otherwise, fly around to find target
        else:
            # compute distance to current intermediary target
            if self.research_pose_count > 0:
                self.research_pose.header.stamp = rospy.Time(0)  # reset time to get transform no matter the timestamp
                error = self.tf_buffer.transform(self.research_pose, FRAME_DRONE).pose.position
                distance = np.sqrt(np.sum(np.array([error.x, error.y, error.z]) ** 2))
            else:
                distance = 0.

            # if intermediary target has been reached without finding target, go to another point
            if distance <= FLIGHT_PRECISION:
                self.research_pose_count += 1
                research_radius = BUNDLE_FINDING_DISTANCE_FACTOR * self.research_pose_count

                # if research radius is becoming too large, we have probably missed the target. Try again.
                if research_radius > self.target_point_precision:
                    self.research_pose_count = 0
                    research_radius = 0

                # set next intermediary goal
                self.research_pose = PoseStamped()
                self.research_pose.header.frame_id = FRAME_WORLD
                self.research_pose.pose.position = self.target_point.point
                self.research_pose.pose.position.x += research_radius * np.sin(self.research_pose_count * np.pi / 2)
                self.research_pose.pose.position.y += research_radius * np.cos(self.research_pose_count * np.pi / 2)

                # send research pose to navigation
                nav_target = NavigationGoal()
                nav_target.header = self.research_pose.header
                nav_target.header.stamp = rospy.Time.now()
                nav_target.pose = self.research_pose.pose
                nav_target.mode = NavigationGoal.ABSOLUTE
                self.nav_pub.publish(nav_target)

    def tracking_loop(self):
        """
        Drone has to follow a specific bundle or marker published by ar_track_alvar
        """
        # if new bundle detection has been received, send its pose to navigation node
        if self.bundle_pose_received:
            self.bundle_pose_received = False
            # compute pose at FLIGHT_ALTITUDE meters above bundle pose
            above_bundle_pose = self.tf_buffer.transform(self.bundle_pose, FRAME_WORLD)
            above_bundle_pose.pose.position.z += FLIGHT_ALTITUDE
            # send tracking pose to navigation
            nav_target = NavigationGoal()
            nav_target.header = above_bundle_pose.header
            nav_target.pose.position = above_bundle_pose.pose.position
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.target_point.point = nav_target.pose.position
            self.nav_pub.publish(nav_target)

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
        # compute distance of drone to target
        # # either consider only difference of altitudes
        # bundle_pose = self.tf_buffer.transform(self.bundle_pose, FRAME_WORLD)
        # drone_position = self.tf_buffer.lookup_transform(FRAME_WORLD, FRAME_DRONE, rospy.Duration(0)).transform.translation
        # distance = drone_position.z - bundle_pose.pose.position.z
        # or consider euclidian distance between drone and target
        error = self.tf_buffer.transform(self.bundle_pose, FRAME_DRONE).pose.position
        error = np.array([error.x, error.y, error.z])
        distance = np.sqrt(np.sum(error ** 2))

        # continue tracking target and decrease altitude smoothly
        if self.bundle_pose_received:
            self.bundle_pose_received = False
            # compute pose above bundle and reduce altitude to get closer to it
            above_bundle_pose = self.tf_buffer.transform(self.bundle_pose, FRAME_WORLD)
            above_bundle_pose.pose.position.z += LANDING_FACTOR * distance
            # send tracking pose to navigation
            nav_target = NavigationGoal()
            nav_target.header = above_bundle_pose.header
            nav_target.pose.position = above_bundle_pose.pose.position
            nav_target.mode = NavigationGoal.ABSOLUTE
            self.target_point.point = nav_target.pose.position
            self.nav_pub.publish(nav_target)

        # if drone is landing or has landed, go to OFF state
        if self.drone_state in {0, 1, 2, 8}:
            rospy.loginfo("Landing on target...")
            self._change_state(STATE.OFF)

        # if last bundle detection is too old, we have probably lost the target : we have to find it again
        elif (rospy.Time.now() - self.bundle_pose.header.stamp).to_sec() > BUNDLE_DETECTION_TIMEOUT:
            self._change_state(STATE.FINDING, warn=True)

        # if very close to target, land
        elif distance < LANDING_MIN_ALTITUDE:
            self.land_pub.publish()

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

        # GREEN when drone is ready and landed
        if new_state == STATE.OFF:
            # send null pose to navigation
            nav_target = NavigationGoal()
            nav_target.header.frame_id = FRAME_DRONE
            nav_target.mode = NavigationGoal.RELATIVE
            self.nav_pub.publish(nav_target)
            self.led_srv(type=8, freq=1.0, duration=5)

        # BLINK_GREEN_RED when we are looking for target
        if new_state == STATE.FINDING:
            self.research_pose_count = 0
            self.led_srv(type=0, freq=3.0, duration=0)

        # BLINK_GREEN when drone is tracking or landing on target
        if new_state in {STATE.TRACKING, STATE.LANDING}:
            self.led_srv(type=1, freq=3.0, duration=0)

        # change state
        self.state = new_state

    # =======================  ROS callbacks =======================

    def _bundle_callback(self, msg):
        """
        Update detected bundle pose.
        :param msg: AlvarMarkers msg
        """
        # only update bundle pose if we need it
        if self.state in {STATE.FINDING, STATE.TRACKING, STATE.LANDING}:
            # loop though detections and check if the interesting bundle has been detected
            for bundle in msg.markers:
                if bundle.id == BUNDLE_ID:
                    self.bundle_pose = bundle.pose
                    self.bundle_pose.header.stamp = bundle.header.stamp
                    self.bundle_pose.header.frame_id = bundle.header.frame_id.strip('/')  # remove '/' in frame_id
                    self.bundle_pose_received = True
                    return

    def _navdata_callback(self, msg):
        """
        Get drone state.
        :param msg:  Navdata msg
        """
        # update drone state (only if test mode is deactivated)
        self.drone_state = msg.state if TAKEOFF_ALLOWED else 0
        # check battery, and RED and print msg if it is too low
        if msg.batteryPercent < 15. and msg.header.seq % 1000 == 0:
            rospy.logwarn("Low battery : {}% !".format(msg.batteryPercent))
            self.led_srv(type=7, freq=1.0, duration=0)

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

        # ORANGE when receiving a new order
        self.led_srv(type=3, freq=10, duration=1)
        # ensure bottom camera is selected
        self.cam_srv(1)
        valid_command = True

        # send landing order
        if msg.command == ArdroneCommand.OFF:
            self._change_state(STATE.OFF, warn=True, previous='')

        # send take-off order and wait
        elif msg.command == ArdroneCommand.STANDBY:
            self._change_state(STATE.STANDBY, previous='')

        # land on target
        elif msg.command == ArdroneCommand.LAND and msg.position == Point() and msg.precision == 0.:
            if self.state != STATE.TRACKING:
                valid_command = False
                rospy.logerr("No target currently tracked : cannot change to LANDING state.")

        # move to specified location
        elif msg.command in {ArdroneCommand.REACH, ArdroneCommand.FIND, ArdroneCommand.TRACK, ArdroneCommand.LAND}:
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

        # debug mode to force state change when sending the negative order
        elif msg.command < 0:
            if -msg.command == ArdroneCommand.REACH:
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
            self._change_state(STATE(-msg.command), warn=True, previous='debug')

        else:
            valid_command = False

        # save command
        if valid_command:
            self.command = abs(msg.command)


if __name__ == '__main__':
    rospy.init_node('flight_manager')
    try:
        flight_manager = FlightManager()
        flight_manager.run()
    except rospy.ROSInterruptException:
        print("Shutting down flight manager node...")
