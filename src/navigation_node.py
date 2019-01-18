#!/usr/bin/python
# coding=utf-8
# Import the ROS libraries, and load the manifest file which through <depend package=... />
## will give us access to the project dependencies
import rospy
import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped, PointStamped
import numpy as np

# Import the messages we're interested in sending and receiving
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_carrier.msg import NavigationGoal
from geometry_msgs.msg import Twist     # for sending commands to the drone
from tf.transformations import euler_from_quaternion
from pid import PID



import roslib; roslib.load_manifest('ardrone_tutorials')


# Other imports
# ???

# Some Constants
TARGET_TOPIC_NAME = '/pose_goal'
TARGET_TOPIC_TYPE = NavigationGoal

# EST_POSE_TOPIC_NAME = '/ardrone/navdata' #TODO: check the name
# EST_POSE_TOPIC_TYPE = Navdata

TF_ARDRONE = 'ardrone_base_link'
TF_TARGET = 'pose_goal' # Not a real tf but should be defined by target msg

LOOP_RATE = 10.  #  [Hz] rate of the ROS node loop

class ArdroneNav:
    """ Class for Ardrone navigation: getting target position (and estimated one) and
        return velocity command to the drone with PID control"""

    def __init__(self):
        """Init function (to be completed)"""

        ## Tf attributes
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.tf_ardrone = TF_ARDRONE
        self.tf_target = TF_TARGET


        ####################
        ## Ardrone topics ##
        ####################

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pub_command = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Setup regular publishing of control packets
        self.command = Twist()

        # rate = rospy.Rate(Hertz) à mettre dans le noeud

        #######################################
        ## Positions (target and estimated) ###
        #######################################

        self.sub_target_pos = rospy.Subscriber(TARGET_TOPIC_NAME, TARGET_TOPIC_TYPE,
                                               self.read_target_pose)

        # Units will be meters, seconds, and radiants.

        ## velocities (linear and angular) of the drone at current time

        self.target_pose = PoseStamped()
        self.command_pose = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}
        # Target pose with Euler angle

        self.errors = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        self.vel_constrain = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        self.mode = 0 # RELATIVE or ABSOLUTE for target



        ##########
        ## PID ###
        ##########

        # if no angle target is given, only compute command for position
        self.no_quaternion = False

        # Differents weights for each composant
        self.p = {'position': [0.2, 0.2, 0.2], 'orientation': [0.2, 0.2, 0.2]}
        self.i = {'position': [0.3, 0.3, 0.3], 'orientation': [0.3, 0.3, 0.3]}
        self.d = {'position': [0.1, 0.1, 0.1], 'orientation': [0.1, 0.1, 0.1]}
        self.dt = {'position': [0.0001, 0.0001, 0.0001], 'orientation': [0.0001, 0.0001, 0.0001]}

        self.pids = {}
        self.pids['position'] = [PID(kp, ki, kd, dt) for (kp, ki,
                                                          kd, dt) in zip(self.p['position'],
                                                                         self.i['position'],
                                                                         self.d['position'],
                                                                         self.dt['position'])]

        self.pids['orientation'] = [PID(kp, ki, kd, dt) for (kp, ki,
                                                             kd, dt) in zip(self.p['orientation'],
                                                                            self.i['orientation'],
                                                                            self.d['orientation'],
                                                                            self.dt['orientation'])]

        # If constraints on velocity are defined, activate saturation in PID
        for id in range(len(self.pids['position'])):
            vel_cstr = self.vel_constrain['position'][id]
            if vel_cstr != 0.0:
                self.pids['position'][id].activate_command_saturation(-vel_cstr, vel_cstr)

        for id in range(len(self.pids['orientation'])):
            vel_cstr = self.vel_constrain['orientation'][id]
            if vel_cstr != 0.0:
                self.pids['orientation'][id].activate_command_saturation(-vel_cstr, vel_cstr)



    def set_command(self, command):
        """Define the command msg (Twist) to be published to the drone"""

        vx = command['position'][0]
        vy = command['position'][1]
        vz = command['position'][2]

        rotX = command['position'][0]
        rotY = command['position'][1]
        rotZ = command['position'][2]

        self.command.linear.x = vx*1000. # should be in mm/s
        self.command.linear.y = vy*1000.
        self.command.linear.z = vz*1000.
        self.command.angular.x = (360.*rotX)/(2*np.pi) # should be in degree
        self.command.angular.y = (360.*rotY)/(2*np.pi)
        self.command.angular.z = (360.*rotZ)/(2*np.pi)


    def send_command(self):
        """publish the command twist msg to cmd_vel/"""
        self.pub_command.publish(self.command)


    def read_target_pose(self, msg_pose):
        """Get target position in specific frame. We assume that target is already expressed in the
        control frame """

        self.tf_target = msg_pose.header.frame_id

        # define mode for target, absolute or relative coordinates
        self.mode = msg_pose.mode

        # Get position of target in target_frame
        self.target_pose.pose = msg_pose.pose

        # Check if we ask for angle command
        if self.target_pose.pose.orientation.quaternion == [0.0, 0.0, 0.0, 0.0]:
            self.no_quaternion = True

        # According to mode, define target pose in referential
        if self.mode == NavigationGoal.ABSOLUTE:
            # This does nothing
            print('ABSOLUTE MODE')
        elif self.mode == NavigationGoal.RELATIVE:
            print('RELATIVE MODE')

            # get position of drone in target frame (as transform between origins is the same)
            transform = self.tf_buffer.lookup_transform(self.tf_target,
                                                        self.tf_ardrone,
                                                        rospy.Time(0))

            self.target_pose.pose.position.x += transform.transform.translation.x
            self.target_pose.pose.position.y += transform.transform.translation.y
            self.target_pose.pose.position.z += transform.transform.translation.z

            # modulo pi caping before or after euler?
            self.target_pose.pose.orientation.x += transform.transform.rotation.x
            self.target_pose.pose.orientation.y += transform.transform.rotation.y
            self.target_pose.pose.orientation.z += transform.transform.rotation.z
            self.target_pose.pose.orientation.w += transform.transform.rotation.w

        else:
            raise NotImplementedError

        # Express target pos in drone frame, which gives the error
        command_pos = self.tf_buffer.transform(self.target_pose, self.tf_ardrone)

        # Transform quaternion into Euler angle
        quaternion = self.command_pose.pose.orientation

        if not self.no_quaternion:
            rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z,
                                              quaternion.w))
            # 2PI capping
            rotation = [angle%(2*np.pi) for angle in rotation]
        else:
            rotation = [0.0, 0.0, 0.0]


        # Get orientation and position into a structure for PID control
        self.command_pose['orientation'] = rotation
        self.command_pose['position'] = command_pos.pose.position


    def update(self):
        """compute velocity command through PID"""

        # Set empty command
        command = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        for id in range(len(self.pids['position'])):
           command['position'][id] = self.pids['position'][id].compute_command(\
                                                                self.command_pose['position'][id])

        # if command for angle is not null, compute angle command
        if not self.no_quaternion:

            for id in range(len(self.pids['orientation'])):
                command['orientation'][id] = self.pids['orientation'][id].compute_command(\
                                                                    self.command_pose['orientation'][id])

        # set and send command to the drone
        self.set_command(command)
        self.send_command()

    def run(self):
        # loop at given rate
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            self.update()
            print('update')
            print(self.target_pose)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('navigation')
    try:
        navigation_node = ArdroneNav()
        navigation_node.run()
    except rospy.ROSInterruptException:
        print("Shutting down drone navigation node...")
