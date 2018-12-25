# Import the ROS libraries, and load the manifest file which through <depend package=... />
## will give us access to the project dependencies
import rospy
import tf2_ros as tf
import numpy as np

# Import the messages we're interested in sending and receiving
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_carrier.msg import NavigationGoal
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from tf.transformations import euler_from_quaternion

import roslib; roslib.load_manifest('ardrone_tutorials')


# Other imports
# ???

# Some Constants
COMMAND_PERIOD = 100 #ms
TARGET_TOPIC_NAME = '/unknown_package/unknown_topic'
TARGET_TOPIC_TYPE = NavigationGoal

EST_POSE_TOPIC_NAME = '/unknown_package/unknown_topic'
EST_POSE_TOPIC_TYPE = Twist()

class ArdroneNav:
    """ Class for Ardrone navigation: getting target position (and estimated one) and
        return velocity command to the drone with PID control"""

    def __init__(self):
        """Init function (to be completed)"""

        ## Tf attributes
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.tf_ardrone = '/ardrone' # to be redefined
        self.tf_target = '/target' # to be redefined
        self.tf_world = '/world' # to be redefined


        ####################
        ## Ardrone topics ##
        ####################

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pub_command = rospy.Publisher('/cmd_vel', Twist)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.command_timer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.SendCommand)

        # rate = rospy.Rate(Hertz) à mettre dans le noeud

        #######################################
        ## Positions (target and estimated) ###
        #######################################

        self.sub_target_pos = rospy.Subscriber(TARGET_TOPIC_NAME, TARGET_TOPIC_TYPE,
                                             self.ReadTargetPos)

        self.sub_est_pos = rospy.Subscriber(EST_POSE_TOPIC_NAME, EST_POSE_TOPIC_TYPE,
                                          self.ReadEstPos)

        # Units will be meters, seconds, and radiants.

        ## velocities (linear and angular) of the drone at current time
        self.curr_vel = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}
        # Target and estimated position
        self.target_pose = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}
        self.est_pose = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        self.error = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        self.vel_constrain = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        self.mode = 0 # RELATIVE or ABSOLUTE for target

        ##########
        ## PID ###
        ##########

        # Differents weights for each composant
        self.p = {'position': [0.2, 0.2, 0.2], 'orientation': [0.2, 0.2, 0.2]}
        self.i = {'position': [0.3, 0.3, 0.3], 'orientation': [0.3, 0.3, 0.3]}
        self.d = {'position': [0.1, 0.1, 0.1], 'orientation': [0.1, 0.1, 0.1]}
        self.dt = {'position': [0.0001, 0.0001, 0.0001], 'orientation': [0.0001, 0.0001, 0.0001]}

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


    # def read_navdata(self, navdata):
    #     """Use Subscriber to read velocity data from ardrone msgs (to be completed)"""
    #     self.curr_vel['position'][0] = navdata.vx/1000. # mm/sec -> m/s
    #     self.curr_vel['position'][1] = navdata.vy/1000.
    #     self.curr_vel['position'][2] = navdata.vz/1000.
    #     self.curr_vel['orientation'][0] = 2*np.pi*(navdata.rotX/360.) # degree -> rad
    #     self.curr_vel['orientation'][1] = 2*np.pi*(navdata.rotY/360.)
    #     self.curr_vel['orientation'][2] = 2*np.pi*(navdata.rotZ/360.)

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
        """Get target position"""

        self.mode = msg_pose.mode
        if self.mode == NavigationGoal.ABSOLUTE:
            # get linear position
            self.target_pose['position'] = msg_pose.pose.position

            # get angular orientation (in Euler angle)
            quaternion = msg_pose.pose.orientation
            rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z,
                                              quaternion.w))
            self.target_pose['orientation'] = [rotation[0], rotation[1], rotation[2]] # in rad


        elif self.mode == NavigationGoal.RELATIVE:
            # get linear position
            self.target_pose['position'] = msg_pose.pose.position
            self.target_pose['position'] += self.est_pose['position']

            # get angular orientation (in Euler angle)
            quaternion = msg_pose.pose.orientation
            rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z,
                                              quaternion.w))

            self.target_pose['orientation'] = [rotation[0], rotation[1], rotation[2]] # in rad
            self.target_pose['orientation'] += self.est_pose['orientation']

            self.target_pose['orientation'] = [angle%(2*np.pi) for angle\
                                               in self.target_pose['orientation']]

        else:
            raise NotImplementedError



    def read_est_pose(self, msg_pose):
        """Get target position"""

        transform = self.tf_buffer.lookup_transform(self.tf_world, self.tf_ardrone,
                                                   rospy.Time(0))
        # Get position from transform
        self.est_pose.position = [transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z]

        # get rotation (in Euler angle)
        quaternion = transform.transform.rotation
        rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z,
                                          quaternion.w))
        self.est_pose.orientation = [rotation[0], rotation[1], rotation[2]]

    def update(self):
        """compute velocity command through PID"""

        self.errors['position'] = [pose_t - pose_e for (pose_t,
                                   pose_e) in zip(self.est_pose['position'],
                                   self.target_pose['position'])]

        self.errors['orientation'] = [(angle_t - angle_e)%(2*np.pi) for (angle_t,
                                   angle_e) in zip(self.est_pose['orientation'],
                                   self.target_pose['orientation'])]

        command = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}

        for id in range(len(self.pids['position'])):
            command['position'][id] = self.pids['position'][id].compute_command(\
                                                                        self.errors['position'][id])
        for id in range(len(self.pids['orientation'])):
            command['orientation'][id] = self.pids['orientation'][id].compute_command(\
                                                                    self.errors['orientation'][id])

        self.set_command(command)
        self.send_command()
