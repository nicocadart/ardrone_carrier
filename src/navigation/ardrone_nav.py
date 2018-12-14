# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import tf

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist, Pose  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# Other imports
# ???

# Some Constants
COMMAND_PERIOD = 100 #ms
TARGET_TOPIC_NAME = '/unknown_package/unknown_topic'
TARGET_TOPIC_TYPE = Twist()

EST_POSE_TOPIC_NAME = '/unknown_package/unknown_topic'
EST_POSE_TOPIC_TYPE = Twist()

class ArdroneNav:
    """ Class for Ardrone navigation: getting target position (and estimated one) and
        return velocity command to the drone with PID control"""

    def __init__(self):
        """Init function (to be completed)"""

        ## Tf attributes
        self.tf_listener = tf.TransformListener()
        self.tf_ardrone = '/ardrone' # to be redefined
        self.tf_target = '/target' # to be redefined
        self.tf_world = '/world' # to be redefined


        ####################
        ## Ardrone topics ##
        ####################
        # Subscribe to the /ardrone/navdata topic, of message type navdata,
        ## and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReadNavdata)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.SendCommand)

        # rate = rospy.Rate(Hertz) à mettre dans le noeud

        #######################################
        ## Positions (target and estimated) ###
        #######################################

        self.subTargetPos = rospy.Subscriber(TARGET_TOPIC_NAME, TARGET_TOPIC_TYPE,
                                             self.ReadTargetPos)

        self.estPos = rospy.Subscriber(EST_POSE_TOPIC_NAME, EST_POSE_TOPIC_TYPE,
                                       self.ReadEstPos)


        ## velocities (linear and angular) of the drone at current time
        self.curr_vx, self.curr_vy, self.curr_vz = 0.0, 0.0, 0.0
        self.curr_rotX, self.curr_rotY, self.curr_rotZ = 0.0, 0.0, 0.0

        # Target and estimated position
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.est_x, self.est_y, self.est_z = 0.0, 0.0, 0.0
        ## TODO: add orientation: 1 variable or 3 ?


        ##########
        ## PID ###
        ##########

        # Differents weights for each composant
        self.K = 6*[0.01]
        self.P = 6*[0.02]
        self.I = 6*[0.03]

        self.integral_err_x, self.integral_err_y, self.integral_err_z = 0.0, 0.0, 0.0


    def ReadNavdata(self, navdata):
        """Use Subscriber to read velocity data from ardrone msgs (to be completed)"""
        ## TODO: check the navdata in the correct frame

        self.curr_vx = navdata.vx
        self.curr_vy = navdata.vy
        self.curr_vz = navdata.vz
        self.curr_rotX = navdata.rotX
        self.curr_rotY = navdata.rotY
        self.curr_rotZ = navdata.rotZ

    def SetCommand(self, vx=0, vy=0, vz=0, rotX=0, rotY=0, rotZ=0):
        """Define the command msg (Twist) to be published to the drone"""

        self.command.linear.x = vx
        self.command.linear.y = vy
        self.command.linear.z = vz
        self.command.angular.x = rotX
        self.command.angular.y = rotY
        self.command.angular.z = rotZ

    def SendCommand(self):
        """publish the command twist msg to cmd_vel/"""
        self.pubCommand.publish(self.command)

    def ReadTargetPos(self, msg_pose):
        """Get target position"""

        self.target_x = msg_pose.x
        self.target_y = msg_pose.y
        self.target_z = msg_pose.z

    def ReadEstPos(self, msg_pose):
        """Get target position"""

        self.est_x = msg_pose.x
        self.est_y = msg_pose.y
        self.est_z = msg_pose.z

    def update(self):
        """compute velocity command through PID"""

        self.error_x = self.target_x - self.est_x
        self.error_y = self.target_y - self.est_y
        self.error_z = self.target_z - self.est_z

        # If we use theta (or angle), modulo pi for error

        # Check for constraints (v< max_v_allowed)
