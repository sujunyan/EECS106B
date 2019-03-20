#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import numpy as np
import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from std_msgs.msg import Empty as EmptyMsg

class BicycleConverter():
    """docstring for BicycleConverter"""

    def get_params(self):
        if not rospy.has_param("~converter/turtlesim"):
            raise ValueError("Converter turtlesim flag not found on parameter server")    
        self.turtlesim = rospy.get_param("~converter/turtlesim")

        if not rospy.has_param("~converter/length"):
            raise ValueError("Converter length not found on parameter server")    
        self.length = rospy.get_param("~converter/length")

        if not rospy.has_param("~converter/turtlebot_command_topic"):
            raise ValueError("Converter input topic not found on parameter server")    
        self.turtlebot_command_topic = rospy.get_param("~converter/turtlebot_command_topic")

        if not rospy.has_param("~converter/turtlesim_command_topic"):
            raise ValueError("Converter input topic not found on parameter server")    
        self.turtlesim_command_topic = rospy.get_param("~converter/turtlesim_command_topic")

        if not rospy.has_param("~converter/turtlesim_pose_topic"):
            raise ValueError("Converter output topic not found on parameter server")
        self.turtlesim_pose_topic = rospy.get_param("~converter/turtlesim_pose_topic")

        if not rospy.has_param("~converter/fixed_frame"):
            raise ValueError("Converter output topic not found on parameter server")
        self.fixed_frame = rospy.get_param("~converter/fixed_frame")

        if not rospy.has_param("~converter/robot_frame"):
            raise ValueError("Converter output topic not found on parameter server")
        self.robot_frame = rospy.get_param("~converter/robot_frame")

        if not rospy.has_param("~converter/state_topic"):
            raise ValueError("Converter output topic not found on parameter server")
        self.state_topic = rospy.get_param("~converter/state_topic")

        if not rospy.has_param("~converter/bicycle_command_topic"):
            raise ValueError("Converter output topic not found on parameter server")
        self.bicycle_command_topic = rospy.get_param("~converter/bicycle_command_topic")


        if not rospy.has_param("~converter/max_steering_angle"):
            raise ValueError("Max Steering Angle not found on parameter server")
        self.max_steering_angle = rospy.get_param("~converter/max_steering_angle")

        if not rospy.has_param("~converter/max_steering_rate"):
            raise ValueError("Max Steering Rate not found on parameter server")
        self.max_steering_rate = rospy.get_param("~converter/max_steering_rate")

        if not rospy.has_param("~converter/max_linear_velocity"):
            raise ValueError("Max Linear Velocity not found on parameter server")
        self.max_linear_velocity = rospy.get_param("~converter/max_linear_velocity")


    def __init__(self):
        self._name = rospy.get_name()
        self.last_time = rospy.Time.now()
        self.rate_hz = 200
        self.rate = rospy.Rate(self.rate_hz)
        self.get_params()

        self.state = BicycleStateMsg()
        self.command = BicycleCommandMsg()
        if self.turtlesim:
            self.turtlesim_subscriber = rospy.Subscriber(self.turtlesim_pose_topic, Pose, self.update_turtlesim_pose)
            self.unicycle_command_topic = self.turtlesim_command_topic

            # resetting stuff
            print 'Waiting for turtlesim/reset service ...',
            rospy.wait_for_service('reset')
            print 'found!'
            self.reset_turtlesim_service = rospy.ServiceProxy('reset', EmptySrv)
        else:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.unicycle_command_topic = self.turtlebot_command_topic

            # resetting stuff
            reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', EmptyMsg, queue_size=10)

        self.command_publisher = rospy.Publisher(self.unicycle_command_topic, Twist, queue_size = 1)
        self.state_publisher = rospy.Publisher(self.state_topic, BicycleStateMsg, queue_size = 1)
        self.subscriber = rospy.Subscriber(self.bicycle_command_topic, BicycleCommandMsg, self.command_listener)
        rospy.Service('converter/reset', EmptySrv, self.reset)

    def command_listener(self, msg):
        msg.steering_rate = max(min(msg.steering_rate, self.max_steering_rate), -self.max_steering_rate)
        msg.linear_velocity = max(min(msg.steering_rate, self.max_linear_velocity), -self.max_linear_velocity)

        self.command = msg
        self.last_time = rospy.Time.now() # Save the time of last command for safety

    def update_turtlesim_pose(self, msg):
        self.state.x = msg.x
        self.state.y = msg.y
        self.state.theta = msg.theta

    def run(self):
        while not rospy.is_shutdown():

            # If we aren't using turtlesim, get the state
            if not self.turtlesim:
                for i in range(10):
                    try:
                        pose = self.tf_buffer.lookup_transform(
                            self.fixed_frame, self.robot_frame, rospy.Time())
                        break
                    except (tf2_ros.LookupException,
                            tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException):
                        pass
                if i == 9:
                    rospy.logerr("%s: Could not extract pose from TF. Using last-known transform", self._name)
                self.state.x = pose.transform.translation.x
                self.state.y = pose.transform.translation.y
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [pose.transform.rotation.x, pose.transform.rotation.y,
                     pose.transform.rotation.z, pose.transform.rotation.w])
                self.state.theta = yaw

            # Now publish the state
            self.state_publisher.publish(self.state)

            # Now execute commands
            # Timeout to ensure that only recent commands are executed
            if (rospy.Time.now() - self.last_time).to_sec() > 1.0:
                self.command.steering_rate = 0
                self.command.linear_velocity = 0

            # We output velocity and yaw rate based on the bicycle model
            output = Twist()
            output.linear.x = self.command.linear_velocity
            output.angular.z = (1/self.length)*np.tan(self.state.phi)*self.command.linear_velocity

            self.state.phi = self.state.phi + self.command.steering_rate/self.rate_hz
            self.state.phi = max(min(self.state.phi, self.max_steering_angle), -self.max_steering_angle)

            self.command_publisher.publish(output)

            self.rate.sleep()

    def reset(self, req):
        if self.turtlesim:
            self.reset_turtlesim_service()
        else:
            self.reset_odom.publish(EmptyMsg())
        self.state = BicycleStateMsg()
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node("Bicycle Conversion", anonymous=True)
    rospy.loginfo("To Stop Turtlebot hit Ctrl-C")

    converter = BicycleConverter()
    converter.run()