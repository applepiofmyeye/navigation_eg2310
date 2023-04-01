#!/usr/bin/env python3
# this gets the twist object and publishes to mqtt

import rclpy
from rclpy.time import Time

from time import sleep
import sys
import threading
import numpy as np
import os
import paho.mqtt.client as mqtt
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
import math
import cmath
import time

# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
# from std_msgs.msg import Int32, Float32


import json

rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians



class RelayRos2Mqtt(Node):
    def __init__(self):
        super().__init__('relay_ros2_mqtt')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sleep_rate = 0.025
        self.rate = 10
        self.r = self.create_rate(self.rate)
        self.broker_address= self.declare_parameter("~broker_ip_address", '172.20.10.6').value
        self.MQTT_PUB_TOPIC = self.declare_parameter("~mqtt_pub_topic", 'esp32/cmd_vel').value
        self.ROS_TWIST_SUB_TOPIC = self.declare_parameter("~twist_sub_topic", '/cmd_vel').value
        self.DOCK_TOPIC = self.declare_parameter("~dock_pub_topic", 'dock').value
        self.mqttclient = mqtt.Client("ros2mqtt") 
        self.mqttclient.connect(self.broker_address) 

        self.get_logger().info('relay_ros2_mqtt:: started...')
        self.get_logger().info(f'relay_ros2_mqtt:: broker_address = {self.broker_address}')
        self.get_logger().info(f'relay_ros2_mqtt:: MQTT_PUB_TOPIC = {self.MQTT_PUB_TOPIC}')
        self.get_logger().info(f'relay_ros2_mqtt:: ROS_TWIST_SUB_TOPIC = {self.ROS_TWIST_SUB_TOPIC}')
        self.get_logger().info(f'relay_ros2_mqtt:: DOCK_TOPIC = {self.DOCK_TOPIC}')

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.create_subscription(
            Twist,
            self.ROS_TWIST_SUB_TOPIC, # subscribe to twist topic 
            self.twist_callback, # publish to mqtt 
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            Odometry, 
            'odom',
            self.odom_callback,
            10
        )
            
        


    def twist_callback(self, tmsg):
        if tmsg.linear.x != 0 or tmsg.angular.z:
            Dictionary ={'x':str(tmsg.linear.x), 'z':str(tmsg.angular.z)}
            self.get_logger().info('dict:: {0}'.format(json.dumps(Dictionary).encode()))
            self.mqttclient.publish(self.MQTT_PUB_TOPIC,json.dumps(Dictionary).encode(),qos=0, retain=False)

    def odom_callback(self, omsg):
        self.get_logger().info('in odom_callback')
        orientation_quat =  omsg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        #odom_info = 'roll: %s\npitch: %s\nyaw: %s' % (roll, pitch, yaw)
        if (roll == 0):
            string = "Home"
        else:
            string = "Away"
        self.get_logger().info(string)
        self.mqttclient.publish(self.DOCK_TOPIC, string, qos=0, retain=False)
        self.get_logger().info('after publish')
        sleep(5)


    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


def main(args=None):
    
    rclpy.init(args=args)
    try:
        relay_ros2_mqtt = RelayRos2Mqtt()
        rclpy.spin(relay_ros2_mqtt)
    except rclpy.exceptions.ROSInterruptException:
        pass

    relay_ros2_mqtt.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

