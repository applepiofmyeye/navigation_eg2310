import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import cmath
import time
import pickle

#constants
waypoints = pickle.load(open("waypoints.p", "rb"))

class Navigation(Node):

    def __init__(self):
        super().__init__('navigation')
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
