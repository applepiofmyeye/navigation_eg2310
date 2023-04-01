# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import pickle

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
poses = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: [], 9: [], 10: [], 11: [], 12: []}

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
    
class GetPose(Node):

    def __init__(self):
        super().__init__('waypoint')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        #self.roll = 0
        #self.pitch = 0
        #self.yaw = 0

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        inp = input("Enter input: ")
        if inp == "w":
            checkpt_id = int(input("Enter checkpoint: "))
            print("saving..")
            #orien =  msg.pose.pose.orientation
            #px = msg.pose.pose.position.x
            #py = msg.pose.pose.position.y
            #ox, oy, oz = euler_from_quaternion(orien.x, orien.y, orien.z, orien.w)

            pose = PoseStamped()
            pose.pose = msg.pose.pose
            # self.get_logger().info(orien)
            #while numbers != 0:
                #num = numbers % 10
                #numbers = (numbers // 10)
            #data = (px, py, ox, oy, oz)
            
            poses[checkpt_id].append(pose)
            print('added waypoint')

        elif inp == "s":
            print("saving...")
            with open('poses.pickle', 'wb') as handle:
                pickle.dump(poses, handle, protocol=pickle.HIGHEST_PROTOCOL)

def main(args=None):
    rclpy.init(args=args)
    try:
        poses = GetPose()
        start = input("Press s to start: ")
        if start == "s":
            rclpy.spin(poses)

    except KeyboardInterrupt:
        poses.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

