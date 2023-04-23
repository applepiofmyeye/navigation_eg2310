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
from geometry_msgs.msg import Twist, Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import pickle

# constants
num_of_waypoints = 12
waypoints = {}

# initialising an array with number of waypoints as predefined
for i in range(1, num_of_waypoints + 1):
    waypoints[i].extend([])

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
    
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.px = 0.0
        self.py = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        
        # create subscription to track orientation
        self.m2b_subscription = self.create_subscription(
            Pose,
            'map2base',
            self.m2b_callback,
            10)
        self.m2b_subscription  # prevent unused variable warning
        

    def m2b_callback(self, msg):
        self.px = msg.position.x
        self.py = msg.position.y
        orien = msg.orientation
        self.ox, self.oy, self.oz = euler_from_quaternion(orien.x, orien.y, orien.z, orien.w)

    
    def get_waypoints(self):
        n = num_of_waypoints
        while n != 0:
            inp = input("Enter input: ")
            if inp == "w":
                checkpt_id = int(input("Enter checkpoint: "))
                print("saving..")
                rclpy.spin_once(self)
            
                data = [self.px, self.py, self.ox, self.oy, self.oz]
                waypoints[checkpt_id].extend(data)
                print(waypoints)
                n -= 1
            elif inp == "s":
                print("saving...")
                with open('waypoints.pickle', 'wb') as handle:
                    pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)
                n -= 1
            else:
                print("Please enter 's' or 'w' only.")

def main(args=None):
    rclpy.init(args=args)
    try:
        waypoint = Waypoint()
        start = input("Press s to start: ")
        if start == "s":
            waypoint.get_waypoints()
    except KeyboardInterrupt:
        waypoint.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()