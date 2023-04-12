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
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

import numpy as np
import math
import cmath
import time
import pickle
with open("waypoints.pickle", "rb") as handle:
    waypoints = pickle.load(handle)
print (waypoints)

#set up MQTT, GPIO settings
BROKER_IP = '172.20.10.6'
GPIO.setmode(GPIO.BCM) # for microswitch
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
#TODO: update channel this is for microswitch

# constants
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

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        self.x = 0.0
        self.y = 0.0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.sleeprate = 0.25
        self.rate = 10
        self.rc = self.create_rate(self.rate)
        self.r = self.create_rate(self.rate)
        self.broker_address= self.declare_parameter("~broker_ip_address", '172.20.10.6').value
        self.DOCK_TOPIC = self.declare_parameter("~dock_pub_topic", 'dock').value
        self.TABLE_TOPIC = self.declare_parameter("~table_sub_topic", 'table').value
        self.mqttclient = mqtt.Client("ros2mqtt")
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_message
        self.get_logger().info('relay_ros2_mqtt:: started...')
        self.get_logger().info(f'relay_ros2_mqtt:: broker_address = {self.broker_address}')
        self.get_logger().info(f'relay_ros2_mqtt:: DOCK_TOPIC = {self.DOCK_TOPIC}') #topic to publish to through mqtt
        self.get_logger().info(f'relay_ros2_mqtt:: TABLE_TOPIC = {self.TABLE_TOPIC}') # topic to subscribe to through mqtt

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def connect_to_mqtt(self):
        print('Connecting...')
        self.mqttclient.connect(self.broker_address)
        global run
        while True:
            self.mqttclient.loop()
        #self.mqttclient.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print('Connected with result code'  + str(rc))
        
        client.subscribe(self.TABLE_TOPIC)
            
    def on_message(self, client, userdata, msg):
        table = msg.payload.decode('utf-8')

        print('Message received\nTopic: ' + msg.topic + '\nMessage: ' + table)
        
        #while there is no can in the robot
        #while (GPIO.input(5) != GPIO.LOW/HIGH): #TODO: update channel number
         #   time.sleep(2)
        #self.mqttclient.loop_stop()
        self.route(table)
        self.mqttclient.loop_stop()
    
    def route(self, table):
        self.table = table
        self.get_logger().info(f'got {table}')

        #configure the paths to take for each checkpoint
        if (table == '1'):
            self.path = 706050201
        elif (table == '2' or table == '3'):
            self.path = 80706050201
        elif (table == '4'):
            self.path = 9080706050201
        elif (table == '5'):
            self.path = 4030201
        elif (table == '6'):
            self.path = 111009080706050201
        elif (table == '7'):
            self.path = 2           
=======
            
        self.mover()


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        print(rot_angle)     
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        ##c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        target_yaw2 = target_yaw - (2 * math.pi)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        ##c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        ##c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        time.sleep(1)
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        #c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while (abs(current_yaw - target_yaw) > 0.1): 
        ##while(c_change_dir * c_dir_diff > 0):
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
            self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        time.sleep(1)
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmin(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def pick_shortest_direction(self):
        if self.laser_range.size != 0:
            # use nanargmin as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmin(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


        


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def traverse_waypoints(self):
        print('in traverse_waypoints')
        path = self.path
        
        while path != 0:
            path, checkpoint = divmod(path, 100)
            print(f'path = {path}')

            print(f"[CURRENT CHECKPOINT]: {checkpoint}")
            #allow the callback functions to run
            rclpy.spin_once(self)
            x = self.x
            y = self.y

            goal_x = waypoints[checkpoint][0]
            goal_y = waypoints[checkpoint][1]  

            #inc_x = goal_x - x
            #inc_y = goal_y - y
            x_diff = goal_x - x
            y_diff = goal_y - y

            angle_to_goal = math.atan2(y_diff, x_diff)

            print(f"angle_to_goal = {angle_to_goal}")
            angle = angle_to_goal - math.degrees(self.yaw)
            self.get_logger().info(f'angle to turn: {angle}')
            self.rotatebot(angle)
            #self.rotatebot(math.degrees(angle_to_goal) - self.yaw)
            # take into account orientation of bot before turning
            #self.rotatebot(math.degrees(angle_to_goal + math.pi - self.yaw))
            # take into account orientation of bot before turning
            self.rotatebot(math.degrees(angle_to_goal + math.pi - self.yaw))
            print("finished rotating")  

            print('waiting..')
            time.sleep(4)
            print(f"[INITIAL] x_diff = {x_diff}; y_diff = {y_diff}")
            
            while (abs(x_diff) > 0.1 or abs(y_diff) > 0.1):
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                time.sleep(1)
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
                print(f"[MOVING] x_diff = {x_diff}; y_diff = {y_diff}")
                x_diff = goal_x - self.x 
                y_diff = goal_y - self.y

        self.dock_to_table()
    
    def dock_to_table(self):
        print('docking..')
        if (self.table == '2'):
            self.rotatebot(-math.pi / 2.0)
        elif (self.table == '3' or self.table == '4'):
            self.rotatebot(math.pi / 2.0)
        
        self.pick_shortest_direction()

    def dock_assist(self, stop_d):
        twist = Twist()

        #get distance from different directions 
        front = np.nan_to_num(self.laser_range[0], nan=3.5 ,posinf=3.5)
        #frontright = np.nan_to_num(self.laser_range[FRONT_RIGHT], nan=3.5 ,posinf=3.5)
        #frontleft = np.nan_to_num(self.laser_range[FRONT_LEFT], nan=3.5 ,posinf=3.5)
        frontfrontleft = np.nan_to_num(self.laser_range[FRONT_FRONT_LEFT], nan=3.5 ,posinf=3.5)
        left = np.nan_to_num(self.laser_range[90], nan=3.5 ,posinf=3.5)
        #right = np.nan_to_num(self.laser_range[270], nan=3.5 ,posinf=3.5)



    

            


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.traverse_waypoints()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        #self.pick_direction()
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.connect_to_mqtt()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
