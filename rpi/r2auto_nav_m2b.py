# Copyright 2016 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
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

# print waypoints collected from get_waypoints_m2b
with open("waypoints.pickle", "rb") as handle:
    waypoints = pickle.load(handle)
print(waypoints)

# set up MQTT, GPIO settings
BROKER_IP = '172.20.10.6'
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # microswitch
left_ir = 17
right_ir = 27
GPIO.setup(left_ir, GPIO.IN)  # infrared sensor 1
GPIO.setup(right_ir, GPIO.IN)  # infrared sensor 2

# constants
rotatechange = 0.8
speedchange = 0.18
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle+1, 1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
speedchange_lf = 0.05
rotatechange_lf = 0.5

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

    return roll_x, pitch_y, yaw_z  # in radians

# class for AutoNav ROS node


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.x = 0.0
        self.y = 0.0

        # set the initial state of the TurtleBot going back to the dispenser to be False
        self.going_back = False

        # set MQTT broker, and MQTT topics
        self.broker_address = self.declare_parameter(
            "~broker_ip_address", '172.20.10.6').value
        self.DOCK_TOPIC = self.declare_parameter(
            "~dock_pub_topic", 'dock').value
        self.TABLE_TOPIC = self.declare_parameter(
            "~table_sub_topic", 'table').value

        # instantiate the MQTTClient object
        self.mqttclient = mqtt.Client("ros2mqtt")

        # set callback functions for when:
        # the RPi is connected to the MQTT broker;
        self.mqttclient.on_connect = self.on_connect
        # the RPi receives a message from the MQTT broker.
        self.mqttclient.on_message = self.on_message

        # print statements to verify the MQTT brokers, and topics used
        self.get_logger().info('relay_ros2_mqtt:: started...')
        self.get_logger().info(
            f'relay_ros2_mqtt:: broker_address = {self.broker_address}')
        self.get_logger().info(
            f'relay_ros2_mqtt:: DOCK_TOPIC = {self.DOCK_TOPIC}')
        self.get_logger().info(
            f'relay_ros2_mqtt:: TABLE_TOPIC = {self.TABLE_TOPIC}')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # create subscription to get map2base data
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.m2b_callback,
            10
        )

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

    # method to connect the RPi to the MQTT broker

    def connect_to_mqtt(self):
        print('Connecting...')
        self.mqttclient.connect(self.broker_address)
        global run
        while True:
            self.mqttclient.loop()

    # callback function when RPi is connected to MQTT broker
    def on_connect(self, client, userdata, flags, rc):
        print('Connected with result code' + str(rc))

        # RPi should subscripe to the table topic,
        # and will call the AutoNav::on_message method when it receives a message
        client.subscribe(self.TABLE_TOPIC)

    # callback function when RPi receives a message from the MQTT broker (table topic)
    def on_message(self, client, userdata, msg):
        table = msg.payload.decode('utf-8')
        print('Message received\nTopic: ' + msg.topic + '\nMessage: ' + table)

        # call the route method to get the path to the table
        self.route(table)

        # stop the MQTT broker, in order to subscribe to spin AutoNav node
        self.mqttclient.loop_stop()

    # method to configure the paths to take for each checkpoint
    # refer to documentation (Section 3.2 Autonomous Navigation) for the location of each checkpoint
    def route(self, table):
        self.table = table
        self.get_logger().info(f'got {table}')
        """
            Note: the order in which the waypoint numbers are given in this route is in the reverse order, 
            (e.g. 706 would go from waypoint 6 then 7, 50403 would go from 3 to 4 to 5)
        """
        if (table == '1'):
            self.path = 6
        elif (table == '2'):
            self.path = 706
        elif (table == '3'):
            self.path = 2
        elif (table == '4'):
            self.path = 3
        elif (table == '5'):
            self.path = 504
        elif (table == '6'):
            self.path = 10090806
        self.mover()

    # callback function when a map2base message is received
    def m2b_callback(self, msg):
        position = msg.position
        self.x = position.x
        self.y = position.y
        orientation_quat = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    # callback function when an occupancy message is received,
    # meant to plot the occupancy grid surrounding the LiDAR sensor,
    # no longer needed.
    def occ_callback(self, msg):
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info(
        # 'Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)
        # )

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)

    # callback function when a scan message is received

    def scan_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)

        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

    # method to rotate the bot a specified angle

    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)

        print(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
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

        # if the difference between the current and target yaw is > 0.15, continue turning,
        while abs(current_yaw - target_yaw) > 0.15:
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # if the difference between the current and target yaw < 1.4, slow down
            if abs(current_yaw - target_yaw) < 1.4:
                rclpy.spin_once(self)
                twist.linear.x = 0.0
                twist.angular.z = c_change_dir * rotatechange * 0.2
                self.publisher_.publish(twist)

        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    # method to pick the shortest direction
    def pick_shortest_direction(self):
        if self.laser_range.size != 0:
            # use nanargmin as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmin(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' %
                                   (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        if lr2i > 180:
            lr2i -= 360
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')

        # move in direction until meet stop distance
        self.pick_direction(0)
        self.stopbot()

    # method to pick a specific direction to scan for shortest direction
    def pick_direction(self, direction):
        print('in pick_direction')
        if direction == 0:
            angle = front_angles
        elif direction == 1:
            angle = range(60, 120, 1)
        else:
            angle = range(-120, -60, 1)
        lri = np.nanargmin(self.laser_range[angle])
        if direction != 0:
            # self.laser range is an array where each index i contains a the shortest distance at degree i
            # hence, an adjustment is needed to recalibrate the indexes to their initial index values
            adjustment = direction * 60.0
        else:
            adjustment = 0.0

        # rotate the bot in the direction of the shortest distance in specified direction
        self.rotatebot(float(lri) + adjustment)

        # move the bot forward
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

        # while it is moving forward..
        while rclpy.ok():
            if self.laser_range[angle].size != 0:
                rclpy.spin_once(self)
                # get the shortest distance 360 degrees around the bot
                lri = np.nanargmin(self.laser_range[angle])

                # if the shortest distance is lesser than predefined stop_distance,
                if self.laser_range[lri] < stop_distance:
                    self.get_logger().info('stopping')
                    # then we stop the bot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    break

                # else if the shortest distance is less than 2 times of the stop_distance,
                elif self.laser_range[lri] < 2 * stop_distance:
                    # slow down the bot
                    twist.linear.x = twist.linear.x * 0.3
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)

                rclpy.spin_once(self)

        rclpy.spin_once(self)

    # method to stop the bot
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    # main method to traverse from one waypoint to another
    def traverse_waypoints(self):
        print('In traverse_waypoints')
        path = self.path

        while path != 0:
            # update path to be the remaining checkpoints
            # and current checkpoint to be the mod of path % 100
            path, checkpoint = divmod(path, 100)

            print(f"[CURRENT CHECKPOINT]: {checkpoint}")

            # allow the callback functions to run
            rclpy.spin_once(self)
            time.sleep(3)
            # we found that spinning twice gave more reliable results
            rclpy.spin_once(self)

            goal_x = waypoints[checkpoint][0]
            goal_y = waypoints[checkpoint][1]
            print(f'goal_x: {goal_x}; goal_y: {goal_y}')
            x_diff = goal_x - self.x
            y_diff = goal_y - self.y

            # tolerance of how close the bot should be before it stops
            tolerance = 0.18
            # distance at which the bot should slow down (for greater accuracy)
            slow_down_dist = 0.5

            while (abs(x_diff) > tolerance or abs(y_diff) > tolerance):
                # calculate the angle to goal position
                angle_to_goal = math.atan2(y_diff, x_diff)
                # rotate the bot by this angle
                self.rotatebot(math.degrees(angle_to_goal - self.yaw))

                # if the x_diff is larger, we will track the x_diff
                if abs(x_diff) > abs(y_diff):
                    while abs(x_diff) > tolerance:
                        # move forward by speed: speedchange
                        twist = Twist()
                        twist.linear.x = speedchange

                        # if x_diff is lesser than slow_down dist, slow down.
                        if abs(x_diff) < slow_down_dist:
                            twist.linear.x = twist.linear.x * 0.2
                        twist.angular.z = 0.0
                        time.sleep(1)
                        self.publisher_.publish(twist)

                        rclpy.spin_once(self)
                        rclpy.spin_once(self)

                        x_diff = goal_x - self.x
                        y_diff = goal_y - self.y

                else:

                    while abs(y_diff) > tolerance:
                        # move forward by speed change
                        twist = Twist()
                        twist.linear.x = speedchange

                        # if x_diff is lesser than slow_down dist, slow down.
                        if abs(y_diff) < slow_down_dist:
                            twist.linear.x = twist.linear.x * 0.2
                        twist.angular.z = 0.0
                        time.sleep(1)
                        self.publisher_.publish(twist)
                        # print(f'[BEFORE SPIN] x_diff = {x_diff}; y_diff = {y_diff}')

                        rclpy.spin_once(self)
                        rclpy.spin_once(self)

                        x_diff = goal_x - self.x
                        y_diff = goal_y - self.y

                        # print(f"[MOVING] x_diff = {x_diff}; y_diff = {y_diff}")

                # update the x_diff and y_diff at the final position,
                # to check if the x_diff and y_diff is lesser then the tolerance
                # if it is, stop the bot. else, self-correct
                self.stopbot()
                x_diff = goal_x - self.x
                y_diff = goal_y - self.y

            self.stopbot()

        # call the docking method based on whether the bot is going back or not.
        if not self.going_back:
            self.dock_to_table()
        else:
            self.dock_to_dispenser()

    # method to navigate to table 6
    def go_to_table_6(self):
        rclpy.spin_once(self)
        time.sleep(1)

        if self.laser_range.size != 0:
            # use nanargmin as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmin(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' %
                                   (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        if lr2i > 180:
            lr2i -= 360

        goal_x = self.x + 0.5
        x_diff = goal_x - self.x
        while x_diff > 0.1 and self.laser_range[lr2i] > 0.5:
            twist = Twist()
            twist.linear.x = speedchange
            if x_diff < 0.5:
                twist.linear.x = twist.linear.x * 0.3
            twist.angular.z = 0.0
            # time.sleep(1)
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
            # rclpy.spin_once(self)
            x_diff = goal_x - self.x
            lr2i = np.nanargmin(self.laser_range)
        self.stopbot()
        self.pick_shortest_direction()

    # dock the bot to the table
    def dock_to_table(self):
        print('docking..')
        rclpy.spin_once(self)
        time.sleep(1)
        while not bool(GPIO.input(21)):
            time.sleep(0.001)
        if (self.table == 1 or 3 or 5):
            self.pick_direction(0)
        elif (self.table == 4):
            self.pick_direction(1)
        elif (self.table == 6):
            self.go_to_table_6()

        self.return_home()

    # main function to return back to the dispenser
    def return_home(self):
        self.going_back = True
        self.path = self.path * 100 + 1
        path_string = str(self.path)
        path_string = path_string[::-1]
        self.path = int(path_string)
        self.traverse_waypoints()
        self.line_following(1)
        self.connect_to_mqtt()

    # when docking back to dispenser, publish to the console that it is home, release soda can
    def dock_to_dispenser(self):

        rclpy.spin_once(self)
        self.going_back = False
        self.mqttclient.loop_start()
        self.mqttclient.publish(self.DOCK_TOPIC, "Home", qos=0, retain=False)
        self.mqttclient.loop_stop()

    # follow line
    def line_following(self, sign):
        speedchange_lf = sign * 0.05
        rotatechange_lf = sign * 0.1
        while True:
            left_detect = GPIO.input(17)
            right_detect = GPIO.input(27)
            twist = Twist()
            if left_detect == 0 and right_detect == 0:
                twist.linear.x = speedchange_lf
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            elif left_detect == 1 and right_detect == 0:
                twist.linear.x = sign * 0.01
                twist.angular.z = rotatechange_lf
                self.publisher_.publish(twist)
            elif left_detect == 0 and right_detect == 1:
                twist.linear.x = sign * 0.01
                twist.angular.z = (-1) * rotatechange_lf
                self.publisher_.publish(twist)
            elif left_detect == 1 and right_detect == 1:
                if sign == 1 and -2.0 < math.degrees(self.yaw) < 2.0:
                    break
                elif sign == -1:
                    break
        print("reached destination, stopping.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    # waits for soda can to load to traverse waypoints
    def mover(self):
        try:
            while bool(GPIO.input(21)):
                print(f'Can not loaded')
                time.sleep(0.001)
            twist = Twist()
            twist.linear.x = -0.2
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(2)
            self.stopbot()
            self.line_following(-1)  # follow line backwards
            self.traverse_waypoints()

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    rclpy.spin_once(auto_nav)
    auto_nav.connect_to_mqtt()

    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
