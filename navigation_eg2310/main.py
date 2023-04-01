import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
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

#set up MQTT, GPIO settings
BROKER_IP = '172.20.10.6'
GPIO.setmode(GPIO.BCM) # for microswitch
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #TODO: update channel this is for microswitch

#set up MQTT topics
DOCK_TOPIC = 'dock'
TABLE_TOPIC = 'table'

rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

class Main(Node):
    def __init__(self):
        super().__init__('main')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sleep_rate = 0.025
        self.rate = 10
        self.r = self.create_rate(self.rate)
        self.broker_address= self.declare_parameter("~broker_ip_address", '172.20.10.6').value
        self.DOCK_TOPIC = self.declare_parameter("~dock_pub_topic", 'dock').value
        self.TABLE_TOPIC = self.declare_parameter("~table_sub_topic", 'table').value
        self.mqttclient = mqtt.Client("ros2mqtt")
        self.mqttclient.connect(self.broker_address)
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_message
        self.mqttclient.loop_start()
        self.get_logger().info('relay_ros2_mqtt:: started...')
        self.get_logger().info(f'relay_ros2_mqtt:: broker_address = {self.broker_address}')
        self.get_logger().info(f'relay_ros2_mqtt:: DOCK_TOPIC = {self.DOCK_TOPIC}') #topic to publish to through mqtt
        self.get_logger().info(f'relay_ros2_mqtt:: TABLE_TOPIC = {self.TABLE_TOPIC}') # topic to subscribe to through mqtt

        self.create_subscription(Odometry, 'odom', self.newOdom, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)



    def on_connect(self, client, userdata, flags, rc):
        print('Connected with result code'  + str(rc))
        client.subscribe(self.TABLE_TOPIC)
    
    def on_message(self, client, userdata, msg):
        table = msg.payload.decode('utf-8')

        print('Message received\nTopic: ' + msg.topic + '\nMessage: ' + table)
        
        #while there is no can in the robot
        #while (GPIO.input(5) != GPIO.LOW/HIGH): #TODO: update channel number
         #   time.sleep(2)
        self.mqttclient.loop_stop()
        self.route(table)
        


        
    def route(self, table):
        table = table

        #configure the paths to take for each checkpoint
        if (table == '1'):
            self.get_logger().info('got 1')
            self.path = 706050201
        elif (table == '2' or table == '3'):
            self.path = 80706050201
        elif (table == '4'):
            self.path = 9080706050201
        elif (table == '5'):
            self.path = 4030201
        elif (table == '6'):
            self.path = 111009080706050201
            
        self.go_to()
    
    def euler_from_quaternion(self, x, y, z, w):
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




    # TODO: test if the following works. if yes, delete all above code
    def newOdom(self, msg):
        print('in odom callback')
        self.odom = msg
        global x
        global y
        global theta
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        rot_q = self.odom.pose.pose.orientation
        (roll, pitch, theta) = self.euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
        print(f'x: {rot_q.x} y: {rot_q.y}')
 
    def go_to(self):
        print('in self.go_to') 


        speed = Twist()
        goal = Point()
        path = self.path
        while  path != 0:
            print(f'{path}')
            # get checkpoint from the path
            checkpoint = path % 100

            #goal.x = waypoints[checkpoint][0]
            #goal.y = waypoints[checkpoint][1]
            goal.x = 2.0
            goal.y = 2.0

            while rclpy.ok():
                inc_x = goal.x -x
                inc_y = goal.y -y

                angle_to_goal = math.atan2(inc_y, inc_x)

                print(f'angle to turn: {theta}')

                if abs(angle_to_goal - theta) > 0.1:
                    print('turning')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.1
                else:
                    print('move forward')
                    speed.linear.x = 0.2
                    speed.angular.z = 0.0

            
                self.publisher_.publish(speed)
                time.sleep(self.sleep_rate)
<<<<<<< HEAD
=======
                time.sleep(self.sleep_rate)

>>>>>>> 21e3770feff50464e48034bbf699c1ec22172ff9
                time.sleep(0.1)
            path = (path // 100)
            #self.get_logger().info(f'reached checkpoint {checkpoint}')
            self.get_logger().info(f'reached a checkpoint')

        self.get_logger().info(f'finished traversing all checkpoints')

        self.dock_to_table()
    
    def dock_to_table(self): #TODO: import rotatebot and check the turning angle
        if (self.table == 2):
            self.rotatebot(-math.pi / 2.0)
        elif (self.table == 3 | self.table == 4):
            self.rotatebot(math.pi / 2.0)
        



def main(args=None):

    rclpy.init(args=args)
    try:
        start = Main()
        rclpy.spin(start)
    except rclpy.exceptions.ROSInterruptException:
        pass

    start.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


