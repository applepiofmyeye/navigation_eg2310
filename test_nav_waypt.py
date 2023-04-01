import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
#import RPi.GPIO as GPIO
import numpy as np
import math
import cmath
import time
import pickle

rotatechange = 0.1
speedchange = 0.05
waypoints = pickle.load(open("waypoints.pickle", "rb"))

class Navigation(Node):

    global get_waypoint
    get_waypoint = lambda x: (waypoints.get(x)[0], waypoints.get(x)[1])

    def __init__(self):
        super().__init__('navigation')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.path_publisher = self.create_publisher(
            Path, 
            'path',
            0 
        )
        
        self.path = Path()
    
    ###############################################################
    # Method to get a series of checkpoints by a checkpoint code  #
    # @param cp                                                   #
    # @return array of tuples (x, y) coordinates                  #
    ###############################################################
    def get(self, cp):
        print('in get')
        path = Path()
        arr = []
        i = 0
        while cp != 0:
            # create a new pose for each waypoints
            pose = PoseStamped()
            num = cp % 100
            cp = (cp // 100)

            # set pose
            
            pose = get_waypoint(num)
            #pose.pose.position.y = get_waypoint(num)[1]
            self.get_logger().info("num is {}".format(num))

            # append pose to the poses of the path
            path.poses.append(pose)
            i += 1
        return path

    def odom_callback(self, msg):
        print('just in odom callback')
        curr_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info('in odom_callback')
        table_number = int(input('Please input table number: ')) # to be changed to gpio later
        if (table_number == 1):
            self.path = self.get(706050201)
        
        publish = input('publish?[y/n] ')
        if (publish == 'y'):
            self,path.header = msg.header
            self.path_publisher.publish(self.path)



        # wait for turtlebot to return to checkpoint 1. (dock)
        #while (curr_position != get_waypoint(1) & gpio_callback == True):
        #   time.sleep(2);

        # once docked, wait for  
        
    
    # def directions(self, table):
    #     twist = Twist()

    #     if (table == 1):
    #         direction = self.get(706050201) # returns an array of odom coords

def main(args=None):
    rclpy.init(args=args)
    try:
        print('starting..')
        nav = Navigation()
        rclpy.spin(nav)
    except KeyboardInterrupt:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
