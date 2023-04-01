import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

import numpy as np
import math
import cmath
import time
import pickle



rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

#waypoints = pickle.load(open("waypoints.p", "rb"))


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

 


def rotatebot(self, rot_angle):
    self.get_logger().info('In rotatebot')
    twist = Twist()
    current_yaw = self.yaw
    self.get_logger().info('Current: %f' % math.degrees(current_yaw))
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    target_yaw = current_yaw + math.radians(rot_angle)
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
    c_change = c_target_yaw / c_yaw
    c_change_dir = np.sign(c_change.imag)
    twist.linear.x = 0.0
    twist.angular.z = c_change_dir * rotatechange
    self.publisher_.publish(twist)

    c_dir_diff = c_change_dir
    while(c_change_dir * c_dir_diff > 0):
        rclpy.spin_once(self)
        current_yaw = self.yaw
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        c_change = c_target_yaw / c_yaw
        c_dir_diff = np.sign(c_change.imag)

    self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
    twist.angular.z = 0.0
    self.publisher_.publish(twist)




class Route(Node):
    def __init__(self, table):
        super().__init__('route')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.table = table


        #configure the chec;kpoints
        if (table == '1'):
            print("11111111111111")
            self.path = 706050201
        elif (table == '2' or table == '3'):
            self.path = 80706050201
        elif (table == '4'):
            self.path = 9080706050201
        elif (table == '5'):
            self.path = 4030201
        elif (table == '6'):
            self.path = 111009080706050201
        
        #subscribers and publishers
        self.create_subscription(
            Odometry,
            'odom',
            self.newOdom,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
       # TODO: test if the following works. if yes, delete all above code
    def newOdom(self, msg):
        self.get_logger().info('In newOdom')
        global x
        global y
        global theta

        
        rot_q = msg.pose.pose.orientation
        print(f'y: {rot_q.y}, z: {rot_q.z}, w: {rot_q.w}')  
        (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)



        speed = Twist()
        goal = Point()
        path = self.path
        path = True
        while  path:
            print("otw")
            # get checkpoint from the path
            checkpoint = path % 100
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            #goal.x = waypoints[checkpoint][0]
            #goal.y = waypoints[checkpoint][1]
            goal.x = 2.0
            goal.y = 2.0

            while rclpy.ok():
                inc_x = goal.x -x
                inc_y = goal.y -y

                angle_to_goal = math.atan2(inc_y, inc_x)

                if abs(angle_to_goal - theta) > 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.3
                else:
                    speed.linear.x = 0.5
                    speed.angular.z = 0.0

            self.publisher_.publish(speed)
            time.sleep(5)
            path = False

            self.get_logger().info(f'reached checkpoint {checkpoint}')
        self.get_logger().info(f'finished traversing all checkpoints')

        self.dock_to_table()
    
    def dock_to_table(self): #TODO: import rotatebot and check the turning angle
        if (self.table == '2'):
            rotatebot(-math.pi / 2.0)
        elif (self.table == '3' | self.table == 4):
            rotatebot(math.pi / 2.0)
        







#def main(args=None):

#    rclpy.init(args=args)
#    try:
#        route = Route()
#        rclpy.spin(route)
#    except rclpy.exceptions.ROSInterruptException:
#        pass

#    route.destroy_node()
#    rclpy.shutdown()

#if name == '__main__':
#    main()

