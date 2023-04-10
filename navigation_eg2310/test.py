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
#
# adapted from https://github.com/Shashika007/teleop_twist_keyboard_ros2/blob/foxy/teleop_twist_keyboard_trio/teleop_keyboard.py

import time
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
GPIO.setup(27,GPIO.IN)


# constants
rotatechange = 0.1
speedchange = 0.07


class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist,'cmd_vel',10)

# function to read keyboard input
    def readKey(self):
        twist = Twist()
        try:
            while True:
                # get keyboard input
                cmd_char = str(input("Start (s); rotate left (a); rotate right (d): "))
        
                # check which key was entered
                if cmd_char == 's':
                    # stop moving
                    twist.linear.x = 0.08
                    twist.angular.z = 0.0
                elif cmd_char == 'a':
                    # turn counter-clockwise
                    twist.linear.x = 0.0
                    twist.angular.z += rotatechange
                elif cmd_char == 'd':
                    # turn clockwise
                    twist.linear.x = 0.0
                    twist.angular.z -= rotatechange

                # start the movement
                self.publisher_.publish(twist)
                self.line_following()
                
                
        except Exception as e:
            print(e)
            
  # Ctrl-c detected
        finally:
         # stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def line_following(self):
        while True:
            left_detect = GPIO.input(17)
            right_detect = GPIO.input(27)
            twist = Twist()
            if left_detect == 0 and right_detect == 0:
                twist.linear.x = 0.07
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            elif left_detect == 1 and right_detect == 0:
                twist.linear.x = 0.0
                twist.angular.z = 0.1
                self.publisher_.publish(twist)
            elif left_detect == 0 and right_detect == 1:
                twist.linear.x = 0.0
                twist.angular.z = (-1) * 0.1
                self.publisher_.publish(twist)
            elif left_detect == 1 and right_detect == 1:
                break
        print("reached destination, stopping.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    mover.readKey()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
