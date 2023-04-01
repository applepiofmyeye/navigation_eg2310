# This node tests the route node.



import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from route import Route

import numpy as np
import math
import cmath
import time
import pickle

#set up MQTT, GPIO settings
#GPIO.setmode(GPIO.BCM) # for microswitch
#GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #TODO: update channel this is for microswitch


broker_ip = '172.20.10.6'
DOCK_TOPIC = 'dock'
TABLE_TOPIC = 'table'
global table
table = '-1'
msg_received = False

mqttclient = mqtt.Client("ros2mqtt")
mqttclient.connect(broker_ip)
def on_connect(client, userdata, flags, rc):
    print('Connected with result code'  + str(rc))
    client.subscribe(TABLE_TOPIC)
    
def on_message(client, userdata, msg):
    table = msg.payload.decode('utf-8')
    print('Message received\nTopic: ' + msg.topic + '\nMessage: ' + table)
    msg_received = True 
    mqttclient.loop_stop() 
    

def main(args=None):
    mqttclient.on_connect = on_connect
    mqttclient.on_message = on_message
    mqttclient.loop_start()

    rclpy.init(args=args)
    try:
            route = Route(table)
            rclpy.spin(route)
    except rclpy.exceptions.ROSInterruptException:
        pass
    
    route.delete_node()
    rclpy.shutdown()     
    


   
if __name__ == '__main__':
    main()

