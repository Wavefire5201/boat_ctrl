#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from boat_interfaces.msg import AiOutput
from boat_interfaces.msg import BuoyCoordinates


class locate_buoys(Node):
    def __init__(self):
        super().__init__("locate_buoys")
        self.get_logger().info("Task Two: its working")
        
        #subscribes to AI Output
        self.AiOutput_subscriber = self.create_subscription(
            AiOutput, "/BuoyCoordinates", self.buoy_coordinates_callback, 10)
        
    self.latitudes = []
    self.longitudes = []
    self.types = []
    def buoy_coordinates_callback(self, BuoyCoordinatesOutput):
        self.latitudes = BuoyCoordinatesOutput.latitudes
        self.longitudes = BuoyCoordinatesOutput.longitudes
        self.types = BuoyCoordinatesOutput.types
        taskTwo(self.latitudes, self.longitudes, self.types)
        

def taskTwo(latitudes, longitudes, types):
    #find the closest green buoy to every red buoy
    #compute the midpoint between the buoy pairs
    #also find the midpoint between the closest yellow or black buoy and the first green and red buoy
    #find closeset midpoint, navigate to it
    pass

def main(args=None):
    rclpy.init(args=args)
    node = locate_buoys()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()