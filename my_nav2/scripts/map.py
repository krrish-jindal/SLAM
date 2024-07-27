#!/usr/bin/env python3

import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import yaml
from launch_ros.substitutions import FindPackageShare
import os
import matplotlib.pyplot as plt
import math
import numpy as np
from matplotlib import colors
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class Mapping(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription_1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10)
        self.subscription_2 = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)
        self.publisher_2 = self.create_publisher(
            MapMetaData,
            '/map_metadata',
            10)
        self.current_x = 0
        self.current_y = 0
        self.center=0
        self.map = MapMetaData()
        self.occupancy_grid = OccupancyGrid()

        self.width =  300
        self.height = 400

        self.center_x = (self.width / 2)
        self.center_y = (self.height / 2)

        self.occupancy_grid.header=Header()
        self.occupancy_grid.info.resolution=0.05


        self.occupancy_grid.header.frame_id="odom"
        self.occupancy_grid.info.width=self.width
        self.occupancy_grid.info.height=self.height
        self.occupancy_grid.info.origin.position.x= -float((self.width*0.05)/2)
        self.occupancy_grid.info.origin.position.y= -float((self.height*0.05)/2)
        self.map.width=self.width
        self.map.height=self.height
        self.map.origin.position.x= -float((self.width*0.05)//2)
        self.map.origin.position.y= -float((self.height*0.05)//2)

        self.occupancy_grid.data =[-1]*(self.width*self.height)

    def odom_callback(self, msg):
        # self.pose_msg=msg.data
        self.pose=msg.pose.pose.position
        self.orientation=msg.pose.pose.orientation

        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y 

    def scan_cb(self, msg):
        laser_data = msg.ranges
        res = msg.angle_increment


        for i, range_val_m in enumerate((laser_data)):
            range_val = range_val_m *20
            angle = (i * res)+3.14

            if math.isinf(range_val):
                continue
            else:

                time=self.get_clock().now().to_msg()
                x = (round((range_val * np.cos(angle))) + self.center_x +(self.current_x))  
                y = (round((range_val * np.sin(angle))) + self.center_y + (self.current_y)) 
                self.occupancy_grid.header.stamp = time
                self.map.map_load_time=time

                index = int(y * self.width + x)
                self.occupancy_grid.data[index]=100
                self.publisher.publish(self.occupancy_grid)
                self.publisher_2.publish(self.map)







def main(args=None):
    rclpy.init(args=args)

    mapping_sub = Mapping()

    try:
        rclpy.spin(mapping_sub)
    except KeyboardInterrupt:
        pass

    mapping_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
