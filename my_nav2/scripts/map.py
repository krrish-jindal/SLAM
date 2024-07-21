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
            '/odometry',
            self.odom_callback,
            10)
        self.current_x = 0
        self.current_y = 0
        self.center=0


    def odom_callback(self, msg):
        # self.pose_msg=msg.data
        self.current_x = msg.pose.pose.position.x * 10
        self.current_y = msg.pose.pose.position.y * 10

    def scan_cb(self, msg):
        self.fig, self.ax = plt.subplots()
        laser_data = msg.ranges
        res = msg.angle_increment
        x = 500
        y = 500
        mid = 500

        self.grid = np.zeros((x, y), dtype=np.int32)
        self.grid.fill(-1)
        self.center = int(mid // 2)




        for i, range_val_m in enumerate(laser_data):
            range_val = range_val_m * 10
            angle = (i * res)

            if math.isinf(range_val):
                continue
            else:
                x = int(round(range_val * np.cos(angle))) + self.center + int(self.current_x)
                y = int(round(range_val * np.sin(angle))) + self.center + int(self.current_y)
                
                self.grid[x, y] = 1
            print(x,y)
        
        self.im = self.ax.imshow(self.grid, cmap=plt.cm.viridis, aspect='equal', origin='lower', extent=(0, 500, 0, 500))

        plt.grid(True)
        plt.tight_layout()
        plt.draw()  
        plt.show()



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
