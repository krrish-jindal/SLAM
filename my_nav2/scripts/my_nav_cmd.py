#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import yaml
import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker

class NavigationController(Node):

	def __init__(self):
		
		rclpy.init()  # Initialize rclpy here
		super().__init__('nav_dock')

		self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
		self.vel_msg = Twist()
		self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
		self.marker_array = MarkerArray()

		self.navigator = BasicNavigator()
		self.robot_pose = [0, 0]

	def nav_reach(self, goal):
		while not self.navigator.isTaskComplete():
			feedback = self.navigator.getFeedback()

			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				self.navigator.cancelTask()

		result = self.navigator.getResult()
		if result == TaskResult.SUCCEEDED:
			print(f'Goal {str(goal)} succeeded!')
		elif result == TaskResult.CANCELED:
			print('Goal was canceled!')
		elif result == TaskResult.FAILED:
			print('Goal failed!')
		else:
			print('Goal has an invalid return status!')

	def nav_theta(self,angle):		
		x,y,z,w=quaternion_from_euler(0,0,angle)
		return (x,y,z,w)
	
	def publish_marker(self, x, y, label):
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.type = Marker.TEXT_VIEW_FACING
		marker.action = Marker.ADD
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0.0
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.color.a = 1.0  # Fully opaque
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.text = label

	
	def main(self):
		package_name = 'my_nav2'
		config = "config/config_1.yaml"
		pkg_share = FindPackageShare(package=package_name).find(package_name)
		config_path = os.path.join(pkg_share, config)
		with open(config_path, 'r') as infp:
			pos_goal = infp.read()
		data_dict = yaml.safe_load(pos_goal)


		# for entry in data_dict['odometry_data']:
		# 	place = entry['place']
		# 	pose = entry['pose']
		# 	position_x = pose['position']['x']
		# 	position_y = pose['position']['y']
		# 	orientation_yaw = pose['orientation']['yaw']

		# 	print(f"Publishing marker for place: {place}")
		# 	self.publish_marker(position_x, position_y, place)  # Change this line

		# 	marker = self.publish_marker(position_x, position_y, place)
		# 	self.marker_array.markers.append(marker)
		# self.marker_pub.publish(self.marker_array.markers)


		while rclpy.ok():

			user_input = input("Enter any place: ")

			found_entry = None
			for entry in data_dict['odometry_data']:
				
				if user_input == entry['place']:
					found_entry = entry
					break
			
			if found_entry:

				orientation_yaw = found_entry['pose']['orientation']['yaw']
				position_x = found_entry['pose']['position']['x']
				position_y = found_entry['pose']['position']['y']
			
				print(f"Found place: {user_input}")
				print(f"Position: ({position_x}, {position_y})")
				self.navigator.waitUntilNav2Active()

				goal = PoseStamped()
				goal.header.frame_id = 'map'
				goal.header.stamp = self.navigator.get_clock().now().to_msg()
				goal.pose.position.x = position_x
				goal.pose.position.y = position_y
				goal.pose.orientation.x = self.nav_theta(orientation_yaw)[0]
				goal.pose.orientation.y = self.nav_theta(orientation_yaw)[1]
				goal.pose.orientation.z = self.nav_theta(orientation_yaw)[2]
				goal.pose.orientation.w = self.nav_theta(orientation_yaw)[3]
				self.navigator.goToPose(goal)
				self.nav_reach(goal)
			else:
				print(f"No entry found for place: {user_input}")


if __name__ == '__main__':
	nav_controller = NavigationController()
	nav_controller.main()
