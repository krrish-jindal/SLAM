#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yaml
from tf_transformations import euler_from_quaternion
import os
from launch_ros.substitutions import FindPackageShare


class OdomSubscriber(Node):
	def __init__(self):
		super().__init__('odom_subscriber')
		self.odom_data = []
		package_name = 'my_nav2'
		config = "config/config_1.yaml"
		pkg_share = FindPackageShare(package=package_name).find(package_name)

		self.subscription = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			10)
		self.robot_pose = [0,0,0] 

		self.get_logger().info("Subscription created")
		self.config_path = os.path.join(pkg_share, config)


	def odom_callback(self, msg):
		self.robot_pose[0] = msg.pose.pose.position.x
		self.robot_pose[1] = msg.pose.pose.position.y
		quaternion_array = msg.pose.pose.orientation
		orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
		_, _, yaw = euler_from_quaternion(orientation_list)
		self.robot_pose[2] = yaw

		print (self.config_path)
		user_input = input("Enter any string to append current odom data (or 'exit' to quit): ")
		if user_input.strip():

			odom_dict = {
				'place': str(user_input),
				'pose': {
					'position': {'x': self.robot_pose[0], 'y': self.robot_pose[1]},
					'orientation': {'yaw': self.robot_pose[2]}
				}
			}
			
			self.odom_data.append(odom_dict)
			print(self.odom_data)
			self.get_logger().info(f"Saved odometry data for label '{user_input}'")
		else:
			pass


	def save_odom_data(self, file_path):
		existing_data = {}
		if os.path.exists(file_path):
			with open(file_path, 'r') as yaml_file:
				existing_data = yaml.safe_load(yaml_file)

		if 'odometry_data' not in existing_data:
			existing_data={'odometry_data': self.odom_data}
		else:
			existing_data['odometry_data'].extend(self.odom_data)

		with open(file_path, 'w') as yaml_file:
			yaml.dump(existing_data, yaml_file, default_flow_style=False)
			
def main(args=None):
	rclpy.init(args=args)

	odom_subscriber = OdomSubscriber()

	try:
		rclpy.spin(odom_subscriber)
	except KeyboardInterrupt:
		pass

	odom_subscriber.save_odom_data(odom_subscriber.config_path)

	odom_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
