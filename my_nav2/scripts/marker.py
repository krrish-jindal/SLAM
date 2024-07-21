#!/usr/bin/env python3

import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import yaml
from launch_ros.substitutions import FindPackageShare
import os


def read_marker_config():
	package_name = 'my_nav2'
	config = "config/config_1.yaml"
	pkg_share = FindPackageShare(package=package_name).find(package_name)
	config_path = os.path.join(pkg_share, config)
	with open(config_path, 'r') as stream:
		try:
			markers = yaml.safe_load(stream)['odometry_data']
			return markers
		except yaml.YAMLError as exc:
			print("Error reading marker config file:", exc)
			return []

def publish_markers(markers):
	marker_pub = node.create_publisher(Marker, '/visualization_marker', 10)
	rate = node.create_rate(10)  # Adjust as needed
	while rclpy.ok():
		for idx, marker_data in enumerate(markers):
			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = node.get_clock().now().to_msg()
			marker.id = idx
			marker.type = Marker.TEXT_VIEW_FACING
			marker.action = Marker.ADD
			marker.pose.position.x = marker_data['pose']['position']['x']
			marker.pose.position.y = marker_data['pose']['position']['y']
			marker.pose.position.z = 0.0
			marker.scale.x = 1.0
			marker.scale.y = 1.0
			marker.scale.z = 0.5
			marker.color.a = 1.0  # Fully opaque
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.text = marker_data['place']
			marker_pub.publish(marker)
			print(marker_data['place'])

def main(args=None):
	rclpy.init(args=args)
	global node
	node = rclpy.create_node('marker_publisher')

	try:
		markers = read_marker_config()
		publish_markers(markers)
	except Exception as e:
		print('Exception in marker publishing:', e)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
