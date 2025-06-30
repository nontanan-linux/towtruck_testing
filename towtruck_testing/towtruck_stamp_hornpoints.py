#!/usr/bin/env python3
import os
import time
import math
import datetime
import csv
from math import asin, atan2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tier4_debug_msgs.msg import Float32Stamped
from towtruck_msgs.msg import RobotState, HornCmd
from towtruck_map_image import StationOutline


class TowtruckStampHornPoints(Node):
	def __init__(self):
		super().__init__('towtruck_stamp_horn_points')
		# Declare parameters
		self.declare_parameter("ndt_score_topic", "/localization/pose_estimator/nearest_voxel_transformation_likelihood")
		self.declare_parameter("pose_twist_fusion_topic", "/localization/pose_twist_fusion_filter/pose")
		self.declare_parameter("kinematic_topic", "/localization/kinematic_state")
		self.declare_parameter("save_csv_path", "/home/nontanan/ros2_ws/src/towtruck_testing/csv")
		self.declare_parameter("csv_horn_path", "~/autoware.bg2/data/BG/HornPoints.csv")
		self.declare_parameter("robot_state_topic", "/robot_state")
		self.declare_parameter("horn_cmd_topic", "/horn_cmd")
		self.declare_parameter("debug_info", True)
		self.declare_parameter("update_endpoint", "internal_report/record_report")
		# Get parameter values
		self.ndt_score_topic = self.get_parameter("ndt_score_topic").get_parameter_value().string_value
		self.pose_twist_fusion_topic = self.get_parameter("pose_twist_fusion_topic").get_parameter_value().string_value
		self.kinematic_topic = self.get_parameter("kinematic_topic").get_parameter_value().string_value
		self.robot_state_topic = self.get_parameter("robot_state_topic").get_parameter_value().string_value
		self.horn_cmd_topic = self.get_parameter("horn_cmd_topic").get_parameter_value().string_value
		self.debug_info = self.get_parameter("debug_info").get_parameter_value().bool_value
		self.update_endpoint = self.get_parameter("update_endpoint").get_parameter_value().string_value
		# File paths
		self.csv_horn_path = os.path.expanduser(self.get_parameter("csv_horn_path").get_parameter_value().string_value)
		self.horn_save_file = os.path.join(
			self.get_parameter("save_csv_path").get_parameter_value().string_value,
			f'HornPointsStamp_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
		# Internal state
		self.points_outline = StationOutline()
		self.robot_pose = PoseStamped()
		self.robot_kinematic = Odometry()
		self.robot_state = RobotState()
		self.horn_cmd = HornCmd()
		self.ndt_score = Float32Stamped()
		self.hornpoints = []
		self.nearest_node = None
		# Load horn station reference data
		self.horn_station = self.read_horn_station_csv(self.csv_horn_path)
		# Subscriptions
		self.create_subscription(HornCmd, self.horn_cmd_topic, self.hornCmdCallback, 10)
		self.create_subscription(PoseStamped, self.pose_twist_fusion_topic, self.positionCallback, 10)
		self.create_subscription(Odometry, self.kinematic_topic, self.kinematicCallback, 10)
		self.create_subscription(Float32Stamped, self.ndt_score_topic, self.ndtScoreCallback, 10)
		# Timer
		# self.main_time = self.create_timer(0.1, self.main)

	def positionCallback(self, msg: PoseStamped):
		self.robot_pose = msg

	def kinematicCallback(self, msg: Odometry):
		self.robot_kinematic = msg

	def ndtScoreCallback(self, msg: Float32Stamped):
		self.ndt_score = msg

	def hornCmdCallback(self, msg: HornCmd):
		self.horn_cmd = msg
		if msg.state:
			self.stamp_hornpose()

	def euler_from_quaternion(self, quaternion):
		x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = atan2(sinr_cosp, cosr_cosp)
		sinp = 2 * (w * y - z * x)
		pitch = asin(sinp)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
		return roll, pitch, yaw

	def stamp_hornpose(self):
		try:
			pose = self.robot_kinematic.pose.pose
			self.nearest_node = self.CallNearestNode(station_list=self.horn_station)
			horn_pose = {
				"station": self.nearest_node[1],
				"node": self.nearest_node[0],
				"x": pose.position.x,
				"y": pose.position.y,
				"z": pose.position.z,
				"qx": pose.orientation.x,
				"qy": pose.orientation.y,
				"qz": pose.orientation.z,
				"qw": pose.orientation.w,
			}
			self.hornpoints.append(horn_pose)
			self.get_logger().info(f"Stamped horn point at: x={horn_pose['x']:.2f}, y={horn_pose['y']:.2f}")
		except Exception as e:
			self.get_logger().error(f"Failed to stamp horn point: {e}")

	def CallNearestNode(self, station_list):
		if not station_list:
			return None
		robot_pos = self.robot_kinematic.pose.pose.position
		min_dist = float('inf')
		nearest_node = None
		nearest_station = None
		nearest_index = -1
		for idx, station in enumerate(station_list):
			dx = robot_pos.x - station['x']
			dy = robot_pos.y - station['y']
			dist = math.hypot(dx, dy)
			if dist < min_dist:
				min_dist = dist
				nearest_node = station['node']
				nearest_station = station['station']
				nearest_index = idx
		return nearest_node, nearest_station, nearest_index

	def read_horn_station_csv(self, csv_path):
		data = []
		try:
			with open(csv_path, mode='r', newline='', encoding='utf-8') as csvfile:
				reader = csv.DictReader(csvfile)
				for row in reader:
					data.append({
						'station': row['station'],
						'node': row['node'],
						'x': float(row['x']),
						'y': float(row['y']),
						'z': float(row['z']),
						'qx': float(row['qx']),
						'qy': float(row['qy']),
						'qz': float(row['qz']),
						'qw': float(row['qw']),})
		except Exception as e:
			self.get_logger().error(f"Error reading horn station CSV: {e}")
		return data

	def save_pose_data_to_csv(self, data_list, csv_path):
		fieldnames = ['station', 'node', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
		try:
			with open(csv_path, mode='w', newline='', encoding='utf-8') as csvfile:
				writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
				writer.writeheader()
				for data in data_list:
					writer.writerow(data)
		except Exception as e:
			self.get_logger().error(f"Error saving horn pose data: {e}")

	def main(self):
		self.nearest_node = self.CallNearestNode(self.horn_station)
		if self.nearest_node:
			self.get_logger().info(f'Nearest horn node: {self.nearest_node[0]}, horn state: {self.horn_cmd.state}')
		else:
			self.get_logger().info('No nearby horn node found.')

	def destroy_node(self):
		if self.hornpoints:
			self.save_pose_data_to_csv(self.hornpoints, self.horn_save_file)
			self.get_logger().info(f"Saved {len(self.hornpoints)} horn point(s) to {self.horn_save_file}")
			# self.plot_outline()
		super().destroy_node()
	
	def plot_outline(self):
		self.points_outline.plot_map(target_map=self.points_outline.map_image,
									target_goalpoints=self.points_outline.goalpoints,
									target_hornpoints=self.hornpoints,
									trajectory_csv_path=self.points_outline.csv_path)


def main(args=None):
	rclpy.init(args=args)
	node = TowtruckStampHornPoints()
	try:
		rclpy.spin(node)
	except Exception as e:
		node.get_logger().error(f'Exception: {e}')
	# except KeyboardInterrupt:
	# 	node.plot_outline()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
