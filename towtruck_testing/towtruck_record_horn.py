#!/usr/bin/env python3
import os
import time
import math
import datetime
import requests
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from ping3 import ping
from math import asin, atan2, cos, sin
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tier4_debug_msgs.msg import Float32Stamped
from std_srvs.srv import Trigger
from towtruck_msgs.srv import RecordGoal


class TowtruckRecordPoint(Node):
	def __init__(self):
		super().__init__('towtruck_record_points')
		self.declare_parameter("ndt_score_topic", "/localization/pose_estimator/nearest_voxel_transformation_likelihood")
		self.declare_parameter("pose_twist_fusion_topic", "/localization/pose_twist_fusion_filter/pose")
		self.declare_parameter("save_csv_path", "/home/nontanan/ros2_ws/src/towtruck_testing/csv")
		self.declare_parameter("kinematic_topic", "/localization/kinematic_state")
		self.declare_parameter("debug_info", True)
		self.declare_parameter("update_endpoint", "internal_report/record_report")
		self.ndt_score_topic = self.get_parameter("ndt_score_topic").get_parameter_value().string_value
		self.pose_twist_fusion_topic = self.get_parameter("pose_twist_fusion_topic").get_parameter_value().string_value
		self.kinematic_topic = self.get_parameter("kinematic_topic").get_parameter_value().string_value
		self.debug_info = self.get_parameter("debug_info").get_parameter_value().bool_value
		self.update_endpoint = self.get_parameter("update_endpoint").get_parameter_value().string_value
		self.station_save_file = os.path.join(self.get_parameter("save_csv_path").get_parameter_value().string_value, f"GoalPoints_{datetime.datetime.now()}.csv")
		self.horn_save_file = os.path.join(self.get_parameter("save_csv_path").get_parameter_value().string_value, f'HoenPoints_{datetime.datetime.now()}.csv')
		self.create_station_service = self.create_service(RecordGoal, 'towtruck2/goalpoint/create', self.create_horn_points)
		self.read_service = self.create_service(RecordGoal, 'towtruck2/goalpoint/read', self.read_goalpose)
		self.update_service = self.create_service(RecordGoal, 'towtruck2/goalpoint/update', self.update_goalpose)
		self.delete_service = self.create_service(RecordGoal, 'towtruck2/goalpoint/delete/one', self.delete_goalpose)
		self.delete_all_service = self.create_service(Trigger, 'towtruck2/goalpoint/delete/all', self.delete_all_goalpose)
		self.save_service = self.create_service(Trigger, 'towtruck2/goalpoint/save', self.save_goalpoints_to_csv)
		self.x, self.y, self.yaw, self.vel = 0.0, 0.0, 0.0, 0.0
		self.robot_pose = PoseStamped()
		self.robot_kinematic = Odometry()
		self.ndt_score = Float32Stamped()
		self.goalpoints = []
		self.received_pose_time = self.get_clock().now().nanoseconds * 1e-9
		self.received_kinematic_time = self.get_clock().now().nanoseconds * 1e-9
		self.position_subscription = self.create_subscription(PoseStamped, self.pose_twist_fusion_topic, self.positionCallback, 10)
		self.kinematic_subscription = self.create_subscription(Odometry, self.kinematic_topic, self.kinematicCallback, 10)
		self.ndt_score_sub = self.create_subscription(Float32Stamped, self.ndt_score_topic, self.ndtScoreCallback, 10)
		self.main_time = self.create_timer(0.1, self.main)
	
	def positionCallback(self, msg: PoseStamped):
		self.received_pose_time = self.get_clock().now().nanoseconds * 1e-9
		self.robot_pose = msg
	
	def kinematicCallback(self, msg):
		self.received_kinematic_time = self.get_clock().now().nanoseconds * 1e-9
		self.robot_kinematic = msg
	
	def ndtScoreCallback(self, msg):
		self.ndt_score = msg

	def euler_from_quaternion(self, quaternion):
		x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = atan2(sinr_cosp, cosr_cosp)
		sinp = 2 * (w * y - z * x)
		pitch = asin(sinp)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = atan2(siny_cosp, cosy_cosp)
		return roll, pitch, yaw
	
	def create_horn_points(self, request, response):
		try:
			goal = {
				"station": request.station_name,
				"node": request.node_name,
				"x": self.robot_kinematic.pose.pose.position.x,
				"y": self.robot_kinematic.pose.pose.position.y,
				"z": self.robot_kinematic.pose.pose.position.z,
				"qx": self.robot_kinematic.pose.pose.orientation.x,
				"qy": self.robot_kinematic.pose.pose.orientation.y,
				"qz": self.robot_kinematic.pose.pose.orientation.z,
				"qw": self.robot_kinematic.pose.pose.orientation.w,
				"yaw": self.euler_from_quaternion(self.robot_kinematic.pose.pose.orientation)[2],
				"note_type": "station",
				"description": f"record station {datetime.datetime.now()}",
				"ndt_score": self.ndt_score.data
			}
			self.goalpoints.append(goal)
			self.save_goalpoints_to_csv()
			response.success = True
			response.message = f"Node '{goal['name']}' at ({goal['x']:.2f}, {goal['y']:.2f}) recorded successfully."
		except Exception as error:
			response.success = False
			response.message = f"Error: {error}"
		return response
	
	def read_goalpose(self, request, response):
		try:
			if not os.path.exists(self.station_save_file):
				response.success = False
				response.message = f"CSV file does not exist: {self.station_save_file}"
				return response
			data = pd.read_csv(self.station_save_file)
			if request.station_name == "":  # Read all goalpoints
				for idx in range(len(data)):
					self.get_logger().info(f'{data["name"].iloc[idx]} coordinate: {data["x"].iloc[idx]},{data["y"].iloc[idx]}')
				response.success = True
				response.message = str(data)
			else:
				station = data[data["name"] == request.station_name]
				if not station.empty:
					self.get_logger().info(f'{station["name"].iloc[0]} coordinate: {station["x"].iloc[0]},{station["y"].iloc[0]}')
					response.success = True
					response.message = str(station)
				else:
					response.success = False
					response.message = f"Station '{request.station_name}' not found in CSV."
		except Exception as read_goal_err:
			self.get_logger().fatal(f'read_goalpose: {read_goal_err}')
			response.success = False
			response.message = str(read_goal_err)
		return response

	def update_goalpose(self, request, response):
		try:
			data = pd.read_csv(self.station_save_file)
			station = data[data["name"] == request.station_name]
			if not station.empty:
				station.loc[station.index, ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw']] = [
					self.robot_kinematic.pose.pose.position.x,
					self.robot_kinematic.pose.pose.position.y,
					self.robot_kinematic.pose.pose.position.z,
					self.robot_kinematic.pose.pose.orientation.x,
					self.robot_kinematic.pose.pose.orientation.y,
					self.robot_kinematic.pose.pose.orientation.z,
					self.robot_kinematic.pose.pose.orientation.w,
					self.euler_from_quaternion(self.robot_kinematic.pose.pose.orientation)[2]]
				station['description'] = f"Updated station {datetime.datetime.now()}"
				station['ndt_score'] = self.ndt_score.data
				station.to_csv(self.station_save_file, index=False)
				response.success = True
				response.message = f"Station '{request.station_name}' updated successfully."
			else:
				response.success = False
				response.message = f"Station '{request.station_name}' not found."
		except Exception as update_goal_err:
			self.get_logger().fatal(f'update_goalpose: {update_goal_err}')
			response.success = False
			response.message = str(update_goal_err)
		return response

	def delete_goalpose(self, request, response):
		try:
			if not os.path.exists(self.station_save_file):
				response.success = False
				response.message = f"CSV file does not exist: {self.station_save_file}"
				return response
			data = pd.read_csv(self.station_save_file)
			station = data[data["name"] == request.station_name]
			if not station.empty:
				data = data[data["name"] != request.station_name]
				data.to_csv(self.station_save_file, index=False)
				self.get_logger().info(f"Deleted station: {request.station_name}")
				response.success = True
				response.message = f"Station '{request.station_name}' deleted successfully."
			else:
				response.success = False
				response.message = f"Station '{request.station_name}' not found in CSV."
		except Exception as delete_err:
			self.get_logger().fatal(f"delete_goalpose error: {delete_err}")
			response.success = False
			response.message = str(delete_err)

	def delete_all_goalpose(self, request, response):
		try:
			if not os.path.exists(self.station_save_file):
				response.success = False
				response.message = f"CSV file does not exist: {self.station_save_file}"
				return response
			data = pd.DataFrame(columns=["name", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw", "note_type", "description", "ndt_score"])
			data.to_csv(self.station_save_file, index=False)
			self.get_logger().info("All stations deleted from the CSV file.")
			response.success = True
			response.message = "All goalpoints deleted successfully."
		except Exception as delete_all_err:
			self.get_logger().fatal(f"delete_all_goalpose error: {delete_all_err}")
			response.success = False
			response.message = str(delete_all_err)

	def save_goalpoints_to_csv(self):
		try:
			csv_dir = os.path.dirname(self.station_save_file)
			os.makedirs(csv_dir, exist_ok=True)
			if not os.path.exists(self.station_save_file):
				df = pd.DataFrame(self.goalpoints)
				df.to_csv(self.station_save_file, index=False)
				self.get_logger().info(f"Goalpoints saved to new CSV file: {self.station_save_file}")
			else:
				df = pd.DataFrame(self.goalpoints)
				df.to_csv(self.station_save_file, mode='a', header=False, index=False)
				self.get_logger().info(f"Goalpoints appended to existing CSV: {self.station_save_file}")
		except Exception as e:
			self.get_logger().error(f"Failed to save goalpoints to CSV: {e}")
	
	def main(self):
		pass


def main(args=None):
	rclpy.init(args=args)
	node = TowtruckRecordPoint()
	try:
		rclpy.spin(node)
	except Exception as error:
		print(f'Error: {error}')
	except KeyboardInterrupt:
		node.save_goalpoints_to_csv()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
