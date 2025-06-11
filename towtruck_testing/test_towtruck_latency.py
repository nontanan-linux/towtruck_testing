#!/usr/bin/env python3
# Built-in
import os
import time
import math
import datetime
# Third-party
import requests
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from ping3 import ping
from math import asin,atan2,cos,sin
# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tier4_debug_msgs.msg import Float32Stamped

class TestTowtruckLatency(Node):
	def __init__(self):
		super().__init__('test_towtruck_latency')
		self.declare_parameter("main_server_ip",'10.60.205.2')
		self.declare_parameter("main_server_port",5012)
		self.declare_parameter("ndt_score_topic","/localization/pose_estimator/nearest_voxel_transformation_likelihood")
		self.declare_parameter("pose_twist_fusion_topic","/localization/pose_twist_fusion_filter/pose")
		self.declare_parameter("save_csv_path", "/home/nontanan/ros2_ws/src/towtruck_testing/csv")
		self.declare_parameter("kinematic_topic", "/localization/kinematic_state")
		self.declare_parameter("update_timeout", 1.0)
		self.declare_parameter("ping_timeout", 1.0)
		self.declare_parameter("debug_info", True)
		self.declare_parameter("update_enpoint", "internal_report/record_report")
		self.main_server_ip = self.get_parameter("main_server_ip").get_parameter_value().string_value
		self.main_server_port = str(self.get_parameter("main_server_port").get_parameter_value().integer_value)
		self.ndt_score_topic = self.get_parameter("ndt_score_topic").get_parameter_value().string_value
		self.pose_twist_fusion_topic = self.get_parameter("pose_twist_fusion_topic").get_parameter_value().string_value
		self.kinematic_topic = self.get_parameter("kinematic_topic").get_parameter_value().string_value
		self.debug_info = self.get_parameter("debug_info").get_parameter_value().bool_value
		self.update_enpoint = self.get_parameter("update_enpoint").get_parameter_value().string_value
		self.update_timeout = self.get_parameter("update_timeout").get_parameter_value().double_value
		self.ping_timeout = self.get_parameter("ping_timeout").get_parameter_value().double_value
		self.latency_log = os.path.join(self.get_parameter("save_csv_path").get_parameter_value().string_value, f"latency_log_{datetime.datetime.now()}.csv")
		self
		self.x, self.y, self.yaw, self.vel = 0.0, 0.0, 0.0, 0.0
		self.robot_pose = PoseStamped()
		self.robot_kinematic = Odometry()
		self.ndt_score = Float32Stamped()
		self.received_pose_time = self.get_clock().now().nanoseconds * 1e-9
		self.received_kinematic_time = self.get_clock().now().nanoseconds * 1e-9
		self.position_subscription = self.create_subscription(PoseStamped, self.pose_twist_fusion_topic, self.positionCallback ,10)
		self.kinematic_subscription = self.create_subscription(Odometry, self.kinematic_topic, self.kinematicCallback, 10)
		self.ndt_score_sub = self.create_subscription(Float32Stamped, self.ndt_score_topic, self.ndtScoreCallback, 10)
		self.main_time = self.create_timer(1.0, self.main)
		self.param_case = f"Test-{datetime.datetime.now()}"
	
	def positionCallback(self,msg:PoseStamped):
		self.received_pose_time = self.get_clock().now().nanoseconds * 1e-9
		self.robot_pose = msg
	
	def kinematicCallback(self, msg):
		self.received_kinematic_time = self.get_clock().now().nanoseconds * 1e-9
		self.robot_kinematic = msg
	
	def ndtScoreCallback(self,msg):
		self.ndt_score = msg

	def euler_from_quaternion(self,quaternion):
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = atan2(sinr_cosp, cosr_cosp)
		sinp = 2 * (w * y - z * x)
		pitch = asin(sinp)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = atan2(siny_cosp, cosy_cosp)
		return roll, pitch, yaw
	
	def update_score(self, ip, port, endpoint, data):
		try:
			url = f'http://{ip}:{port}/{endpoint}'
			params = {"case_name": self.param_case}
			headers = {
				'accept': 'application/json',
				'Content-Type': 'application/json'}
			response = requests.post(url, params=params, headers=headers, json=data, timeout=self.update_timeout)
			return response.status_code, response.text
		except Exception as err:
			self.get_logger().fatal(f'Update Score Error: {err}')
			return None, str(err)
	
	def record_to_csv(self, data, status_code):
		data['status_code'] = status_code
		df = pd.DataFrame([data])
		if not os.path.isfile(self.latency_log):
			df.to_csv(self.latency_log, index=False)
		else:
			df.to_csv(self.latency_log, mode='a', header=False, index=False)
	
	def plot_results(self, csv_file = ''):
		try:
			df = pd.read_csv(csv_file)
			df[['x', 'y', 'yaw']] = df['coordinate'].str.split(',', expand=True).astype(float)
			plt.figure(figsize=(10, 8))
			scatter = plt.scatter(
				df['x'], df['y'],
				c=df['score'],
				s=df['latency'],
				cmap='viridis',
				alpha=0.8,
				edgecolors='w')
			plt.colorbar(scatter, label='NDT Score')
			plt.xlabel('X')
			plt.ylabel('Y')
			plt.title('AGV Position and Latency')
			plt.grid(True)
			plt.axis('equal')
			plt.show()
		except Exception as plt_err:
			self.get_logger().fatal(f'Has Error plot: {plt_err}')

	def main(self):
		try:
			self.current_time = self.get_clock().now().nanoseconds * 1e-9
			if self.current_time - self.received_kinematic_time < 1.0:
				_, _, yaw = self.euler_from_quaternion(self.robot_kinematic.pose.pose.orientation)
				self.x = self.robot_kinematic.pose.pose.position.x
				self.y = self.robot_kinematic.pose.pose.position.y
				self.yaw = yaw
				self.vel = self.robot_kinematic.twist.twist.linear.x
			else:
				self.get_logger().fatal('No recent pose or kinematic data received.')
				return
			latency = ping(self.main_server_ip, timeout=self.ping_timeout)
			if latency is None:
				self.get_logger().fatal(f"Ping to {self.main_server_ip} failed.")
				return
			data = {
				"coordinate": f"{self.x},{self.y},{self.yaw}",
				"latency": float(latency*1000),
				"score": float(self.ndt_score.data),
				"vehicle_name": "AGV2",
				"velocity": float(self.vel)}
			status_code, response_text = self.update_score(ip=self.main_server_ip,port=self.main_server_port,endpoint=self.update_enpoint,data=data)
			if status_code == 200:
				self.get_logger().info(f'Successfully updated: {response_text} | Data: {data}')
			else:
				self.get_logger().error(f'Update failed with code {status_code}: {response_text}')
			self.record_to_csv(data, status_code)
		except Exception as error:
			self.get_logger().fatal(f'Error in main loop: {error}')

def main(args=None):
	rclpy.init(args=args)
	node = TestTowtruckLatency()
	try:
		rclpy.spin(node)
	except Exception as error:
		print(f'Error: {error}')
	# except KeyboardInterrupt:
		# node.plot_results(csv_file=node.latency_log)
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()