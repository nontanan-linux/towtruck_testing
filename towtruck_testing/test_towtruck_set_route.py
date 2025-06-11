#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from autoware_auto_vehicle_msgs.msg import Engage
from autoware_auto_system_msgs.msg import AutowareState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, SetRoutePointsWithId 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tier4_debug_msgs.msg import Float32Stamped
from towtruck_msgs.msg import RobotState, HornCmd
# from towtruck_map_image import MapImageOutline
import os
import time
import math
import pandas as pd
import numpy as np
import random
import asyncio
import tf_transformations
from collections import defaultdict 
from itertools import permutations
from dijsktra_calc import DijsktraCalculation
from unit_edges import customer_uni_edges
from enum import Enum
from datetime import datetime
import requests,json

class AutowareStateValue(Enum):
	INITIALIZING = 1
	WAITING_FOR_ROUTE = 2
	PLANNING = 3
	WAITING_FOR_ENGAGE = 4
	DRIVING = 5
	ARRIVED_GOAL = 6
	FINALIZING = 7

class TestRoutePointsClient(Node):
	def __init__(self):
		super().__init__('TestRoutePointsWithID')
		self.declare_parameter("csv_goal_path", "~/autoware.bg2/data/BG/GoalPoints.csv")
		self.declare_parameter("csv_horn_path", "~/autoware.bg2/data/BG/HornPoints.csv")
		# self.declare_parameter("csv_station_path", '/home/nontanan/ros2_ws/src/towtruck_testing/csv/Station.csv')
		self.declare_parameter("save_log_mission", '/home/nontanan/ros2_ws/src/towtruck_testing/csv/')
		self.declare_parameter("robot_state_topic", "/robot_state")
		self.declare_parameter("horn_cmd_topic", "/horn_cmd")
		self.declare_parameter("debug_info", True)
		self.declare_parameter("simulation", False)
		self.declare_parameter("save_log_to_csv", True)
		self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
		self.csv_goal_path = self.get_parameter("csv_goal_path").get_parameter_value().string_value
		self.save_log_path = self.get_parameter("save_log_mission").get_parameter_value().string_value
		# self.stations_path = self.get_parameter("csv_station_path").get_parameter_value().string_value
		# self.csv_goal_path = os.path.join(os.getcwd(), self.csv_goal_path)
		# self.csv_save_log = self.generate_log_csv(save_path=self.save_log_path)
		self.goal_data_frame = pd.read_csv(self.csv_goal_path)
		# self.stations_frame = pd.read_csv(self.stations_path)
		self.csv_horn_path = os.path.expanduser(self.get_parameter("csv_horn_path").get_parameter_value().string_value)
		self.horn_save_file = os.path.join(
			self.get_parameter("save_log_mission").get_parameter_value().string_value,
			f'HornPointsStamp_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
		self.kinematic = Odometry()
		self.engage_cmd = Engage()
		self.engage_msg = Engage()
		self.autoware_state = AutowareState()
		# self.points_outline = MapImageOutline()
		self.robot_pose = PoseStamped()
		self.robot_kinematic = Odometry()
		self.robot_state = RobotState()
		self.horn_cmd = HornCmd()
		self.ndt_score = Float32Stamped()
		self.hornpoints = []
		self.current_node = None
		self.start_point = self.CallNearestNode()
		self.goal_point = 'P01S'
		self.dijsktra = DijsktraCalculation()
		self.d_path = []
		self.path_approved = set()
		self.mission_status = False
		self.mission_message = ''
		self.mission_route_status = None
		self.path_activated = False
		self.wait_for_response = True
		self.ready = True
		self.round = 1
		self.arrived_time = 0
		self.control_state = 'initial'
		self.success = {'round': self.round, 'start': None, 'goal': None,'send_goal': False,
						'planner': False, 'engage': False, 'driving': False,'reach_goal': False, 'round_status': None,
						'ready': False,'mission': None, 'path_approved': self.path_approved,'start_time':str(), 'finish_time':str()}
		#=============================== Autoware Service ===============================		
		self.engage_client = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_autonomous")
		self.stop_client = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_stop")
		self.overtake_client = self.create_client(SetRoutePointsWithId, "/planning/mission_planning/set_route_points_with_id")
		self.client = self.create_client(SetRoutePointsWithId, '/planning/mission_planning/set_route_points_with_id')
		while not (self.engage_client.wait_for_service(timeout_sec=2.0) and self.stop_client.wait_for_service(timeout_sec=2.0) and self.client.wait_for_service(timeout_sec=2.0)) :
			self.get_logger().warn("Autoware service is not available ...")
			print(self.engage_client.wait_for_service(timeout_sec=2.0),self.stop_client.wait_for_service(timeout_sec=2.0),self.client.wait_for_service(timeout_sec=2.0))
		#=============================== Autoware Subscription ===============================
		self.autoware_engage_subcription = self.create_subscription(Engage, 'autoware/engage', self.autoware_engage_callback, 10)
		self.autoware_state_subcription = self.create_subscription(AutowareState, 'autoware/state', self.autoware_state_callback, 10)
		self.engage_pub = self.create_publisher(Engage, 'autoware/engage', 10)
		self.main_time = self.create_timer(0.1, self.main)
		if self.simulation:
			self.tf_buffer = Buffer()
			self.tf_listener = TransformListener(self.tf_buffer, self)
			self.lookup_timer = self.create_timer(0.2, self.lookup_transfrom)
		else:
			self.kinematic_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.kinematic_callback, 10)
		self.stations = self.goal_data_frame['name'].to_list()

	def initial_state(self):
		start_time = time.time()
		while time.time()-start_time <= 1.5:
			self.get_logger().info('initial state')
		return self.CallNearestNode()
	
	def get_station(self):
		return pd.read_csv(self.csv_goal_path).query('node_type == "station"')['name'].to_list()
		# return pd.read_csv(self.stations_path)['name'].to_list()
	
	def autoware_engage_callback(self, msg):
		self.engage_msg = msg
	
	def autoware_state_callback(self, msg):
		self.autoware_state = msg
	
	def euler_from_quaternion(self,quaternion):
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = math.atan2(sinr_cosp, cosr_cosp)
		sinp = 2 * (w * y - z * x)
		pitch = math.asin(sinp)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = math.atan2(siny_cosp, cosy_cosp)
		return roll, pitch, yaw
	
	def lookup_transfrom (self) :
		try:
			t = self.tf_buffer.lookup_transform("map","base_link",rclpy.time.Time())
			_, _, yaw = self.euler_from_quaternion(t.transform.rotation)
			self.kinematic.pose.pose.position.x = t.transform.translation.x 
			self.kinematic.pose.pose.position.y = t.transform.translation.y
			self.kinematic.pose.pose.position.z = t.transform.translation.z
			self.kinematic.pose.pose.orientation = t.transform.rotation
		except Exception as txt:
			self.get_logger().warn(
				f'(API) Could not transform base_link to map: {txt}')
	
	def activate_path(self):
		try:
			if(not self.path_activated):
				self.start_point = self.current_node
				random_station = self.get_station()
				if self.current_node in random_station:
					random_station.remove(self.current_node)
				self.goal_point = random.choice(random_station)
				self.d_path = self.dijsktra.cal_path(self.current_node, self.goal_point)
				self.get_logger().info(f'round: {self.round}, from: {self.current_node}, goal: {self.goal_point}, Set route: {self.d_path[0]}')
				if self.d_path[-1] == 0:
					self.get_logger().fatal(f'Path is worng format {self.d_path}. try again')
					return False
				self.send_route_request(path=self.d_path[0])
				self.success['round'] = self.round
				self.success['start'] = self.start_point
				self.success['goal'] = self.goal_point
				self.success['send_goal'] = True
				self.success['start_time'] = datetime.now().strftime('%Y_%m_%d__%H_%M_%S_%f')
				# self.success['start_time'] = datetime.fromtimestamp(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9)
				# self.save_log_to_csv(source=self.success, save_path=self.csv_save_log)
				self.path_activated = True
				self.control_state = 'planning'
				self.wait_for_response = True
				return True
			else:
				return True
		except Exception as error:
			self.get_logger().error(f'Set route error: {error}')
			self.path_activated = False
			return False
	
	def wait_for_planning(self):
		if not self.wait_for_response:
			try:
				if(self.mission_message != '' or self.mission_route_status == 4 or self.mission_route_status == None):
						self.get_logger().fatal(f'Mission_Planning({self.mission_status}): {self.mission_message}. try again')
						return False
				print(f'mission soute state: {self.mission_route_status}')
				self.control_state = 'wait_for_engage'
				self.success['mission'] = [self.mission_status, self.mission_route_status, self.mission_message]
				self.success['planner'] = True
				# self.save_log_to_csv(source=self.success, save_path=self.csv_save_log)
				self.ready = False
				self.path_activated = False
				return True
			except Exception as planning_error:
				self.get_logger().fatal(f'Planning error: {planning_error}')
				return False
		else:
			return False
	
	def activate_vehicle(self):
		self.engage_cmd.stamp = self.get_clock().now().to_msg()
		self.engage_cmd.engage = True
		self.engage_pub.publish(self.engage_cmd)
		self.success['engage'] = True
		self.success['driving'] = True
		self.control_state = 'driving'
		# self.save_log_to_csv(source=self.success, save_path=self.csv_save_log)
	
	def reach_goal(self):
		self.get_logger().info(f'Round: {self.round} mission complete, path approve: {self.path_approved}')
		self.success['finish_time'] = datetime.now().strftime('%Y_%m_%d__%H_%M_%S_%f')
		# self.success['finish_time'] = datetime.fromtimestamp(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9)
		self.success['reach_goal'] = True
		self.success['ready'] = True
		self.success['path_approved'] = self.path_activated
		self.control_state = 'arrived_goal'
		self.get_logger().info(f'Record mission status{self.success}')
		self.save_log_to_csv(source=self.success, save_path=self.csv_save_log)
		self.round += 1
		self.mission_status = False
		self.mission_message = ''
		self.path_approved = set()
		self.ready = True

	def kinematic_callback(self, msg):
		self.kinematic = msg

	def main(self):
		self.current_node, self.st, self.min_idx = self.CallNearestNode()
		if self.autoware_state.state == 2 and self.ready or self.control_state == 'arrived_goal' and (not self.engage_msg.engage):
			while(not self.activate_path()):
				print('wait for activate path')
		elif self.path_activated:
			self.wait_for_planning()
		elif self.autoware_state.state == 4 and self.control_state == 'wait_for_engage':
			self.activate_vehicle()
		elif self.autoware_state.state == 6 and self.control_state == 'driving' and (not self.engage_msg.engage):
			self.reach_goal()
		self.check_arrived_goal()
		if self.arrived_time >= 20:
			self.get_logger().error(f'Round: {self.round}, autoware state ({self.autoware_state.state}) not ARRIVED_GOAL')
		if self.control_state == 'driving':
			self.path_approved.add(self.current_node)
		# print(f'waitting response: {self.wait_for_response},path activate: {self.path_activated}, control state {self.control_state}')
	
	def generate_log_csv(self, save_path):
		os.makedirs(save_path, exist_ok=True)
		timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
		filename = f"save_mission_log_{timestamp}.csv"
		file_path = os.path.join(save_path, filename)
		return file_path
	
	def save_log_to_csv(self, source, save_path):
		save_df = pd.DataFrame(source)
		save_df.to_csv(save_path, mode='a', index=False, header=not pd.io.common.file_exists(save_path))
	
	def check_arrived_goal(self):
		if self.autoware_state.state == 5 and (self.kinematic.twist.twist.linear.x <1e-9) and self.control_state=='driving':
			self.arrived_time += 1
		else:
			self.arrived_time = 0

	def send_route_request(self, path):
		request = SetRoutePointsWithId.Request()
		request.ids = path
		request.mode = True  
		future = self.client.call_async(request)
		future.add_done_callback(self.response_callback)

	def response_callback(self, future):
		try:
			response = future.result()
			self.mission_status = response.status.success
			self.mission_message = str(response.status.message)
			self.mission_route_status = response.status.code
			self.wait_for_response = False
			self.get_logger().info(f'Response: {response}')
		except Exception as e:
			self.get_logger().error(f'Service call failed: {str(e)}')
	
	def calc_heading(self, quaternion):
		return tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
	
	def CallCurrentNode(self):
		try:
			if self.goal_data_frame.empty:
				self.get_logger().warn(f"No matching station found for node: {node}")
			if self.d_path:
				for node in self.d_path:
					current_data_frame = self.goal_data_frame.loc[self.goal_data_frame['name'] == node].to_dict(orient="records")[0]
					sub_goal = np.array([
						float(current_data_frame["x"]), 
						float(current_data_frame["y"])
					])
					abs_distance_incoming = self.distance_pt_to_frame(self.kinematic.pose.pose.position, current_data_frame)
					if abs_distance_incoming < 5.0:
						self.current_node = node
						self.current_node_position = sub_goal
		except Exception as txt:
			self.get_logger().error(f"CallCurrentNode error: {txt}")

	def CallNearestNode(self):
		if self.goal_data_frame.empty:
			return None
		stations = self.goal_data_frame["name"].tolist()
		distances = []
		for station in stations:
			current_data_frame = self.goal_data_frame.loc[self.goal_data_frame["name"] == station].to_dict(orient="records")[0]
			if not current_data_frame:
				continue
			dist = self.distance_pt_to_frame(self.kinematic.pose.pose.position, current_data_frame)
			distances.append(dist)
		if not distances:
			return None
		min_index = distances.index(min(distances))
		return stations[min_index], stations, min_index
	
	def distance_pt_to_frame(self, curr_pose, goal_pose):
		try:
			goal_arr = np.array([float(goal_pose["x"]), float(goal_pose["y"])])
			curr_arr = np.array([float(curr_pose.x), float(curr_pose.y)])
			return np.linalg.norm(goal_arr - curr_arr)
		except KeyError:
			print("Error: Missing x or y coordinate in goal_pose.")
			return float('inf')
	
	def test(self):
		print(f'Edges: {customer_uni_edges}')

def test():
	rclpy.init()
	node = TestRoutePointsClient()
	node.test()
	node.destroy_node()
	rclpy.shutdown()

def main():
	rclpy.init()
	node = TestRoutePointsClient()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Shutting down...")
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	# main()
	test()
