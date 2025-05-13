#!/usr/bin/env python3
# Built-in
import os
import time
import math

# Third-party
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from autoware_auto_planning_msgs.msg import PathWithLaneId, Path

# Project
from enum import Enum
from itertools import zip_longest
from unit_edges import uni_edges

class RouteState(Enum):
	INITIALIZING = 1
	WAITING_FOR_ROUTE = 2
	ROUTE_PLANNING = 3
	READY_TO_RECORD = 4
	PATH_RECORDED = 5
	FINALIZING = 6
	FAILURE = 0

class PathWithLaneIDSubscriber(Node):
	def __init__(self):
		super().__init__('path_with_lane_id_subscriber')
		self.declare_parameter("csv_goal_path", "/home/nontanan/autoware.bg2/data/BG/GoalPoints.csv")
		self.declare_parameter("simulation", False)
		self.declare_parameter("initialpose_topic", "initialpose")
		self.declare_parameter("goal_topic", "/planning/mission_planning/goal")
		self.declare_parameter("path_topic", "/planning/scenario_planning/lane_driving/behavior_planning/path")
		self.declare_parameter("path_lane_topic", "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id")
		self.declare_parameter("save_picture_path", "/home/nontanan/ros2_ws/src/towtruck_testing/towtruck_testing/pict/07052025")
		self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
		self.csv_goal_path = self.get_parameter("csv_goal_path").get_parameter_value().string_value
		self.initialpose_topic = self.get_parameter("initialpose_topic").get_parameter_value().string_value
		self.path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
		self.goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
		self.path_lane_topic = self.get_parameter("path_lane_topic").get_parameter_value().string_value
		self.save_picture_path = self.get_parameter("save_picture_path").get_parameter_value().string_value
		self.csv_goal_path = os.path.join(os.getcwd(), self.csv_goal_path)
		self.goal_data_frame = pd.read_csv(self.csv_goal_path)
		self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)
		self.path_sub = self.create_subscription(Path, self.path_topic,self.path_callback, self.qos_profile)
		self.path_with_lane_id_sub = self.create_subscription(PathWithLaneId, self.path_lane_topic, self.path_with_lane_id_callback,self.qos_profile)
		self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
		self.planning_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
		self.csv_goal_file = '~/autoware.bg2/data/BG/GoalPoints.csv'
		self.station_file = '~/ros2_ws/src/towtruck_testing/csv/Station.csv'
		self.csv_save_path = '~/ros2_ws/src/towtruck_testing/csv/path_with_lane_id.csv'
		self.main_time = self.create_timer(0.2, self.main)
		self.path_lane = []
		self.path_msg = Path()
		self.route_state = None
		self.x, self.y, self.yaw = [], [], []
		self.path_x, self.path_y, self.path_yow = [], [], []
		self.linear_vel, self.angular_vel, self.lane_id = [], [], []
		self.path = {'x': [], 'y': []}
		self.left_bound = {'x': [], 'y': []}
		self.rigth_bound = {'x': [], 'y': []}
		self.path_left_bound = {'x': [], 'y': []}
		self.path_rigth_bound = {'x': [], 'y': []}
		self.define = False
		self.wait_for_generate_path = None
		self.route_num = 0
		self.save = False
		
	def generate_paths_from_edges(self):
		if self.route_num < len(uni_edges):
			print(f"[LOOP] route_num: {self.route_num}/{len(uni_edges)} | state: {self.route_state}")
			print(f'path type: {type(self.path_lane)}, route_state: {self.route_state}, waitting: {self.wait_for_generate_path}')	
			if self.route_state in [None, RouteState.INITIALIZING, RouteState.FINALIZING]:
				edges = uni_edges[self.route_num]
				self.file_name = f"{edges[0]}_{edges[1]}"
				print(f'[{self.route_num}] from: {edges[0]}, to: {edges[1]}, file name: {self.file_name}')
				self.start_data = self.goal_data_frame[self.goal_data_frame['name'] == edges[0]].to_dict(orient='records')[0]
				self.goals_data = self.goal_data_frame[self.goal_data_frame['name'] == edges[1]].to_dict(orient='records')[0]
				print(f'[{self.route_num}] from: {self.start_data}, to: {self.goals_data}')
				print("[ACTION] initial_state()")
				if not self.initial_state(self.start_data):
					return
			elif self.route_state == RouteState.WAITING_FOR_ROUTE:
				print("[ACTION] execute_goal()")
				if not self.execute_goal(self.goals_data):
					return
				self.execute_goal_time = time.time()
				print(f'path type: {type(self.path_lane)}, route_state: {self.route_state}, waitting: {self.wait_for_generate_path}')
				while(time.time() - self.execute_goal_time < 0.5):
					pass
			elif self.route_state == RouteState.ROUTE_PLANNING and self.wait_for_generate_path==False and (type(self.path_lane) != list):
				generate_path = self.generate_path()
				print(f"[ACTION] generate_path(): {generate_path}")
				if not generate_path:
					print('...waiting for path...')
					return
			elif self.route_state == RouteState.READY_TO_RECORD:
				print("[ACTION] plot_path() & NEXT")
				plot_fleg = self.plot_path(self.path, self.path_left_bound, self.path_rigth_bound, self.start_data, self.goals_data, self.file_name)
				if not plot_fleg:
					self.get_logger().info('Waiting for plot data')
					return
				self.start_data = None
				self.goals_data = None
				self.path_lane = []
				self.wait_for_generate_path = None
				self.route_num += 1 
				self.route_state = RouteState.FINALIZING
			print('==============================================================')

	def initial_state(self, station):
		try:
			initial = PoseWithCovarianceStamped()
			initial.header.frame_id = 'map'
			initial.header.stamp = self.get_clock().now().to_msg()
			initial.pose.pose.position.x = station['x']
			initial.pose.pose.position.y = station['y']
			initial.pose.pose.orientation.z = station['qz']
			initial.pose.pose.orientation.w = station['qw']
			self.initialpose_pub.publish(initial)
			self.route_state = RouteState.WAITING_FOR_ROUTE
			return True
		except Exception as initial_error:
			self.get_logger().fatal(f'Initial Position Error: {initial_error}')
			return False
	
	def execute_goal(self, station):
		try:
			goal = PoseStamped()
			goal.header.frame_id = 'map'
			goal.header.stamp = self.get_clock().now().to_msg()
			goal.pose.position.x = station['x']
			goal.pose.position.y = station['y']
			goal.pose.orientation.z = station['qz']
			goal.pose.orientation.w = station['qw']
			self.planning_pub.publish(goal)
			self.wait_for_generate_path = True
			return True
		except Exception as planning_err:
			self.get_logger().fatal(f'Planning Error: {planning_err}')
			return False

	def path_with_lane_id_callback(self, msg):
		if self.route_state == RouteState.WAITING_FOR_ROUTE and self.wait_for_generate_path==True and (type(self.path_lane) == list):
			self.path_lane = PathWithLaneId()
			self.path_lane = msg
			self.route_state = RouteState.ROUTE_PLANNING
			self.wait_for_generate_path = False
			# self.get_logger().info(f'get new path, state: {self.route_state}, wait: {self.wait_for_generate_path}, check: {self.path_lane.points}')
	
	def path_callback(self, msg):
		self.path_ms = msg

	def generate_path(self):
		try:
			self.path = {'x': [], 'y': [], 'yaw': []}
			self.path_left_bound = {'x': [], 'y': []}
			self.path_rigth_bound = {'x': [], 'y': []}
			# self.path_lane
			# for pt in points:
			# 	self.path['x'].append(pt.point.pose.position.x)
			# 	self.path['y'].append(pt.point.pose.position.y)
			for idx in range(2, len(self.path_lane.points)):
				self.path['x'].append(self.path_lane.points[idx].point.pose.position.x)
				self.path['y'].append(self.path_lane.points[idx].point.pose.position.y)
				self.path['yaw'].append(self.quaternion_to_yaw(self.path_lane.points[idx].point.pose.orientation))
			if hasattr(self.path_lane, 'left_bound') and hasattr(self.path_lane, 'right_bound'):
				for lb in range(0, len(self.path_lane.left_bound)):
					self.path_left_bound['x'].append(self.path_lane.left_bound[lb].x)
					self.path_left_bound['y'].append(self.path_lane.left_bound[lb].y)
				for rb in range(0, len(self.path_lane.right_bound)):
					self.path_rigth_bound['x'].append(self.path_lane.right_bound[rb].x)
					self.path_rigth_bound['y'].append(self.path_lane.right_bound[rb].y)
			self.route_state = RouteState.READY_TO_RECORD
			return True
		except Exception as gen_err:
			self.get_logger().fatal(f'Generate Path Error: {gen_err}')
			return False
	
	def plot_path(self, path, left_bound, right_bound, start_point, goal_point, file_name):
		try:
			plt.figure(figsize=(10, 6))
			plt.scatter(start_point['x'], start_point['y'], c='orange', label='Start Point', s=50, marker='*')
			plt.text(start_point['x'] + 1, start_point['y'] + 1, start_point['name'], fontsize=12, color='orange')
			plt.scatter(goal_point['x'], goal_point['y'], c='purple', label='Goal Point', s=50, marker='X')
			plt.text(goal_point['x'] + 1, goal_point['y'] + 1, goal_point['name'], fontsize=12, color='purple')
			plt.scatter(path['x'], path['y'], c='blue', label='Path (x, y)', s=5)
			plt.plot(left_bound['x'], left_bound['y'], color='black', label='Left Bound')
			plt.plot(right_bound['x'], right_bound['y'], color='black', label='Right Bound')
			plt.xlabel('X')
			plt.ylabel('Y')
			plt.title('Path with Lane Boundaries')
			plt.legend()
			plt.axis('equal')
			os.makedirs(self.save_picture_path, exist_ok=True)
			full_path = os.path.join(self.save_picture_path, f'{file_name}.png')
			save_path = os.path.join(self.save_picture_path, f'{file_name}.csv')
			plt.savefig(full_path)
			self.get_logger().info(f'Path image saved to: {full_path}')
			plt.clf()
			self.save_path_to_csv(path, left_bound, right_bound,save_path)
			return True
		except Exception as plot_err:
			self.get_logger().fatal(f'Plot Error: {plot_err}')
			return False
	
	def main(self):
		self.generate_paths_from_edges()
		# self.merge_paths()

	def define_path(self, points):
		for idx in range(len(points)):
			self.x.append(points[idx].point.pose.position.x)
			self.y.append(points[idx].point.pose.position.y)
			self.yaw.append(self.quaternion_to_yaw(points[idx].point.pose.orientation))
			self.linear_vel.append(points[idx].point.longitudinal_velocity_mps)
			self.angular_vel.append(points[idx].point.heading_rate_rps)
			self.lane_id.append(points[idx].lane_ids)
	
	def cals_dis_points(self, path_x, path_y):
		distance = 0.0
		for idx in range(1, len(path_x)):
			distance += math.sqrt(pow(path_x[idx]-path_x[idx-1], 2)+pow(path_y[idx]-path_y[idx-1],2))
		return distance
	
	def save_path_to_csv(self, path, left_bound, right_bound, save_file):
		data = zip_longest(path['x'], path['y'], path['yaw'], left_bound['x'], left_bound['y'], right_bound['x'], right_bound['y'])
		df = pd.DataFrame(data=data, columns=['x', 'y', 'yaw', 'left_x', 'left_y', 'right_x', 'right_y'])
		df.to_csv(save_file, index=False)
		self.get_logger().info(f'Save csv to {save_file}')

	def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
		x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
		return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

	def save_data(self):
		data = zip_longest(self.x, self.y, self.yaw, self.linear_vel, self.angular_vel,
				   self.lane_id, self.left_bound['x'], self.left_bound['y'],
				   self.rigth_bound['x'], self.rigth_bound['y'], fillvalue=None)
		df = pd.DataFrame(data, columns=['x', 'y', 'yaw', 'linear_vel', 'angular_vel', 'lane_id',
								 'left_x', 'left_y', 'right_x', 'right_y'])
		# df = pd.DataFrame({
		#     'x': self.x,
		#     'y': self.y,
		#     'yaw': self.yaw,
		#     'linear_vel': self.linear_vel,
		#     'angular_vel': self.angular_vel,
		#     'lane_id': self.lane_id,
		#     'left_x': self.left_bound['x'],
		#     'left_y': self.left_bound['y'],
		#     'right_x': self.rigth_bound['x'],
		#     'right_y': self.rigth_bound['y'],
		# })
		df.to_csv(self.csv_save_path, index=False)
		print(f"Path data saved to {self.csv_save_path}")
		self.define = True
	
	def plot_data(self):
		plt.figure(figsize=(10, 6))
		plt.scatter(self.x, self.y, c='blue', label='Path (x, y)', s=5)
		plt.plot(self.left_bound['x'], self.left_bound['y'], color='green', label='Left Bound')
		plt.plot(self.rigth_bound['x'], self.rigth_bound['y'], color='red', label='Right Bound')
		plt.xlabel('X')
		plt.ylabel('Y')
		plt.title('Path with Lane Boundaries')
		plt.legend()
		plt.axis('equal')
		plt.show()

	def merge_paths(self, file_name=[]):
		paths = ['H02S', 'H01N', 'H02N', 'H03N', 'H04N', 'H05N', 'D15S', 'P02S']
		path = {'x': [], 'y': [], 'yaw': []}
		path_left_bound = {'x': [], 'y': []}
		path_rigth_bound = {'x': [], 'y': []}	
		if not self.save:
			for idx in range(1, len(paths)):
				file_name.append(os.path.join(self.save_picture_path, f'{paths[idx-1]}_{paths[idx]}.csv'))
			for file in file_name:
				print(file)
				data = pd.read_csv(file)
				path['x'].extend(data['x'].to_list())
				path['y'].extend(data['y'].to_list())
				path['yaw'].extend(data['yaw'].to_list())
				path_left_bound['x'].extend(data['left_x'].to_list())
				path_left_bound['y'].extend(data['left_y'].to_list())
				path_rigth_bound['x'].extend(data['right_x'].to_list())
				path_rigth_bound['y'].extend(data['right_y'].to_list())
			for i in range(0, len(path['x'])):
				print(f'index: {i} | x: {path["x"][i]}, y: {path["y"][i]} yaw: {path["yaw"][i]}')
			new_path = self.segment_path(path=path, segment_length=5)
			plt.scatter(path['x'], path['y'], c='blue', label='Path (x, y)', s=5)
			plt.plot(path_left_bound['x'], path_left_bound['y'], color='black', label='Left Bound')
			plt.plot(path_rigth_bound['x'], path_rigth_bound['y'], color='black', label='Right Bound')
			plt.plot(new_path['x'], new_path['y'],"*",color= "red", linewidth=1, label="generate course")
			plt.xlabel('X')
			plt.ylabel('Y')
			plt.title('Path with Lane Boundaries')
			plt.legend()
			plt.axis('equal')
			plt.show()
			self.save = True
		else:
			pass
	
	def segment_path(self, path, segment_length):
		path_reform = [[x, y] for x, y in zip(path['x'], path['y'])]
		segment_path = [path_reform[0]]
		i = 0
		while i < len(path_reform) - 1:
			j = i + 1
			while j < len(path_reform):
				dist = np.linalg.norm(np.array(path_reform[j]) - np.array(path_reform[i]))
				if dist >= segment_length:
					break
				j += 1
			if j == len(path_reform):
				break
			dx = (path_reform[j][0] - path_reform[i][0]) / dist
			dy = (path_reform[j][1] - path_reform[i][1]) / dist
			steps = int(dist // segment_length)
			for k in range(1, steps + 1):
				segment_path.append([path_reform[i][0] + dx * segment_length * k, path_reform[i][1] + dy * segment_length * k])
			i = j
		segment_path.append(path_reform[-1])
		# for id in range(1, len(segment_path)):
		# 	dist = np.linalg.norm(np.array(segment_path[id]) - np.array(segment_path[id-1]))
		# 	print(dist)
		x_vals, y_vals = zip(*segment_path)
		reformatted_path = {'x': list(x_vals), 'y': list(y_vals)}
		return reformatted_path

	def segment_path2(self, path, segment_length):
		path_reform = [[x, y] for x, y in zip(path['x'], path['y'])]
		yaw_list = path['yaw']
		segment_path = [path_reform[0]]
		segment_yaw = [yaw_list[0]]
		i = 0
		while i < len(path_reform) - 1:
			j = i + 1
			while j < len(path_reform):
				dist = np.linalg.norm(np.array(path_reform[j]) - np.array(path_reform[i]))
				if dist >= segment_length:
					break
				j += 1
			if j == len(path_reform):
				break
			dx = (path_reform[j][0] - path_reform[i][0]) / dist
			dy = (path_reform[j][1] - path_reform[i][1]) / dist
			steps = int(dist // segment_length)
			for k in range(1, steps + 1):
				segment_path.append([path_reform[i][0] + dx * segment_length * k, path_reform[i][1] + dy * segment_length * k])
				segment_yaw.append(np.degrees(np.arctan2(dy, dx)))
			i = j
		segment_path.append(path_reform[-1])
		segment_yaw.append(yaw_list[-1])
		x_vals, y_vals = zip(*segment_path)
		reformatted_path = {'x': list(x_vals), 'y': list(y_vals), 'yaw': list(segment_yaw)}
		return reformatted_path
	
	def segment_all_path(self):
		csv_directory = '/home/nontanan/ros2_ws/src/towtruck_testing/towtruck_testing/pict/07052025'
		segment_dir = os.path.join(csv_directory, 'segment_5m')
		os.makedirs(segment_dir, exist_ok=True)
		csv_files = [f for f in os.listdir(csv_directory) if f.endswith('.csv')]
		for file in csv_files:
			path = {'x': [], 'y': [], 'yaw': []}
			path_left_bound = {'x': [], 'y': []}
			path_rigth_bound = {'x': [], 'y': []}
			data = pd.read_csv(os.path.join(csv_directory, file))
			path['x'].extend(data['x'].to_list())
			path['y'].extend(data['y'].to_list())
			path['yaw'].extend(data['yaw'].to_list())
			path_left_bound['x'].extend(data['left_x'].to_list())
			path_left_bound['y'].extend(data['left_y'].to_list())
			path_rigth_bound['x'].extend(data['right_x'].to_list())
			path_rigth_bound['y'].extend(data['right_y'].to_list())
			new_path = self.segment_path2(path=path, segment_length=5)
			df = pd.DataFrame({'x': [float(v) for v in new_path['x']], 'y': [float(v) for v in new_path['y']], 'yaw': [float(v) for v in new_path['yaw']]})
			out_file = os.path.join(segment_dir, file)
			df.to_csv(out_file, index=False)

	
	# def calPositionAGV(self, name): 
	# 	pre_degree = this.state.robotModel.positionNum[2]
	# 	#   coor= '-186.87,-8.32,0.57'
	# 	#   coor= '0,0,0'

	# 	# console.log(this.imageHeightY,y);
	# 	rawPose = coor.split(",")
	# 	x = Number(rawPose[0]) * -Math.cos(-0.082) - Number(rawPose[1]) * -Math.sin(-0.082)
	# 	y = Number(rawPose[0]) * -Math.sin(-0.082) + Number(rawPose[1]) * -Math.cos(-0.082)
	# 	positionX = (((x + 45) / 996.782) * 100).toFixed(3) + '%'	#width :1016.8 height: 598.9952
	# 	positionY = (((y + 270) / 586.10) * 100).toFixed(3) + '%'
	# 	degree = ((Number(rawPose[2]) - 0.082) * -180) / Math.PI
	# 	if (prev_deg.current[name] == undefined):
	# 		prev_deg.current[name] = 0.0
	# 	delta = degree - prev_deg.current[name]
	# 	if (delta >= 180) {delta = delta - 360;} else if (delta <= -180) {
	# 	delta = delta + 360}
	# 	prev_deg.current[name] = prev_deg.current[name] + delta
	# 	return [positionX, positionY, prev_deg.current[name].toString()]

	
	# def segment_path(self, path, segment_length):
	# 	segment_path = []
	# 	path_reform = [[x, y] for x, y in zip(path['x'], path['y'])]
	# 	segment_path.append(path_reform[0])
	# 	current_point = np.array(path_reform[0])
	# 	for next_point in path_reform[1:]:
	# 		next_point = np.array(next_point)
	# 		distance = np.linalg.norm(next_point - current_point)
	# 		while distance >= segment_length:
	# 			direction = (next_point - current_point) / distance
	# 			new_point = current_point + direction * segment_length
	# 			segment_path.append(new_point.tolist())
	# 			current_point = new_point
	# 			distance = np.linalg.norm(next_point - current_point)
	# 		current_point = next_point
	# 	if not np.array_equal(segment_path[-1], path_reform[-1]):
	# 		segment_path.append(path_reform[-1])
	# 	for id in range(1, len(segment_path)):
	# 		dist = np.linalg.norm(np.array(segment_path[id]) - np.array(segment_path[id-1]))
	# 		print(dist)
	# 	return segment_path


def main(args=None):
	rclpy.init(args=args)
	node = PathWithLaneIDSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

def segment_all(args=None):
	rclpy.init(args=args)
	node = PathWithLaneIDSubscriber()
	node.segment_all_path()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	segment_all()
	# main()