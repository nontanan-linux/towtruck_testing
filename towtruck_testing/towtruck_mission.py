import Dijsktra_calculation
import os
import time
import math
import pandas as pd
import numpy as np
import requests
import asyncio
import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import Engage
from autoware_adapi_v1_msgs.srv import SetRoutePointsWithId
from towtruck_msgs.msg import MissionCmd
from towtruck_msgs.srv import MissionService
from towtruck_msgs.action import TowtruckMission

class RoutePointsClient(Node):
	def __init__(self):
		super().__init__('route_points_client')
		self.declare_parameter("csv_goal_path", "data/BG/GoalPoints.csv")
		self.csv_goal_path = self.get_parameter("csv_goal_path").get_parameter_value().string_value
		self.csv_goal_path = os.path.join(os.getcwd(), self.csv_goal_path)
		self.goal_data_frame = pd.read_csv(self.csv_goal_path)
		self.kinematic = Odometry()
		self.mission_cmd = MissionCmd()
		self.engage_cmd = Engage()
		self.engage_msg = Engage()
		self.current_node = None
		self.d_path = []
		self.pick_point = ''
		self.drop_point = []
		self.pick_progress = 0
		self.drop_progress = [0, 0, 0, 0]
		self.activate_mission = False
		self.mission_fleg = False
		self.towtruck_mission = ActionServer(self, TowtruckMission, 'towtruck/mission', self.execute_towtruck_mission)
		self.server = self.create_service(MissionService, 'towtruck/mission', self.service_towtruck_mission)
		self.client = self.create_client(SetRoutePointsWithId, '/planning/mission_planning/set_route_points_with_id')
		self.kinematic_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.kinematic_callback, 10)
		self.mission_sub = self.create_subscription(MissionCmd, '/towtruck/mission', self.mission_callback, 10)
		self.engage_sub = self.create_subscription(Engage, 'autoware/engage', self.engage_callback, 10)
		self.engage_pub = self.create_publisher(Engage, 'autoware/engage', 10)
		self.main_time = self.create_timer(0.1, self.main)
		
	def initial_state(self):
		start_time = time.time()
		while time.time()-start_time <= 1.5:
			self.get_logger().info('initial state')
		return self.CallNearestNode()
	
	def engage_callback(self, msg):
		self.engage_msg = msg
	
	def service_towtruck_mission(self, request, response):
		try:
			self.pick_point = request.pick
			self.drop_point = [request.drop1, request.drop2, request.drop3, request.drop4]
			response.success = True
			response.message = f'send message pick {self.pick_point} to {self.drop_point}'
			self.get_logger().info(f'Service Towtruck Mission: {response.message}')
			self.activate_mission = True
			return response
		except Exception as error:
			response.success = False
			response.message = error
			return response
	
	def mission_callback(self, msg):
		self.mission_cmd = msg
		self.d_path = Dijsktra_calculation.Dijsktra().cal_path(self.mission_cmd.pick, self.mission_cmd.drop)[0]
		self.send_request(self.d_path)
		if self.mission_fleg:
			self.engage_cmd.stamp = self.get_clock().now().to_msg()
			self.engage_cmd.engage = True
			self.engage_pub.publish(self.engage_cmd)
			# self.mission_fleg = False

	def execute_towtruck_mission(self, goal_handle):
		pick = goal_handle.request.pick
		drop = goal_handle.request.drop1
		feedback_msg = TowtruckMission.Feedback()
		result = TowtruckMission.Result()
		feedback_msg.pick_progress = 0
		feedback_msg.drop1_progress = 0
		feedback_msg.drop2_progress = 0
		feedback_msg.drop3_progress = 0
		feedback_msg.drop4_progress = 0
		feedback_msg.current_node = self.current_node
		result.success = False
		if not result.success:
			feedback_msg.current_node = self.current_node  # Update current node dynamically
			# Move towards pick location
			if self.current_node != pick:
				self.activate_path([self.current_node, pick])
			else:
				feedback_msg.pick_progress = 1  # Mark pick-up as complete
			# If pick-up is done, move towards drop location
			if feedback_msg.pick_progress == 1:
				if self.current_node != drop:
					self.activate_path([self.current_node, drop])
				else:
					feedback_msg.drop1_progress = 1  # Mark drop-off as complete
			goal_handle.publish_feedback(feedback_msg)
			if feedback_msg.pick_progress == 1 and feedback_msg.drop1_progress == 1:
				result.success = True
				time.sleep(0.5)  # Prevent CPU overuse (adjust if needed)
		goal_handle.succeed()
		result.message = "Towtruck mission completed successfully!"
		return result
	
	def activate_path(self, mission):
		try:
			self.d_path = Dijsktra_calculation.Dijsktra().cal_path(mission[0], mission[1])[0]
			self.get_logger().debug(f'activate_path: {self.d_path}')
			self.send_request(self.d_path)
			self.get_logger().info("Engage Vehicle")
			self.engage_cmd.stamp = self.get_clock().now().to_msg()
			self.engage_cmd.engage = True
			for i in range(0, 3):
				self.engage_pub.publish(self.engage_cmd)
		except Exception as error:
			self.get_logger().error(f'Error: {error}')

	def kinematic_callback(self, msg):
		self.kinematic = msg

	def main(self):
		if self.current_node is None:
			self.current_node = self.CallNearestNode()
		x = self.kinematic.pose.pose.position.x
		y = self.kinematic.pose.pose.position.y
		linear_x = self.kinematic.twist.twist.linear.x
		self.CallCurrentNode()

		if self.activate_mission and (self.pick_point != '') and (self.drop_point != []):
			if self.current_node != self.pick_point and self.pick_progress == 0:
				if not self.d_path or (self.d_path[-1] != self.pick_point):  # Check if d_path is not empty
					self.activate_path([self.current_node, self.pick_point])
					self.get_logger().info(f'go to pick path: {self.d_path}, mission: {self.mission_fleg}, engage: {self.engage_cmd.engage}')
			elif self.current_node == self.pick_point and not self.engage_msg.engage and self.pick_progress == 0:
				self.get_logger().info('pick complete')
				self.pick_progress = 1

			if self.pick_progress == 1 and (self.drop_progress != [1, 1, 1, 1]):
				print('start to drop')
				if self.current_node != self.drop_point[0] and self.drop_progress[0] == 0:
					if not self.d_path or self.d_path[-1] != self.drop_point[0]:  # Safe check for empty list
						print('go to drop 1')
						self.activate_path([self.current_node, self.drop_point[0]])
				elif self.current_node == self.drop_point[0] and not self.engage_msg:
					self.get_logger().info("set progress drop1")
					self.drop_progress = [1, 0, 0, 0]

				if self.current_node != self.drop_point[1] and self.drop_progress[0] == [1,0,0,0]:
					if not self.d_path or self.d_path[-1] != self.drop_point[1]:  # Safe check for empty list
						print('go to drop 2')
						self.activate_path([self.current_node, self.drop_point[1]])
				elif self.current_node == self.drop_point[1] and not self.engage_msg:
					self.get_logger().info("set progress drop1")
					self.drop_progress = [1, 1, 1, 1]

	def send_request(self, path):
		request = SetRoutePointsWithId.Request()
		request.ids = path
		request.mode = True  
		future = self.client.call_async(request)
		future.add_done_callback(self.response_callback)

	def response_callback(self, future):
		try:
			response = future.result()
			self.mission_fleg = response.status.success
			self.get_logger().info(f'Response: {response}, update mission fleg: {self.mission_fleg}')
		except Exception as e:
			self.get_logger().error(f'Service call failed: {str(e)}')
	
	def calc_heading(self, quaternion):
		return tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
	
	def CallCurrentNode(self):
		try:
			if self.d_path:
				for node in self.d_path:
					current_data_frame = self.goal_data_frame.loc[self.goal_data_frame['station'] == node]
					if current_data_frame.empty:
						self.get_logger().warn(f"No matching station found for node: {node}")
						continue  # Skip this iteration
					sub_goal = np.array([
						float(current_data_frame["x"].iloc[0]), 
						float(current_data_frame["y"].iloc[0])
					])
					abs_distance_incoming = self.distance_pt_to_frame(self.kinematic.pose.pose.position, current_data_frame)
					if abs_distance_incoming < 5.0:
						self.current_node = node
						self.current_node_position = sub_goal
		except Exception as txt:
			self.get_logger().error(f"CallCurrentNode error: {txt}")


	def CallNearestNode(self):
		stations = self.goal_data_frame["station"].tolist()
		distances = []
		for station in stations:
			current_data_frame = self.goal_data_frame.loc[self.goal_data_frame["station"] == station]
			if current_data_frame.empty:
				continue  # Skip empty data
			dist = self.distance_pt_to_frame(self.kinematic.pose.pose.position, current_data_frame)
			distances.append(dist)
		if not distances:
			return None
		min_index = distances.index(min(distances))  # Get index of min distance
		return stations[min_index]  # Return the nearest station
	
	def distance_pt_to_frame(self, curr_pose, goal_pose):
		try:
			goal_arr = np.array([float(goal_pose["x"]), float(goal_pose["y"])])
			curr_arr = np.array([float(curr_pose.x), float(curr_pose.y)])
			return np.linalg.norm(goal_arr - curr_arr)  # More efficient distance calculation
		except KeyError:
			print("Error: Missing x or y coordinate in goal_pose.")
			return float('inf')  # Return a large number to ignore invalid entries

def main():
	rclpy.init()
	node = RoutePointsClient()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Shutting down...")
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()