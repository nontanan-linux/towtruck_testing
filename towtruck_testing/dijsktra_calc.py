#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autoware_adapi_v1_msgs.srv import SetRoutePointsWithId 
from towtruck_msgs.msg import RobotState
import json
import re
from collections import defaultdict 
from itertools import permutations
import requests,json

class DijsktraCalculation(Node):
	def __init__(self):
		super().__init__('Dijkstra_calc_node')
		self.dijk_subscription = self.create_subscription(String, 'dijkstra/request', self.dijk_callback, 10)
		self.dijk_publisher = self.create_publisher(String, 'dijkstra/path', 10)
		self.state_publisher = self.create_publisher(RobotState, '/mission_control/robot_state', 10)
		self.overtake_client = self.create_client(SetRoutePointsWithId, "/planning/mission_planning/set_route_points_with_id")
		self.edges = defaultdict(list)
		self.robot_msgs = RobotState()
		self.weights = {}
		edges = []
		self.start = None
		self.vias = None
		uni_edges = []
		self.init_edges()
		for uni_edge in uni_edges:
			self.add_edge_uni(*uni_edge)
		self.get_logger().info('DijkstraNode started. Listening on /dijkstra/request')

	def overtake_service_request(self,node_list):
		overtake_request = SetRoutePointsWithId.Request()
		overtake_request.ids = node_list  # List of route point IDs
		overtake_request.mode = True  # Set the mode to True
		future = self.overtake_client.call_async(overtake_request)
		future.add_done_callback(self.overtake_response_callback)

	def overtake_response_callback(self, future):
		try:
			# Get the response from the service
			response = future.result()
			self.get_logger().info(f'Service response: {response}')
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def fix_and_parse(self, s):
		s = re.sub(r'\[([^\]]+)\]', lambda m: '[' + ','.join(f'"{x.strip()}"' for x in m.group(1).split(',')) + ']', s)
		s = re.sub(r':\s*([A-Za-z0-9_]+)', r': "\1"', s)
		return json.loads(s)

	def dijk_callback(self, msg):
		try:
			msg_data = re.sub(r'(\w+):', r'"\1":', msg.data)
			msg_dict = self.fix_and_parse(s=msg_data)
			self.start = msg_dict["start"]
			self.vias = msg_dict["via"]
			print(f"start: {self.start}, type: {type(self.start)}")
			print(f"vias: {self.vias}, type: {type(self.vias)}")
			path_calc = self.CalPathWithVia(start=self.start, vias=self.vias)[1]
			path_str = ', '.join(path_calc)
			print(f'Path str: {path_str}')
			self.robot_msgs.path = path_str
			self.overtake_service_request(path_calc)
			self.state_publisher.publish(self.robot_msgs)
		except Exception as e:
			self.get_logger().error(f"Error parsing request: {e}")
	# ros2 topic pub /dijkstra/request std_msgs/String "{data: '{"start": "H01S", "via": [D01S,D04S]}'}"

	def main(self):
		self.overtake_service_request([self.overtake_point[self.robot_node_status['reserving_path'][0][1]],self.path_progression[0][-1]])

	def add_edge(self, from_node, to_node, weight):
		#bi-directional
		self.edges[from_node].append(to_node)
		self.edges[to_node].append(from_node)
		self.weights[(from_node, to_node)] = weight
		self.weights[(to_node, from_node)] = weight

	def add_edge_uni(self, from_node, to_node, weight):
		#uni-directional
		self.edges[from_node].append(to_node)
		self.weights[(from_node, to_node)] = weight

	def init_edges(self):		
		edges = []
		uni_edges = [
			('H01S','H01N',1),
			('H02S','H01N',1),
			('H01N','H02N',1),
			('H02N','H03N',1),
			('H03N','H04N',1),
			('H04N','H05N',1),
			('H05N','D15S',1),
			('H05N','P0504N',1),
			('D15S','D21S',1),
			('D21S','D1501N',1),
			('D15S','D1501N',1),
			('D1501N','D22S',1),
			('D22S','D2201N',1),
			('D22S','P01S',1),
			('D1501N','D2201N',1),
			('D1501N','P01S',1),
			('D2201N','D2202N',1),
			('D2202N','D2203N',1),
			('D2203N','P06S',1),
			('P06S','P0601N',1),
			('P0601N','D16S',1),
			('D16S','D2201N',1),
			('D16S','P01S',1),
			('P01S','P02S',1),
			('D21S','P02S',1),
			('D15S','P02S',1),
			('P02S','P03S',1),
			('P03S','P05S',1),
			('P05S','P0501N',1),
			('P0501N','P0502N',1),
			('P0502N','P0503N',1),
			('P0503N','H01S',1),
			('P0503N','H02S',1),
			('P05S','P0504N',1),
			('P0504N','P0505N',1),
			('P0505N','D01S',1),
			('P0505N','D11A',1),
			('P0505N','D11S',1),
			('D01S','D0117N',1),
			('D0117N','D17A',1),
			('D0117N','D17S',1),
			('D17A','P0502N',1),
			('D17S','P0502N',1),
			('D17S','H05N',1),
			('D17A','H05N',1),
			('D01S','D02S',1),
			('D02S','D03S',1),
			('D03S','D0301N',1),
			('D0301N','D04A',1),
			('D0301N','D04S',1),
			('D04A','D0401N',1),
			('D04S','D0401N',1),
			('D0401N','D05S',1),
			('D0401N','D05A',1),
			('D05A','D06A',1),
			('D05A','D06S',1),
			('D05S','D06A',1),
			('D05S','D06S',1),
			('D06A','D0601N',1),
			('D06S','D0601N',1),
			('D0601N','D11S',1),
			('D0601N','D11A',1),
			('D11A','D12A',1),
			('D11A','D12S',1),
			('D11S','D12A',1),
			('D11S','D12S',1),
			('D12A','P04S',1),
			('D12S','P04S',1),
			('D04A','D07A',1),
			('D04A','D07S',1),
			('D04S','D07A',1),
			('D04S','D07S',1),
			('D07A','D08A',1),
			('D07A','D08S',1),
			('D07S','D08A',1),
			('D07S','D08S',1),
			('D08A','D0801N',1),
			('D08S','D0801N',1),
			('D0801N','D09A',1),
			('D0801N','D09S',1),
			('D09A','D10A',1),
			('D09A','D10S',1),
			('D09S','D10A',1),
			('D09S','D10S',1),
			('D10A','D1001N',1),
			('D10S','D1001N',1),
			('D1001N','P04S',1),
			('P04S','P0401N',1),
			('P0401N','D18A',1),
			('P0401N','D18S',1),
			('D18A','D19A',1),
			('D18A','D19S',1),
			('D18S','D19A',1),
			('D18S','D19S',1),
			('D19A','D1901N',1),
			('D19S','D1901N',1),
			('D1901N','D15S',1),
			('P04S','D13S',1),
			('D13S','D14S',1),
			('D14S','D1401N',1),
			('D1401N','D1402N',1),
			('D1402N','D20S',1),
			('D20S','D15S',1),
			('P0401N','P07S',1), #like a P0401N to D18S.
			('P07S','P08S',1), #Like a D18S to D19S.
			('P07S','D19A',1), #Like a D18S to D19A.
			('P07S','D19S',1), #Like a D18S to D19S.
			('P08S','D1901N',1), #Like a D19S to D1901N.
			('D18S','P08S',1), #Like a D18S to D19S
			('D18A','P08S',1), #Like a D18A to D19S
		]
		for edge in edges:
			self.add_edge(*edge)
		for uni_edge in uni_edges:
			self.add_edge_uni(*uni_edge)

	def cal_path(self, initial, end):
		# shortest paths is a dict of nodes
		# whose value is a tuple of (previous node, weight)
		shortest_paths = {initial: (None, 0)}
		current_node = initial
		visited = set()
		while current_node != end:
			visited.add(current_node)
			destinations = self.edges[current_node]
			weight_to_current_node = shortest_paths[current_node][1]
			#print ('*****',current_node,'*****')
			#print ('destinations',destinations)
			for next_node in destinations:
				weight = self.weights[(current_node, next_node)] + weight_to_current_node
				if next_node not in shortest_paths:
					shortest_paths[next_node] = (current_node, weight)
				else:
					current_shortest_weight = shortest_paths[next_node][1]
					if current_shortest_weight > weight:
						shortest_paths[next_node] = (current_node, weight)
						#print ('weight changed')
				# print ('next',next_node,shortest_paths)
			next_destinations = {node: shortest_paths[node] for node in shortest_paths if ((node not in visited))}
			#print next_destinations
			if not next_destinations:
				return (["Route Not Possible","Route Not Possible"],0)
			# next node is the destination with the lowest weight
			current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
			#print weight
		# Work back through destinations in shortest path
		path = []
		all_weight = shortest_paths[current_node][1]
		while current_node is not None:
			path.append(current_node)
			next_node = shortest_paths[current_node][0]
			current_node = next_node
		# Reverse path
		path = path[::-1]
		return (path,all_weight)

	def CalPathWithVia(self, start, vias):
		min_path = None
		section_min_path = None
		min_weight = float('inf')
		min_node = None
		equal_list = []
		current_path_detail = None
		min_path_detail = None
		# Generate all permutations of the via points
		print(permutations(vias))
		for perm in permutations(vias):
			# print('perm',perm)
			current_path = [start]
			section_path = []
			section_weight = []
			current_weight = 0
			# Calculate the path and weight for each segment in the permutation
			for i in range(len(perm)):
				segment_start = current_path[-1]
				segment_end = perm[i]
				segment_path, segment_weight = self.cal_path(segment_start, segment_end)
				if len(segment_path)>1:
					current_path.extend(segment_path[1:])
					section_path.append(segment_path[1:])
					current_weight += segment_weight
					section_weight.append(segment_weight)
			###############################################################
			current_path_detail = (list(perm),current_path,section_path,current_weight,section_weight)
			###############################################################
			if not equal_list:
				equal_list = [current_path_detail]
			# Update minimum path and weight if this permutation is shorter
			if min_path_detail:
				if current_path_detail[3] < min_path_detail[3]:
					min_path_detail = current_path_detail
					equal_list = [current_path_detail]
					# min_node = list(perm)
					# min_path = current_path
					# section_min_path = section_path
					# min_weight = current_weight
				elif current_path_detail[3] == min_path_detail[3]:
					equal_list.append(current_path_detail)
			else:
				min_path_detail = current_path_detail
		# print("dijk",equal_list)
		while len(equal_list) > 1:
			for i in range(len(equal_list[0][4])):
				if equal_list[0][4][i]>equal_list[1][4][i]:
					equal_list.pop(0)
					break
		min_node, min_path,section_min_path, min_weight = equal_list[0][0],equal_list[0][1],equal_list[0][2],equal_list[0][3]
		return (min_node, min_path,section_min_path, min_weight)

# ros2 topic echo /robot_state
# header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# state: 3
# path: H02S, H01N, H02N, H03N, H04N, H05N, D15S, D1501N, P01S
# ---

def test(args=None):
	rclpy.init(args=args)
	dji_calc = DijsktraCalculation()
	path = dji_calc.CalPathWithVia(start='D01S', vias=['D01S','D17S', 'H01S'])
	# path = dji_calc.CalPathWithVia(start='H01S', vias=['H03N', 'H04N', 'H05N', 'D15S', 'D1501N', 'P01S'])
	path_calc = path[1]
	path_str = ', '.join(path_calc)
	print(path_calc, type(path_calc))
	print(path_str, type(path_str))

# test()

def main(args=None):
	rclpy.init(args=args)
	node = DijsktraCalculation()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()