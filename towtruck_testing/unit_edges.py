#!/usr/bin/env python3
import os
import subprocess
from collections import defaultdict 
import requests,json
import csv
import rclpy
from rclpy.node import Node

customer_uni_edges = [
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
			('D15S','P03S',1),
			('P02S','P03S',1),
			('P03S','P05S',1),
			('P03S','P0501N',1),
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

class DijsktraCalc():
	def __init__(self,main_server_ip,main_server_port,reload=True):
		# self.edges = defaultdict(list)
		self.edges_list = []
		self.weights = {}
		edges = []
		uni_edges = []
		csv_path = "/home/nontanan/ros2_ws/src/towtruck_testing/resource/towtruck_path.csv"
		if reload:
			r = requests.get("http://"+main_server_ip+":"+main_server_port+"/path/paths",params={'page':1,'page_size':200})
			self.paths =json.loads(r.text)['payload']
			self.edges = self.initial_edges_from_api(self.paths)
			# for path in self.paths:
			# 	uni_edges.append((path['start_node'],path['to_node'],path['path_weight']))
			# for uni_edge in uni_edges:
			# 	self.add_edge_uni(*uni_edge)
		else:
			all_path = self.read_path_from_csv(file_path=csv_path)
			self.edges = self.initial_edges(all_path)
	
	def add_edge_uni(self, from_node, to_node, weight):
		#uni-directional
		self.edges[from_node].append(to_node)
		self.weights[(from_node, to_node)] = weight
	
	def read_path_from_csv(self, file_path):
		with open(file_path, mode='r', newline='', encoding='utf-8-sig') as file:
			reader = csv.DictReader(file)
			return [row for row in reader]
	
	def initial_edges(self, all_path):
		edges = []
		for path in all_path:
			edge = (path['start_node'], path['to_node'], path['path_weight'])
			edges.append(edge)
		return edges
	
	def initial_edges_from_api(self, path):
		edges = []
		for node in path:
			# print(node['start_node'], node['to_node'], node['path_weight'])
			edge = (node['start_node'], node['to_node'], node['path_weight'])
			edges.append(edge)
		return edges


def dji_calc(args=None):
	rclpy.init(args=args)
	try:
		dji_calc = DijsktraCalc(main_server_ip="192.168.1.114", main_server_port="5010",reload=False)
		# dji_calc.initial_edges_from_api(dji_calc.paths)
		# print(dji_calc.edges)
		# print(type(dji_calc.edges))
		for node in dji_calc.edges:
			print(node)
		# print(dji_calc.edges)
		# for node in dji_calc.edges:
		# 	print(node)
		# print(len(dji_calc.edges_list))
	except Exception as err:
		print(f'Error: {err}')
	finally:
		rclpy.shutdown()


def main(args=None):
	rclpy.init(args=args)
	try:
		save_path = "/home/nontanan/ros2_ws/src/towtruck_testing/towtruck_testing/pict/21052025/segment_5m"
		result = subprocess.run(f'ls {save_path} | grep ".csv"', shell=True, stdout=subprocess.PIPE, text=True)
		csv_files = result.stdout.strip().split('\n') if result.stdout else []
		csv_files = [f for f in csv_files if f.strip()]
		print(f"CSV files found:\n{csv_files}")
		print(f"Number of CSV files: {len(csv_files)}")
		print(f'Number of path split: {len(customer_uni_edges)}')
	except Exception as error:
		print(f'Error: {error}')
	finally:
		rclpy.shutdown()

# def main():
#     save_path = "/home/nontanan/ros2_ws/src/towtruck_testing/towtruck_testing/pict/21052025/segment_5m"
#     result = subprocess.run(f'ls {save_path} | grep ".csv"', shell=True, stdout=subprocess.PIPE, text=True)
#     csv_files = result.stdout.strip().split('\n') if result.stdout else []
#     csv_files = [f for f in csv_files if f.strip()]
#     print(f"CSV files found:\n{csv_files}")
#     print(f"Number of CSV files: {len(csv_files)}")
#     print(f'Number of path split: {len(customer_uni_edges)}')

if __name__ == "__main__":
	# main()
	dji_calc()