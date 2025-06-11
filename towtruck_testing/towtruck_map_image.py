#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import cv2
import os
import datetime
import csv
from matplotlib.legend_handler import HandlerBase
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D
from unit_edges import customer_uni_edges

class HandlerTextBox(HandlerBase):
    def __init__(self, text, **kw):
        super().__init__(**kw)
        self.text = text

    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        box = Rectangle([xdescent, ydescent], width, height,
                        facecolor=(1.0, 0.0, 0.5), edgecolor='none', transform=trans)
        txt = plt.Text(x=xdescent + width / 2,
                       y=ydescent + height / 2,
                       text=self.text,
                       color='white',
                       fontsize=fontsize*0.5,
                       ha='center',
                       va='center',
                       transform=trans)
        return [box, txt]

class MapImageOutline:
	def __init__(self,):
		self.compare_img_path = '/home/nontanan/Pictures/bg_map-compare.png'
		self.map_rviz_path = '/home/nontanan/Pictures/bg_map-rviz.png'
		self.map_layout_path = '/home/nontanan/Pictures/bg_map-img.png'
		self.goalpoints_path = '/home/nontanan/ros2_ws/src/towtruck_testing/csv/GoalPoints.csv'
		self.hornpoints_path = '~/autoware.bg2/data/BG/HornPoints.csv'
		self.csv_directory = '/home/nontanan/ros2_ws/src/towtruck_testing/resource/2025-06-09'
		self.save_image = False
		self.horn_info = False
		self.segment_dir = self.csv_directory #os.path.join(self.csv_directory, 'segment_5m')
		self.map_image = cv2.imread(self.compare_img_path)
		self.rviz_image = cv2.imread(self.map_rviz_path)
		self.layout_image = cv2.imread(self.map_layout_path)
		self.layout_image = cv2.cvtColor(self.layout_image, cv2.COLOR_BGR2RGB)
		self.hornpoints_color = (255/255, 255/255, 0.0/255)
		self.goalpoints_color = (0.0/255, 255/255, 0.0/255)
		self.trajectory_color = (255/255, 0.0/255, 0.0/125)
		self.pix_dist = [[1207,577], [1207,383], [1225,232], [1227,419]]
		self.map_dist = [[561.25,217.58], [561.55,179.20],[568.08,108.73],[569.78,191.93]]
		self.map_yaw = -0.09
		if self.map_image is None:
			raise FileNotFoundError(f"Not found img {self.compare_img_path}")
		if self.map_rviz_path is None:
			raise FileNotFoundError(f"Not found img {self.map_rviz_path}")
		if self.map_layout_path is None:
			raise FileNotFoundError(f"Not found img {self.map_layout_path}")
		self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2RGB)
		self.img_height, self.img_width, _ = self.map_image.shape
		# self.fig = plt.figure(figsize=(16, 9)) 
		self.coordinate = [0.0,0.0,0.0]
		self.legend_elements = [
			Line2D([0], [0], color=self.trajectory_color, lw=2, label='Trajectory path of the vehicle'),
			Line2D([0], [0], marker='o', color='red', label='Goalpoints', markerfacecolor=self.goalpoints_color, markersize=6),
			Rectangle((0,0),1,1, label='Station Name')] # Patch(facecolor=(1.0, 0.0, 0.5), edgecolor='none', label='Station Name')
		self.x = self.coordinate[0]*np.cos(self.map_yaw) - self.coordinate[1]*-np.sin(self.map_yaw)
		self.y = self.coordinate[0]*-np.sin(self.map_yaw) + self.coordinate[1]*-np.cos(self.map_yaw)  
		self.offset = {"x": 835.0, "y": 272.0}
		self.x_map_ratio, self.y_map_ratio = self.cal_map_resolution(pixel=self.pix_dist, map=self.map_dist)
		self.goalpoints = self.get_goalpoints(self.goalpoints_path)
		self.hornpoints = self.get_hornpoints(self.hornpoints_path)
		self.csv_path = [f for f in os.listdir(self.segment_dir) if f.endswith('.csv')]
		self.time_now = datetime.datetime.now()
		print(f"image resolution: width = {self.img_width}, height: {self.img_height}")
		print(f"Map Resolution x:{self.x_map_ratio} y:{self.y_map_ratio}")
	
	def cal_map_resolution(self, pixel, map):
		x_res, y_res = [],[]
		if len(pixel) == len(map):
			for i in range(0, len(pixel)):
				x_res.append(pixel[i][0]/map[i][0])
				y_res.append(pixel[i][1]/map[i][1])
			return np.mean(x_res), np.mean(y_res[1:])
		else:
			return
	
	def get_goalpoints(self, path):
		try:
			df = pd.read_csv(path)
			df = df[df["node_type"] == "station"].copy()
			if not {"name", "x", "y"}.issubset(df.columns):
				raise ValueError("CSV file must contain 'name', 'x', and 'y' columns")
			df['x'], df['y'] = self.transfrom(position=[df['x'], df['y']])
			return df[["name", "x", "y", "position"]].values
		except Exception as get_goal_err:
			print(f'Get Goal Points Error: {get_goal_err}')
			return None
	
	def get_hornpoints(self, path):
		try:
			df = pd.read_csv(path)
			if not {"station", "node", "x", "y", "z", "qx", "qy", "qz", "qw"}.issubset(df.columns):
				raise ValueError("CSV file must contain 'name', 'x', and 'y' columns")
			df['x'], df['y'] = self.transfrom(position=[df['x'], df['y']])
			return df[["station", "node", "x", "y", "z", "qx", "qy", "qz", "qw"]].values
		except Exception as get_horn_err:
			print(f'Get Horn Points Error: {get_horn_err}')
			return None
	
	def transfrom(self, position):
		tf_x = position[0]*np.cos(self.map_yaw) - position[1]*np.sin(self.map_yaw)
		tf_y = position[0]*-np.sin(self.map_yaw) + position[1]*-np.cos(self.map_yaw)
		tf_x = (tf_x + self.offset["x"])*self.x_map_ratio
		tf_y = (tf_y + self.offset["y"])*self.y_map_ratio
		return [tf_x,tf_y]

	def plot_map(self, target_map, target_goalpoints, target_hornpoints, trajectory_csv_path):
		plt.imshow(target_map)
		for path_file in trajectory_csv_path:
			full_path = os.path.join(self.segment_dir, path_file)
			path = pd.read_csv(full_path)
			x_list, y_list = [], []
			for idx in range(len(path)):
				x, y = self.transfrom(position=[path['x'][idx], path['y'][idx]])
				x_list.append(x)
				y_list.append(y)
			plt.plot(x_list, y_list, c=self.trajectory_color, linewidth=1.2, zorder=1)
		if self.horn_info:
			for i, (station, node, x, y, z, qx, qy, qz, qw) in enumerate(target_hornpoints):
				plt.scatter(x, y, c=self.hornpoints_color, s=12, label="Hornpoints" if i == 0 else "", zorder=2)
			self.legend_elements.append(Line2D([0], [0], marker='o', color='red', label='Hornpoints', markerfacecolor=self.hornpoints_color, markersize=6))
		for i, (name, x, y, position) in enumerate(target_goalpoints):
			text_x, text_y = x, y
			if position == 'under':
				text_y += 18
			elif position == 'top':
				text_y -= 10
			elif position == 'right':
				text_x += 30
			elif position == 'left':
				text_x -= 30
			elif position == 'left+under':
				text_x -= 16
				text_y += 18
			elif position == 'right+under':
				text_x += 30
				text_y += 18
			elif position == 'top+right':
				text_y += 10
				text_x += 50
			plt.text(text_x, text_y, name,fontsize=6, color='white', ha='center', fontweight='bold',
				bbox=dict(facecolor=(255/255, 0.0/255, 127/255), edgecolor='none', boxstyle='round,pad=0.2'))
			plt.scatter(x, y, c=self.goalpoints_color, s=12,label="Goalpoints" if i == 0 else "", zorder=2)
		print(f'Type of element: {type(self.legend_elements)}')
		plt.axis('off')
		plt.grid('off')
		plt.legend(handles=self.legend_elements,handler_map={self.legend_elements[-1]: HandlerTextBox("D01S")},loc='upper right')
		# plt.subplots_adjust(left=0.05, bottom=0.05, right=1.0, top=1.0)
		# mng = plt.get_current_fig_manager()
		# mng.full_screen_toggle()
		if self.save_image:
			plt.savefig(os.path.join(trajectory_csv_path, f'map_image_{self.time_now:%Y%m%d_%H%M%S}.png'), bbox_inches='tight')
		plt.show()
	
	def plot_rviz_map(self):
		plt.imshow(self.rviz_image)
		for path_file in self.csv_path:
			full_path = os.path.join(self.segment_dir, path_file)
			path = pd.read_csv(full_path)
			x_list, y_list = [], []
			for idx in range(len(path)):
				x, y = self.transfrom(position=[path['x'][idx], path['y'][idx]])
				x_list.append(x)
				y_list.append(y)
			plt.plot(x_list, y_list, c=(1.0, 0.0, 0.0), linewidth=1.2, zorder=1)
		for name, x, y, position in self.goalpoints:
			text_x, text_y = x, y
			if position == 'under':
				text_y += 18
			elif position == 'top':
				text_y -= 10
			elif position == 'right':
				text_x += 30
			elif position == 'left':
				text_x -= 30
			elif position == 'left+under':
				text_x -= 16
				text_y += 18
			elif position == 'top+right':
				text_y += 10
				text_x += 50
			plt.text(text_x, text_y, name, fontsize=6, color='white', ha='center', fontweight='bold',
					bbox=dict(facecolor=(255/255, 0.0/255, 127/255), edgecolor='none', boxstyle='round,pad=0.2'))
			plt.scatter(x, y, c=(0.0, 1.0, 0.0), s=12, label=name if name == self.goalpoints[0, 0] else "", zorder=2)
		plt.subplots_adjust(left=0.05, bottom=0.05, right=1.0, top=1.0)
		# plt.savefig(os.path.join(self.csv_directory, f'rviz_image_{self.time_now:%Y%m%d_%H%M%S}.png'), dpi=300, bbox_inches='tight')
		plt.show()
	
	def plot_layout_map(self):
		plt.imshow(self.layout_image)
		for path_file in self.csv_path:
			full_path = os.path.join(self.segment_dir, path_file)
			path = pd.read_csv(full_path)
			x_list, y_list = [], []
			for idx in range(len(path)):
				x, y = self.transfrom(position=[path['x'][idx], path['y'][idx]])
				x_list.append(x)
				y_list.append(y)
			plt.plot(x_list, y_list, c=(1.0, 0.0, 0.0), linewidth=1.2, zorder=1)
		for name, x, y, position in self.goalpoints:
			text_x, text_y = x, y
			if position == 'under':
				text_y += 18
			elif position == 'top':
				text_y -= 10
			elif position == 'right':
				text_x += 30
			elif position == 'left':
				text_x -= 30
			elif position == 'left+under':
				text_x -= 16
				text_y += 18
			elif position == 'right+under':
				text_x += 30
				text_y += 18
			elif position == 'top+right':
				text_y += 10
				text_x += 50
			plt.text(text_x, text_y, name, fontsize=6, color='white', ha='center', fontweight='bold',
					bbox=dict(facecolor=(255/255, 0.0/255, 127/255), edgecolor='none', boxstyle='round,pad=0.2'))
			plt.scatter(x, y, c=(0.0, 1.0, 0.0), s=10, label=name if name == self.goalpoints[0, 0] else "", zorder=2)
		plt.subplots_adjust(left=0.05, bottom=0.05, right=1.0, top=1.0)
		# plt.savefig(os.path.join(self.csv_directory, f'layout_image_{self.time_now:%Y%m%d_%H%M%S}.png'), dpi=300, bbox_inches='tight')
		plt.show()


def main():
	towtruck_map_outline = MapImageOutline()
	# so.plot_map(target_map=so.map_image)
	# so.plot_map(target_map=so.rviz_image)
	towtruck_map_outline.plot_map(target_map=towtruck_map_outline.layout_image, 
				target_goalpoints=towtruck_map_outline.goalpoints,
				target_hornpoints=towtruck_map_outline.hornpoints,
				trajectory_csv_path=towtruck_map_outline.csv_path)
	# so.plot_rviz_map()
	# so.plot_layout_map()

if __name__ == '__main__':
	# main()
	print(f'type of edges: {customer_uni_edges}')
	for idx in range(0, len(customer_uni_edges)):
		print(f'no {idx+1}, start: {customer_uni_edges[idx][0]}, goal: {customer_uni_edges[idx][1]}')