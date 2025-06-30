import pandas as pd
import os
import numpy as np
from math import asin, atan2

class TowtruckEditGoalPoints:
	def __init__(self):
		self.csv_goal_path = os.path.expanduser('~/autoware.bg2/data/BG/GoalPoints.csv')
		self.csv_goal_edit = os.path.expanduser('~/Documents/towtruck-check-station.csv')
		self.edit_list = []

	def get_distance(self, in_point, fi_point):
		return np.sqrt(np.power(fi_point[0] - in_point[0], 2) + np.power(fi_point[1] - in_point[1], 2))

	def get_df(self, csv_path):
		return pd.read_csv(csv_path)
	
	def euler_from_quaternion(self, qx, qy, qz, qw):
		x, y, z, w = qx, qy, qz, qw
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = atan2(sinr_cosp, cosr_cosp)
		sinp = 2 * (w * y - z * x)
		pitch = asin(sinp)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / np.pi)
		return roll, pitch, yaw
	
	def shift_point_forward(self, x, y, yaw_deg, distance):
		yaw_rad = np.radians(yaw_deg)
		new_x = x + distance * np.cos(yaw_rad)
		new_y = y + distance * np.sin(yaw_rad)
		return new_x, new_y
	
	def update_edit_points(self, ori_data, edit_data):
		matching_names = edit_data['name'][edit_data['name'].isin(ori_data['name'])]
		edit_list = []
		for name in matching_names:
			raw_data = ori_data[ori_data['name'] == name].iloc[0].copy()
			row_edit = edit_data[edit_data['name'] == name].iloc[0]
			if row_edit['point_dist'] != 0.0:
				raw_data['x'] = row_edit['new_x']
				raw_data['y'] = row_edit['new_y']
				edit_list.append(raw_data)
		return pd.DataFrame(edit_list)

	def main(self):
		goal_df = self.get_df(self.csv_goal_path)
		edit_df = self.get_df(self.csv_goal_edit)
		save_df = goal_df.copy()

		matching_names = edit_df['station'][edit_df['station'].isin(goal_df['name'])]
		for name in matching_names:
			goal_row = goal_df[goal_df['name'] == name].iloc[0]
			edit_row = edit_df[edit_df['station'] == name].iloc[0]
			roll, pitch, yaw = self.euler_from_quaternion(goal_row['qx'], goal_row['qy'], goal_row['qz'], goal_row['qw'])
			old_x, old_y = goal_row['x'], goal_row['y']
			new_x, new_y = self.shift_point_forward(old_x, old_y, yaw, edit_row['dist'])
			dist = self.get_distance((old_x, old_y), (new_x, new_y))
			frame = {
				"name": name,
				'old_x': old_x,
				'old_y': old_y,
				'old_qx': goal_row['qx'],
				'old_qy': goal_row['qy'],
				'old_qz': goal_row['qz'],
				'old_qw': goal_row['qw'],
				'edit': [edit_row['dist'], edit_row['ship']],
				'new_x': new_x,
				'new_y': new_y,
				'new_qx': goal_row['qx'],
				'new_qy': goal_row['qy'],
				'new_qz': goal_row['qz'],
				'new_qw': goal_row['qw'],
				'point_dist': dist
			}
			self.edit_list.append(frame)
		self.update_df = pd.DataFrame(self.edit_list)
		self.update_goal = self.update_edit_points(ori_data=save_df, edit_data=self.update_df)
		for item in self.edit_list:
			print(f"{item['name']} shifted by {item['point_dist']:.3f} meters x:{item['new_x']}, y:{item['new_y']}")
		save_path = os.path.expanduser('~/Documents/GoalPoints_Edited.csv')
		self.update_goal.to_csv(save_path, index=False)
		print(f"Edited goal points saved to {save_path}")

def main():
	node = TowtruckEditGoalPoints()
	node.main()

if __name__ == "__main__":
	main()
