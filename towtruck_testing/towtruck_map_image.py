import cv2
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

class StationOutline:
    def __init__(self,):
        self.map_image_path = '/home/nontanan/Pictures/bg_map-compare.png'
        self.stations_path = '/home/nontanan/ros2_ws/src/towtruck_testing/resource/GoalPoints.csv'
        self.csv_directory = '/home/nontanan/ros2_ws/src/towtruck_testing/towtruck_testing/pict/07052025'
        self.segment_dir = os.path.join(self.csv_directory, 'segment_5m')
        self.map_image = cv2.imread(self.map_image_path)
        if self.map_image is None:
            raise FileNotFoundError(f"Not found img {self.map_image_path}")
        self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2RGB)
        self.img_height, self.img_width, _ = self.map_image.shape
        self.pix_dist = [[1207,577], [1207,383], [1225,232], [1227,419]]
        self.map_dist = [[561.25,217.58], [561.55,179.20],[568.08,108.73],[569.78,191.93]]
        self.map_yaw = -0.09
        self.coordinate = [0.0,0.0,0.0]
        self.x = self.coordinate[0]*np.cos(self.map_yaw) - self.coordinate[1]*-np.sin(self.map_yaw)
        self.y = self.coordinate[0]*-np.sin(self.map_yaw) + self.coordinate[1]*-np.cos(self.map_yaw)  
        self.offset = {"x": 835.0, "y": 272.0}
        self.x_map_ratio, self.y_map_ratio = self.cal_map_resolution(pixel=self.pix_dist, map=self.map_dist)
        self.stations = self.get_stations(self.stations_path)
        self.csv_path = [f for f in os.listdir(self.segment_dir) if f.endswith('.csv')]
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
    
    def get_stations(self, path):
        df = pd.read_csv(path)
        if not {"name", "x", "y"}.issubset(df.columns):
            raise ValueError("CSV file must contain 'name', 'x', and 'y' columns")
        df['x'], df['y'] = self.transfrom(position=[df['x'], df['y']])
        return df[["name", "x", "y", "position"]].values
    
    def transfrom(self, position):
        tf_x = position[0]*np.cos(self.map_yaw) - position[1]*np.sin(self.map_yaw)
        tf_y = position[0]*-np.sin(self.map_yaw) + position[1]*-np.cos(self.map_yaw)
        tf_x = (tf_x + self.offset["x"])*self.x_map_ratio
        tf_y = (tf_y + self.offset["y"])*self.y_map_ratio
        return [tf_x,tf_y]

    def plot(self):
        print(f"image resolution: width = {self.img_width}, height: {self.img_height}")
        plt.imshow(self.map_image)
        origin_x = (self.x + self.offset["x"]) * self.x_map_ratio
        origin_y = (self.y + self.offset["y"]) * self.y_map_ratio
        for path_file in self.csv_path:
            full_path = os.path.join(self.segment_dir, path_file)
            path = pd.read_csv(full_path)
            x_list, y_list = [], []
            for idx in range(len(path)):
                x, y = self.transfrom(position=[path['x'][idx], path['y'][idx]])
                x_list.append(x)
                y_list.append(y)
            plt.plot(x_list, y_list, c=(1.0, 0.0, 0.0), linewidth=1.2, zorder=1)
        for name, x, y, position in self.stations:
            # plt.scatter(x, y, c=(0.0, 1.0, 0.0), s=15, label=name if name == self.stations[0, 0] else "", zorder=2)
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
            # plt.text(
            #     text_x, text_y, name,
            #     fontsize=6,
            #     color='white',
            #     ha='center',
            #     fontweight='bold',
            #     bbox=dict(
            #         facecolor='none',
            #         edgecolor=(255/255, 0.0/255, 127/255),
            #         boxstyle='round,pad=0.2'
            #     ),
            #     zorder=3 
            # )
            plt.text(text_x, text_y, name, fontsize=6, color='white', ha='center', fontweight='bold',
                    bbox=dict(facecolor=(255/255, 0.0/255, 127/255), edgecolor='none', boxstyle='round,pad=0.2'))
            plt.scatter(x, y, c=(0.0, 1.0, 0.0), s=12, label=name if name == self.stations[0, 0] else "", zorder=2)
        plt.subplots_adjust(left=0.05, bottom=0.05, right=1.0, top=1.0)
        plt.show()


def main():
    so = StationOutline()
    so.plot()

if __name__ == '__main__':
    main()