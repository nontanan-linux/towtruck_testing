import cv2
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

class StationOutline:
    def __init__(self, zoom_factor=1.0):
        self.map_image_path = '/home/nontanan/Pictures/bg-station-plane.bmp'
        self.stations_path = '/home/nontanan/ros2_ws/src/towtruck_testing/resource/GoalPoints.csv'
        self.map_image = cv2.imread(self.map_image_path)
        if self.map_image is None:
            raise FileNotFoundError(f"Not found img {self.map_image_path}")
        self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2RGB)
        self.img_height, self.img_width, _ = self.map_image.shape
        self.dpi = 400 
        self.zoom_factor = zoom_factor
        self.offset_x = 1000
        self.offset_y = 610
        self.ref_point = [-146.8, -614.5]  # New axis origin
        self.ref_scale = [1747.2, 804.9]
        self.stations = self.get_stations()
        self.dx = self.stations[:, 1].max() - self.stations[:, 1].min()
        self.dy = self.stations[:, 2].max() - self.stations[:, 2].min()
        self.zoomed_image = cv2.resize(self.map_image, (int(self.img_width * self.zoom_factor), int(self.img_height * self.zoom_factor)))

    def get_stations(self):
        df = pd.read_csv(self.stations_path)
        if not {"name", "x", "y"}.issubset(df.columns):
            raise ValueError("CSV file must contain 'name', 'x', and 'y' columns")
        df['y'] = df['y'] + self.offset_y
        df['x'] = ((self.img_height - df['x']) - self.offset_x)
        df['x'] = df['x'] - self.ref_point[0]
        df['y'] = df['y'] - self.ref_point[1]
        return df[["name", "x", "y"]].values

    def plot(self):
        plt.figure(figsize=(self.zoomed_image.shape[1] / self.dpi, self.zoomed_image.shape[0] / self.dpi))
        plt.imshow(self.zoomed_image)
        plt.subplots_adjust(left=0.05, bottom=0.05, right=1.0, top=1.0)
        plt.xlabel('New X (pixels) [Origin at ref_point]')
        plt.ylabel('New Y (pixels) [Origin at ref_point]')
        plt.title(f'Station Map with New Axis (dx={self.dx}, dy={self.dy})')
        new_x = -self.ref_point[0]  # X position of new axis origin
        new_y = -self.ref_point[1]  # Y position of new axis origin
        plt.axhline(new_y, color='blue', linewidth=2, linestyle="-")  # New X-axis
        plt.axvline(new_x, color='blue', linewidth=2, linestyle="-")  # New Y-axis
        plt.grid(True, linestyle="--", alpha=0.5)
        for name, x, y in self.stations:
            plt.scatter(x, y, c='red', s=50, label=name if name == self.stations[0, 0] else "")
            plt.text(x, y, name, fontsize=8, color='white', ha='right')
        plt.show()

def main():
    zoom_factor = 1.5
    so = StationOutline(zoom_factor)
    print(f"dx: {so.dx}, dy: {so.dy}")
    so.plot()

if __name__ == '__main__':
    main()
