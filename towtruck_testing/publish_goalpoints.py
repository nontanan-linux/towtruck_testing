import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
import time

class MultiMarkerPublisher(Node):
    def __init__(self):
        super().__init__('multi_marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'visualization/marker/goalpoints', 10)
        # self.timer = self.create_timer(1.0, self.publish_markers)  # Publish every second
        self.csv_goal_path = '~/autoware.bg2/data/BG/GoalPoints.csv'
        self.station = pd.read_csv(self.csv_goal_path)
        self.origin_point = ('Origit', 0.0, 0.0)
        self.station_points = []
        for i in range(0, len(self.station)):
            self.station_points.append((self.station['name'][i], self.station['x'][i], self.station['y'][i]))
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        # for i, (name, x, y) in enumerate(self.station_points):
        #     sphere_marker = Marker()
        #     sphere_marker.header.frame_id = "map"
        #     sphere_marker.header.stamp = self.get_clock().now().to_msg()
        #     sphere_marker.ns = "goal_markers"
        #     sphere_marker.id = i
        #     sphere_marker.type = Marker.SPHERE
        #     sphere_marker.action = Marker.ADD
        #     sphere_marker.pose.position.x = x
        #     sphere_marker.pose.position.y = y
        #     sphere_marker.pose.orientation.w = 1.0
        #     sphere_marker.scale.x = 1.5
        #     sphere_marker.scale.y = 1.5
        #     sphere_marker.scale.z = 1.5
        #     sphere_marker.color.r = 1.0
        #     sphere_marker.color.g = 0.0
        #     sphere_marker.color.b = 0.0
        #     sphere_marker.color.a = 1.0 
        #     marker_array.markers.append(sphere_marker)
        #     text_marker = Marker()
        #     text_marker.header.frame_id = "map"
        #     text_marker.header.stamp = self.get_clock().now().to_msg()
        #     text_marker.ns = "goal_markers_text"
        #     text_marker.id = i + 100
        #     text_marker.type = Marker.TEXT_VIEW_FACING
        #     text_marker.action = Marker.ADD
        #     text_marker.pose.position.x = x + 5.5
        #     text_marker.pose.position.y = y + 2.5
        #     text_marker.scale.z = 1.8 
        #     text_marker.color.r = 1.0 
        #     text_marker.color.g = 1.0
        #     text_marker.color.b = 0.0
        #     text_marker.color.a = 1.0 
        #     text_marker.text = name
        #     marker_array.markers.append(text_marker)
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "goal_markers"
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = 0.0
        sphere_marker.pose.position.y = 0.0
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.scale.x = 1.5
        sphere_marker.scale.y = 1.5
        sphere_marker.scale.z = 1.5
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0 
        marker_array.markers.append(sphere_marker)
        ori_marker = Marker()
        ori_marker.header.frame_id = "map"
        ori_marker.header.stamp = self.get_clock().now().to_msg()
        ori_marker.ns = "goal_markers_text"
        ori_marker.id = 1
        ori_marker.type = Marker.TEXT_VIEW_FACING
        ori_marker.action = Marker.ADD
        ori_marker.pose.position.x =  2.5
        ori_marker.pose.position.y =  2.5
        ori_marker.scale.z = 1.8 
        ori_marker.color.r = 1.0 
        ori_marker.color.g = 1.0
        ori_marker.color.b = 0.0
        ori_marker.color.a = 1.0 
        ori_marker.text = 'Origin_point'
        marker_array.markers.append(ori_marker)
        self.publisher.publish(marker_array)
        self.get_logger().info(f"Published {len(self.station_points)} Markers with Labels")

class StationMarkPublisher():
    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.station_data = pd.read_csv(self.csv_path)
        self.station_points = []
        self.marker_array = MarkerArray()
    
    def apply_marker_points(self):
        for idx in range(0, len(self.station_data)):
            self.station_points.append((self.station_data['name'][idx],
                                        self.station_data['x'][idx],
                                        self.station_data['y'][idx]))

    def get_marker_array(self):
        self.apply_marker_points()
        for idx, (name, x, y) in enumerate(self.station_points):
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'map'
            # sphere_marker.header.stamp = time.time()
            sphere_marker.ns = 'stations_markers'
            sphere_marker.id = idx
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = x
            sphere_marker.pose.position.y = y
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 1.5
            sphere_marker.scale.y = 1.5
            sphere_marker.scale.z = 1.5
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 1.0 
            self.marker_array.markers.append(sphere_marker)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            # text_marker.header.stamp = 0.0
            text_marker.ns = "goal_markers_text"
            text_marker.id = idx + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD    
            if name == 'D04A' or name == 'D07A' or name == 'D08A' or name == 'D11A' or name == 'D12A' or name == 'D18S' or name == 'D19S':
                text_marker.pose.position.x = x - 2.7
                text_marker.pose.position.y = y
            elif name == 'P07S' or name == 'P08S':
                text_marker.pose.position.x = x - 5.7
                text_marker.pose.position.y = y
            elif name == 'D05S' or name == 'D06S' or name == 'D09S' or name == 'D10S' or name == 'H01S' or name == 'D17S':
                text_marker.pose.position.x = x + 2.0
                text_marker.pose.position.y = y - 2.5
            else:
                text_marker.pose.position.x = x + 2.0
                text_marker.pose.position.y = y + 2.0
            text_marker.scale.z = 1.8 
            text_marker.color.r = 1.0 
            text_marker.color.g = 0.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0 
            text_marker.text = name
            self.marker_array.markers.append(text_marker)
        # sphere_marker = Marker()
        # sphere_marker.header.frame_id = "map"
        # # sphere_marker.header.stamp = self.get_clock().now().to_msg()
        # sphere_marker.ns = "goal_markers"
        # sphere_marker.id = 0
        # sphere_marker.type = Marker.SPHERE
        # sphere_marker.action = Marker.ADD
        # sphere_marker.pose.position.x = 0.0
        # sphere_marker.pose.position.y = 0.0
        # sphere_marker.pose.orientation.w = 1.0
        # sphere_marker.scale.x = 1.5
        # sphere_marker.scale.y = 1.5
        # sphere_marker.scale.z = 1.5
        # sphere_marker.color.r = 1.0
        # sphere_marker.color.g = 0.0
        # sphere_marker.color.b = 0.0
        # sphere_marker.color.a = 1.0 
        # self.marker_array.markers.append(sphere_marker)
        # ori_marker = Marker()
        # ori_marker.header.frame_id = "map"
        # # ori_marker.header.stamp = self.get_clock().now().to_msg()
        # ori_marker.ns = "goal_markers_text"
        # ori_marker.id = 1
        # ori_marker.type = Marker.TEXT_VIEW_FACING
        # ori_marker.action = Marker.ADD
        # ori_marker.pose.position.x =  2.5
        # ori_marker.pose.position.y =  2.5
        # ori_marker.scale.z = 1.8 
        # ori_marker.color.r = 1.0 
        # ori_marker.color.g = 1.0
        # ori_marker.color.b = 0.0
        # ori_marker.color.a = 1.0 
        # ori_marker.text = 'Origin_point'
        # self.marker_array.markers.append(ori_marker)
        return self.marker_array


def main(args=None):
    rclpy.init(args=args)
    node = MultiMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
