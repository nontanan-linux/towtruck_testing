import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
from towtruck_publish_marker_points import GoalPointsMarkPublisher, HornPointsMarkPublisher

class PublishStation(Node):
    def __init__(self):
        super().__init__('Marker_publisher')
        self.goalpoints_csv = '~/autoware.bg2/data/BG/GoalPoints.csv'
        self.hornpoints_csv = '~/autoware.bg2/data/BG/HornPoints.csv'
        # self.hornpoints_csv = '/home/nontanan/ros2_ws/src/towtruck_testing/csv/HornPoints.csv'
        self.stations_marker = GoalPointsMarkPublisher(csv_path=self.goalpoints_csv)
        self.hornpoints_marker = HornPointsMarkPublisher(csv_path=self.hornpoints_csv)
        self.station_array = self.stations_marker.get_marker_array()
        self.hornpoints_array = self.hornpoints_marker.get_marker_array()
        self.goalpoints_publisher = self.create_publisher(MarkerArray, 'visualization/marker/goalpoints', 10)
        self.hornpoints_publisher = self.create_publisher(MarkerArray, 'visualization/marker/hornpoints', 10)
        self.goalpoints_publisher.publish(self.station_array)
        self.hornpoints_publisher.publish(self.hornpoints_array)

def main(args=None):
    rclpy.init(args=args)
    node = PublishStation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()