import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
from publish_goalpoints import StationMarkPublisher

class PublishStation(Node):
    def __init__(self):
        super().__init__('Marker_publisher')
        self.csv = '~/autoware.bg2/data/BG/GoalPoints.csv'
        self.stations_marker = StationMarkPublisher(csv_path=self.csv)
        self.station_array = self.stations_marker.get_marker_array()
        self.publisher = self.create_publisher(MarkerArray, 'visualization/marker/goalpoints', 10)
        self.publisher.publish(self.station_array)

def main(args=None):
    rclpy.init(args=args)
    node = PublishStation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()