import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from autoware_auto_planning_msgs.msg import PathWithLaneId
from geometry_msgs.msg import Quaternion
import math
import pandas as pd
import matplotlib.pyplot as plt
from itertools import zip_longest

class PathWithLaneIDSubscriber(Node):
    def __init__(self):
        super().__init__('path_with_lane_id_subscriber')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.path_with_lane_id_sub = self.create_subscription(PathWithLaneId,  
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.path_with_lane_id_callback,
            self.qos_profile)
        self.csv_goal_file = '~/autoware.bg2/data/BG/GoalPoints.csv'
        self.csv_save_path = '~/ros2_ws/src/towtruck_testing/csv/area1/vehicle.csv'
        self.main_time = self.create_timer(0.1, self.main)
        self.path_lane = PathWithLaneId()
        self.x, self.y, self.yaw = [], [], []
        self.linear_vel, self.angular_vel, self.lane_id = [], [], []
        self.left_bound = {'x': [], 'y': []}
        self.rigth_bound = {'x': [], 'y': []}
        self.weight = False
        self.define = False

    def path_with_lane_id_callback(self, msg):
        self.path_lane = msg
        self.weight = True
    
    def main(self):
        print(self.weight, self.define)
        if self.weight and not self.define:
            points = self.path_lane.points
            print(f"Received {len(points)} points.")
            for idx in range(0, len(self.path_lane.left_bound)):
                self.left_bound['x'].append(self.path_lane.left_bound[idx].x)
                self.left_bound['y'].append(self.path_lane.left_bound[idx].y)
            for idy in range(0, len(self.path_lane.right_bound)):
                self.rigth_bound['x'].append(self.path_lane.right_bound[idy].x)
                self.rigth_bound['y'].append(self.path_lane.right_bound[idy].y)
            self.define_path(points)
            self.save_data()
            print(f'Distance: {self.cals_dis_points(path_x=self.x, path_y=self.y)}')
            self.plot_data()

    def define_path(self, points):
        for idx in range(len(points)):
            self.x.append(points[idx].point.pose.position.x)
            self.y.append(points[idx].point.pose.position.y)
            self.yaw.append(self.quaternion_to_yaw(points[idx].point.pose.orientation))
            self.linear_vel.append(points[idx].point.longitudinal_velocity_mps)
            self.angular_vel.append(points[idx].point.heading_rate_rps)
            self.lane_id.append(points[idx].lane_ids)
    
    def cals_dis_points(self, path_x, path_y):
        distance = 0.0
        for idx in range(1, len(path_x)):
            distance += math.sqrt(pow(path_x[idx]-path_x[idx-1], 2)+pow(path_y[idx]-path_y[idx-1],2))
        return distance
        
    def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def save_data(self):
        data = zip_longest(self.x, self.y, self.yaw, self.linear_vel, self.angular_vel,
                   self.lane_id, self.left_bound['x'], self.left_bound['y'],
                   self.rigth_bound['x'], self.rigth_bound['y'], fillvalue=None)
        df = pd.DataFrame(data, columns=['x', 'y', 'yaw', 'linear_vel', 'angular_vel', 'lane_id',
                                 'left_x', 'left_y', 'right_x', 'right_y'])

        df.to_csv(self.csv_save_path, index=False)
        print(f"Path data saved to {self.csv_save_path}")
        self.define = True
    
    def plot_data(self):
        plt.figure(figsize=(10, 6))
        plt.scatter(self.x, self.y, c='blue', label='vehicle trajectory', s=5)
        plt.plot(self.left_bound['x'], self.left_bound['y'], color='green')
        plt.plot(self.rigth_bound['x'], self.rigth_bound['y'], color='green', label='vehicle lane')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Path with Lane Boundaries')
        plt.legend()
        plt.axis('equal')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PathWithLaneIDSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
