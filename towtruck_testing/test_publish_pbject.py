import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import TrackedObjects, TrackedObject, ObjectClassification
from dummy_perception_publisher.msg import Object
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance
import time
import requests
import math

class TrackedObjectsPublisher(Node):
    def __init__(self):
        super().__init__('towtruck_objects_publisher')
        self.declare_parameter("main_server_ip", "192.168.1.14")
        self.declare_parameter("main_server_port", "5000")
        self.main_server_ip = self.get_parameter("main_server_ip").get_parameter_value().string_value
        self.main_server_port = self.get_parameter("main_server_port").get_parameter_value().string_value
        self.publisher_object = self.create_publisher(TrackedObjects, '/perception/object_recognition/tracking/objects', 10)
        self.dummy_publisher = self.create_publisher(Object, '/simulation/dummy_perception_publisher/object_info', 10)
        self.object = Object()
        # self.timer = self.create_timer(0.2, self.main)  # 10 Hz
        self.api_path = f'http://{self.main_server_ip}:{self.main_server_port}/fleet/vehicles'
        self.api_param = {'vehicle_name': 'AGV1'}
        self.data = {
            "message": "Payload of all available vehicles",
            "payload": [
                {
                "name": "AGV1",
                "ip_address": "192.168.1.29",
                "port": "5000",
                "mission_id": None,
                "state": 0,
                "mode": "MANUAL",
                "emergency_state": True,
                "battery": 100,
                "coordinate": "0.0, 0.0,0.0",
                "velocity": 0,
                "node": "",
                "home": ""
                }]}
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        car = self.car_object()
        self.dummy_publisher.publish(car)
        # time.sleep(2.5)
        # self.dummy_publisher.publish(self.delete_obj)

    def update_another_vehicle(self):
        try:
            response = requests.get(self.api_path, params=self.api_param)
            response.raise_for_status()
            self.data = response.json()
            agv_coordinates = self.data['payload'][0]['coordinate'].split(',')
            self.agv = {'x': float(agv_coordinates[0]), 'y': float(agv_coordinates[1]), 'yaw': float(agv_coordinates[2]),
                        'node': self.data['payload'][0]['node'],
                        'quaternion': self.yaw_to_quaternion(float(agv_coordinates[2])),
                        'velocity': self.data['payload'][0]['velocity']}
            # self.agv_velocity = self.data['payload'][0]['velocity']
            # self.agv_x = float(agv_coordinates[0])
            # self.agv_y = float(agv_coordinates[1])
            # self.agv_node = self.data['payload'][0]['node']
            # self.quaternion = self.yaw_to_quaternion(float(agv_coordinates[2]))
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
    
    def yaw_to_quaternion(self, yaw):
        w = float(math.cos(yaw / 2))
        x = float(0)
        y = float(0)
        z = float(math.sin(yaw / 2))
        return Quaternion(x=x, y=y, z=z, w=w)

    def main(self):
        delete_obj = self.delete_object()
        self.dummy_publisher.publish(delete_obj)
        self.curr_time = self.get_clock().now().nanoseconds / 1e9
        if self.curr_time - self.prev_time >= 0.5:
            self.update_another_vehicle()
            car = self.car_object()
            obj = self.presdestrian_object()
            bus = self.bus_object()
            self.dummy_publisher.publish(bus)
            print(f"agv data {self.agv['node']} --> x: {self.agv['x']}, y: {self.agv['y']}, yaw: {self.agv['yaw']}, vel: {self.agv['velocity']}")
            self.prev_time = self.prev_time = self.get_clock().now().nanoseconds / 1e9
    
    def presdestrian_object(self):
        obj = Object()
        obj.header.frame_id = 'map'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id.uuid = 244, 107, 198, 33, 178, 5, 230, 214, 69, 175, 74, 212, 99, 94, 219, 207
        obj.initial_state.pose_covariance.pose.position.x = -4.710485458374023
        obj.initial_state.pose_covariance.pose.position.y = 131.7153778076172
        obj.initial_state.pose_covariance.pose.orientation.z = -0.6941514886369103
        obj.initial_state.pose_covariance.pose.orientation.w = 0.7198289455302289
        obj.initial_state.twist_covariance.twist.linear.x = 0.0
        obj.initial_state.twist_covariance.twist.angular.z = 0.0
        obj.classification.label = 7
        obj.classification.probability = 1.0
        obj.shape.type = 1
        obj.shape.footprint.points = []
        obj.shape.dimensions.x = 1.8
        obj.shape.dimensions.y = 1.8
        obj.shape.dimensions.z = 2.0
        obj.max_velocity = 33.00
        obj.min_velocity = -33.00
        obj.action = 0
        return obj
    
    def delete_object(self):
        obj = Object()
        obj.header.frame_id = 'map'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id.uuid = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        obj.initial_state.pose_covariance.pose.position.x = 0.0
        obj.initial_state.pose_covariance.pose.position.y = 0.0
        obj.initial_state.pose_covariance.pose.orientation.z = 0.0
        obj.initial_state.pose_covariance.pose.orientation.w = 0.0
        obj.initial_state.twist_covariance.twist.linear.x = 0.0
        obj.initial_state.twist_covariance.twist.angular.z = 0.0
        obj.classification.label = 0
        obj.classification.probability = 0.0
        obj.shape.type = 0
        obj.shape.footprint.points = []
        obj.shape.dimensions.x = 0.0
        obj.shape.dimensions.y = 0.0
        obj.shape.dimensions.z = 0.0
        obj.max_velocity = 0.0
        obj.min_velocity = 0.0
        obj.action = 3
        return obj
    
    def car_object(self):
        car = Object()
        car.header.frame_id = 'map'
        car.header.stamp = self.get_clock().now().to_msg()
        car.id.uuid = 119, 254, 129, 148, 58, 246, 50, 101, 125, 52, 129, 242, 175, 230, 114, 219
        # car.initial_state.pose_covariance.pose.position.x = self.agv['x']
        # car.initial_state.pose_covariance.pose.position.y = self.agv['y']
        # car.initial_state.pose_covariance.pose.orientation = self.agv['quaternion']
        # car.initial_state.twist_covariance.twist.linear.x = self.agv['velocity']
        car.initial_state.pose_covariance.pose.position.x = -9.196030616760254
        car.initial_state.pose_covariance.pose.position.y = 182.21859741210938
        car.initial_state.pose_covariance.pose.orientation.z = -0.6740537542235466
        car.initial_state.pose_covariance.pose.orientation.w = 0.7386822973492343
        car.initial_state.twist_covariance.twist.linear.x = 2.5
        car.initial_state.twist_covariance.twist.angular.z = 0.0
        car.classification.label = 1
        car.classification.probability = 1.0
        car.shape.type = 0
        car.shape.footprint.points = []
        car.shape.dimensions.x = 4.0
        car.shape.dimensions.y = 1.8
        car.shape.dimensions.z = 2.0
        car.max_velocity = 3.0
        car.min_velocity = 3.0
        car.action = 0
        return car
    
    def bus_object(self):
        bus = Object()
        bus.header.frame_id = 'map'
        bus.header.stamp = self.get_clock().now().to_msg()
        bus.id.uuid = 119, 254, 129, 148, 58, 246, 50, 101, 125, 52, 129, 242, 175, 230, 114, 219
        bus.initial_state.pose_covariance.pose.position.x = -5.708061695098877
        bus.initial_state.pose_covariance.pose.position.y = 143.5214080810547
        bus.initial_state.pose_covariance.pose.orientation.z = -0.6735728569403476
        bus.initial_state.pose_covariance.pose.orientation.w = 0.7391208334184729
        bus.initial_state.twist_covariance.twist.linear.x = 0.0
        bus.initial_state.twist_covariance.twist.angular.z = 0.0
        bus.classification.label = 3
        bus.classification.probability = 1.0
        bus.shape.type = 0
        bus.shape.footprint.points = []
        bus.shape.dimensions.x = 10.5
        bus.shape.dimensions.y = 2.5
        bus.shape.dimensions.z = 3.5
        bus.max_velocity = 0.0
        bus.min_velocity = 0.0
        bus.action = 0
        return bus

def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()