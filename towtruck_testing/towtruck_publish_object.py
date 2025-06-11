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
        self.timer = self.create_timer(0.1, self.publish_message)  # 10 Hz
        self.api_path = f'http://{self.main_server_ip}:{self.main_server_port}/fleet/vehicles'
        self.api_param = {'vehicle_name': 'AGV1'}
        self.data = {
            "message": "Payload of all available vehicles",
            "payload": [
                {
                "name": "AGV1",
                "ip_address": "192.168.1.134",
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
                }
            ]
            }

    def update_other_vehicle(self):
        try:
            response = requests.get(self.api_path, params=self.api_param)
            response.raise_for_status()
            self.data = response.json()
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
    
    def yaw_to_quaternion(self, yaw):
        # Convert yaw (in radians) to quaternion
        w = float(math.cos(yaw / 2))
        x = float(0)
        y = float(0)
        z = float(math.sin(yaw / 2))
        return Quaternion(x=x, y=y, z=z, w=w)

    def publish_message(self):
        self.update_other_vehicle()
        agv_coordinates = self.data['payload'][0]['coordinate'].split(',')
        agv_velocity = self.data['payload'][0]['velocity']
        agv_x = float(agv_coordinates[0])
        agv_y = float(agv_coordinates[1])
        agv_yaw = float(agv_coordinates[2])
        agv_node = self.data['payload'][0]['node']
        quaternion = self.yaw_to_quaternion(agv_yaw)
        msg = TrackedObjects()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        obj = TrackedObject()
        obj.object_id = UUID(uuid=[244, 107, 198, 33, 178, 5, 230, 214, 69, 175, 74, 212, 99, 94, 219, 207])
        obj.existence_probability = 0.0
        
        classification = ObjectClassification()
        classification.label = 1
        classification.probability = 1.0
        obj.classification.append(classification)
        
        obj.kinematics.pose_with_covariance = PoseWithCovariance()
        obj.kinematics.pose_with_covariance.pose = Pose(
            position=Point(x=agv_x, y=agv_y, z=2.059),
            orientation=quaternion
            # orientation=Quaternion(x=0.0, y=0.0, z=-0.9988674928779524, w=0.04757868926014768)
        )
        obj.kinematics.pose_with_covariance.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 4.52e-05
        ]
        obj.kinematics.orientation_availability = 1
        obj.kinematics.twist_with_covariance = TwistWithCovariance()
        obj.kinematics.twist_with_covariance.twist.linear.x = agv_velocity
        obj.kinematics.twist_with_covariance.twist.angular.z = 0.0
        obj.shape.type = 0
        obj.shape.dimensions.x = 2.8
        obj.shape.dimensions.y = 1.9204451971234415
        obj.shape.dimensions.z = 2.0272561493488457
        
        msg.objects.append(obj)
        print(f'agv data {agv_node}--> x:{agv_x}, y:{agv_y},  yaw: {agv_yaw}, vel: {agv_velocity}')
        self.publisher_object.publish(obj)

    def publish_predestrain(self):
        obj = self.presdestrian_object()
        print(obj)
        self.dummy_publisher.publish(obj)
    
    def presdestrian_object(self):
        obj = Object()
        obj.header.frame_id = 'map'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id.uuid = 244, 107, 198, 33, 178, 5, 230, 214, 69, 175, 74, 212, 99, 94, 219, 207
        obj.initial_state.pose_covariance.pose.position.x = -4.710485458374023
        obj.initial_state.pose_covariance.pose.position.y = 131.7153778076172
        obj.initial_state.pose_covariance.pose.orientation.z = -0.6941514886369103
        obj.initial_state.pose_covariance.pose.orientation.w = 0.7198289455302289
        obj.initial_state.twist_covariance.twist.linear.x = 1.0
        obj.initial_state.twist_covariance.twist.angular.z = 0.0
        obj.classification.label = 7
        obj.classification.probability = 1.0
        obj.shape.type = 1
        obj.shape.footprint.points = []
        obj.shape.dimensions.x = 2.8
        obj.shape.dimensions.y = 1.4
        obj.shape.dimensions.z = 2.0
        obj.max_velocity = 33.00
        obj.min_velocity = -33.00
        obj.action = 0
        return obj

# {'message': 'Payload of all available vehicles', 
#  'payload': [{'name': 'AGV1', 
#               'ip_address': '192.168.1.34', 
#               'port': '5000', 
#               'mission_id': None, 
#               'state': 0, 
#               'mode': 'MANUAL', 
#               'emergency_state': True, 
#               'battery': 100, 
#               'coordinate': '0.0,0.0,0.0', 
#               'velocity': 0.0, 
#               'node': 'D07S', 
#               'home': 'D07S'}]}

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
	
# create python to publish this command
# ros2 topic pub -r 10 /perception/object_recognition/tracking/objects autoware_auto_perception_msgs/msg/TrackedObjects '{
#   "header": {
#     "stamp": { "sec": 1741055761, "nanosec": 432791711 },
#     "frame_id": "map"
#   },
#   "objects": [
#     {
#       "object_id": { "uuid": [244, 107, 198, 33, 178, 5, 230, 214, 69, 175, 74, 212, 99, 94, 219, 207] },
#       "existence_probability": 0.0,
#       "classification": [{ "label": 7, "probability": 1.0 }],
#       "kinematics": {
#         "pose_with_covariance": {
#           "pose": {
#             "position": { "x": -423.93511962890625, "y": 78.63873291015625, "z": 2.059 },
#             "orientation": { "x": 0.0, "y": 0.0, "z": -0.9988674928779524, "w": 0.02508348045926657 }
#           },
#           "covariance": [
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 4.52e-05
#           ]
#         }
#       },
#       "shape": {
#         "dimensions": { "x": 0.68, "y": 0.68, "z": 0.08 }
#       }
#     }
#   ]
# }'

# car data
# header:
#   stamp:
#     sec: 1741066801
#     nanosec: 156021579
#   frame_id: map
# objects:
# - object_id:
#     uuid:
#     - 162
#     - 141
#     - 87
#     - 7
#     - 144
#     - 48
#     - 211
#     - 157
#     - 235
#     - 121
#     - 241
#     - 130
#     - 145
#     - 186
#     - 87
#     - 195
#   existence_probability: 0.0
#   classification:
#   - label: 1
#     probability: 1.0
#   kinematics:
#     pose_with_covariance:
#       pose:
#         position:
#           x: -193.6587781973886
#           y: -29.03496528130226
#           z: 2.169313907623291
#         orientation:
#           x: 0.0
#           y: -0.0
#           z: -0.9997700278025949
#           w: 0.02144508119822632
#       covariance:
#       - 0.25706163206864646
#       - 0.010633875058064585
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010633875058064587
#       - 0.03913514443089769
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.02265651192842225
#     orientation_availability: 1
#     twist_with_covariance:
#       twist:
#         linear:
#           x: 3.002834521916745
#           y: 0.0
#           z: 0.0
#         angular:
#           x: 0.0
#           y: 0.0
#           z: -0.0014531857058772614
#       covariance:
#       - 0.7365170326281959
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.00013784205202590152
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.010000000000000002
#       - 0.0
#       - 0.00013784205202590228
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.02223873140127866
#     acceleration_with_covariance:
#       accel:
#         linear:
#           x: 0.0
#           y: 0.0
#           z: 0.0
#         angular:
#           x: 0.0
#           y: 0.0
#           z: 0.0
#       covariance:
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#       - 0.0
#     is_stationary: false
#   shape:
#     type: 0
#     footprint:
#       points: []
#     dimensions:
#       x: 4.4
#       y: 1.8036462255794103
#       z: 0.11573672083004931
# ---


# pub pointcloud
# ros2 topic pub /perception/obstacle_segmentation/pointcloud sensor_msgs/msg/PointCloud2 "{header: {stamp: {sec: 1741071546, nanosec: 397092496}, frame_id: 'base_link'}, height: 1, width: 200, fields: [{name: 'x', offset: 0, datatype: 7, count: 1}, {name: 'y', offset: 4, datatype: 7, count: 1}, {name: 'z', offset: 8, datatype: 7, count: 1}], is_bigendian: false, point_step: 16, row_step: 3200, data: [105, 39, 209, 65, 102, 63, 51, 192, 224, 157, 102, 191, 0, 0, 128, 63, 149, 116, 209, 65, 79, 209, 51, 192, 229, 242, 229, 190, 0, 0, 128, 63, 51, 194, 208, 65, 169, 163, 52, 192, 15, 192, 33, 60, 0, 0, 128, 63, 186, 80, 209, 65, 143, 1, 51, 192, 238, 184, 246, 62, 0, 0, 128, 63, 235, 25, 209, 65, 71, 24, 49, 192, 232, 105, 106, 63, 0, 0, 128, 63, 84, 234, 208, 65, 56, 146, 49, 192, 114, 185, 111, 191, 0, 0, 128, 63, 7, 208, 208, 65, 59, 37, 48, 192, 123, 102, 246, 190, 0, 0, 128, 63, 158, 241, 208, 65, 78, 77, 43, 192, 139, 13, 137, 61], is_dense: true}"
