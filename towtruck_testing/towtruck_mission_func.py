#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
import time

class TowtruckMission(Node):
	def __init__(self, username, password):
		super().__init__("TowtruckMission_Func")
		self.get_logger().info("Initial Towtruck Mission Tester")
		self.declare_parameter("main_server_ip", "192.168.1.14")
		self.declare_parameter("main_server_port", "5000")
		self.main_server_ip = self.get_parameter("main_server_ip").get_parameter_value().string_value
		self.main_server_port = self.get_parameter("main_server_port").get_parameter_value().string_value
		self.server_url = f"http://{self.main_server_ip}:{self.main_server_port}"
		self.username = username
		self.password = password
		self.token = None

	def login(self, username, password):
		login_url = self.server_url + "/authentication/login"
		login_payload = {"username": username,"password": password,}
		headers = {"Accept": "application/json","Content-Type": "application/x-www-form-urlencoded"}
		response = self.request_func("post", login_url, args=login_payload, headers=headers)
		if response and response.status_code == 200:
			try:
				response_data = response.json()
				print("Login Response JSON:", response_data)
				self.token = response_data.get("access_token")  # Ensure this matches the actual response key
				if self.token:
					self.headers = {
						"Authorization": f"Bearer {self.token}",
						"Accept": "application/json",
						"Content-Type": "application/json"}
					self.get_logger().info("Login Successful, Token Acquired")
					return True, response_data
				else:
					self.get_logger().error("Login failed, token not found")
			except Exception as e:
				self.get_logger().error(f"Error parsing JSON response: {e}")
		else:
			self.get_logger().error(f"Login Failed: {response.status_code if response else 'No Response'}")
		return False, response

	def logout(self, username):
		logout_url = self.server_url + "/authentication/logout"
		logout_payload = {"username": username}
		try:
			response = requests.post(url=logout_url, json=logout_payload, timeout=2.0)
			if response and response.status_code == 200:
				self.get_logger().info(f"Logout Successful: {response.json()}")
				return True
			else:
				self.get_logger().error(f"Logout Failed: {response.status_code if response else 'No Response'}")
		except requests.exceptions.RequestException as error:
			self.get_logger().error(f"Logout Request Error: {error}")
		except Exception as e:
			self.get_logger().error(f"Unexpected Error: {e}")
		return False

	def get_missions(self):
		mission_url = self.server_url + '/fleet/missions'
		headers = {"Accept": "application/json",
				   "Content-Type": "application/x-www-form-urlencoded",
				   "Authorization": f"Bearer {self.token}"}
		params = {"vehicle_name": "ALL", "status": "ALL"}
		missions = None
		try:
			response = self.request_func("get", url=mission_url, headers=headers, params=params)
			if response and response.status_code == 200:
				missions = response.json()
				self.all_missions = missions
				self.get_logger().info('Call Mission Successful')
			else:
				self.get_logger().error(f"Failed to fetch missions: {response.status_code if response else 'No Response'}")
		except requests.exceptions.RequestException as error:
			self.get_logger().error(f"Error fetching missions: {error}")
		except Exception as e:
			self.get_logger().error(f"Unexpected Error: {e}")
		return missions
	
	def get_mission_by_id(self, mission_id):
		mission_url = self.server_url + '/fleet/mission_id'
		headers = {"Accept": "application/json",
				"Content-Type": "application/x-www-form-urlencoded",
				"Authorization": f"Bearer {self.token}"}
		params = {"mission_id": mission_id}
		mission = None
		try:
			response = self.request_func('get', mission_url, headers=headers, params=params)
			if response and response.status_code == 200:
				mission = response.json()
			else:
				self.get_logger().error(f"Failed to fetch mission id {mission_id}: {response.status_code}")
		except requests.exceptions.RequestException as req_error:
			self.get_logger().error(f"Error fetching mission id {mission_id}: {req_error}")
		except Exception as exc_error:
			self.get_logger().error(f"Unexpected error: {exc_error}")
		return mission

	def mission_create(self, agv, pick, requester='admin', type=1):
		mission_url = self.server_url + "/mission/create"
		mission_payload = {
			"nodes": pick,
			"requester": requester,
			"type": type,
			"vehicle_name": agv}
		headers = {
			"Accept": "application/json",
			"Content-Type": "application/json",
			"Authorization": f"Bearer {self.token}"}
		message = None
		try:
			# response = self.request_func("post", url=mission_url, args=mission_payload, headers=headers)
			response = requests.post(url=mission_url, json=mission_payload, headers=headers)
			if response and response.status_code == 200:
				message = response.json()
			else:
				self.get_logger().error(f"Create Mission Error {response.status_code}: {response.text}")
		except requests.exceptions.RequestException as req_error:
			self.get_logger().error(f"Create Mission Error: {req_error}")
		except Exception as exc_error:
			self.get_logger().error(f"Unexpected error: {exc_error}")
		return message

	def mission_update(self, id, nodes, paths='', status=2, transport_state=2, vehicle_name='AGV2'):
		mission_url = self.server_url + "/mission/update"
		mission_payload = {
			"id": id,
			"nodes": nodes,
			"paths": paths,
			"status": status,
			"transport_state": transport_state,
			"vehicle_name": vehicle_name
		}
		headers = {
			"Accept": "application/json",
			"Content-Type": "application/json",
			"Authorization": f"Bearer {self.token}"
		}
		message = None
		try:
			response = requests.put(mission_url, json=mission_payload, headers=headers)
			if response and response.status_code == 200:
				message = response.json()
			else:
				self.get_logger().error(f"Update Mission Error {response.status_code}: {response.text}")
		except requests.exceptions.RequestException as req_error:
			self.get_logger().error(f"Update Mission Error: {req_error}")
		except Exception as exc_error:
			self.get_logger().error(f"Unexpected error: {exc_error}")
		return message

	def mission_next(self, agv, command):
		mission_url = self.server_url + '/fleet/command'
		params = {"command": command, "vehicle_name": agv}
		message = None
		try:
			response = self.request_func(methods='put', url=mission_url, params=params)
			message = response.json()
		except Exception as exc_error:
			self.get_logger().error(f"Unexpected error: {exc_error}")
		return message

	def request_func(self, methods, url, args=None, params=None, headers=None):
		first_attempt = True
		while True:
			try:
				rq = None
				if methods == "post":
					rq = requests.post(url, data=args, timeout=2.0, params=params, headers=headers)
				elif methods == "put":
					rq = requests.put(url, json=args, timeout=2.0, params=params, headers=headers)
				elif methods == "get":
					rq = requests.get(url, timeout=2.0, params=params, headers=headers)
				if rq:
					return rq
			except requests.exceptions.RequestException as error:
				self.get_logger().error(f"Request function Error: {error}")
				if not first_attempt:
					break
				first_attempt = False

	def check_login(self):
		"""Check if login is successful and publish the status."""
		if self.login():
			self.publish_status("Logged in successfully")
		else:
			self.publish_status("Login failed")
	
	def publish_status(self, status_message):
		"""Publish the login status message."""
		self.get_logger().info("Status: %s", status_message)
	
	def get_user_info(self):
		url = f'{self.server_url}/user/me'
		headers = {
			'accept': 'application/json',
			'Authorization': f"Bearer {self.token}"}
		response = requests.get(url, headers=headers)
		if response.status_code == 200:
			return response.json()
		else:
			print(f"Get user info Error: {response.status_code} - {response.text}")
			return None
	
	def get_all_users(self, username="ALL", page=1, page_size=10):
		url = f"{self.server_url}/user/users"
		headers = {"accept": "application/json","Authorization": f"Bearer {self.token}"}
		params = {"username": username,"page": page,"page_size": page_size}
		response = requests.get(url, headers=headers, params=params)
		if response.status_code == 200:
			return response.json()
		else:
			print(f"Get all users Error {response.status_code}: {response.text}")
			return None
	
	def get_vehicle_info(self):
		pass

	def get_all_vehicles(self):
		url = f"{self.server_url}/vehicle/vehicles"
		headers = {"accept": "application/json","Authorization": f"Bearer {self.token}"}
		params = {"vehicle_name": "ALL","state": "ALL"}
		response = requests.get(url, headers=headers, params=params)
		if response.status_code == 200:
			return response.json()
		else:
			print(f"Get all vehicles Error {response.status_code}: {response.text}")
			return None
	
	def get_vehicle_state(self, vehicle_name):
		url = f"{self.server_url}/fleet/vehicles"
		headers = {"accept": "application/json"}
		params = {"vehicle_name": vehicle_name,"state": "ALL"}
		response = requests.get(url, headers=headers, params=params)
		if response.status_code == 200:
			return response.json()
		else:
			print(f"Get vehicle state error {response.status_code}: {response.text}")
			return None

def main(args=None):
	rclpy.init(args=args)
	node = TowtruckMission(username='nontanan', password='1234')
	node.login(node.username, password=node.password)
	node.get_user_info()
	node.get_all_users()
	print(node.get_vehicle_state(vehicle_name='AGV2'))
	# node.mission_create(agv='AGV2', pick='P01S')
	try:
		rclpy.spin(node)
	except Exception as err:
		print(f'Towtruck test mission Error: {err}')
	# except KeyboardInterrupt:
	# 	node.logout()
	finally:
		node.logout(username=node.username)
		node.destroy_node()
		rclpy.shutdown()

if __name__ == "__main__":
	main()