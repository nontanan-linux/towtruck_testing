#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.common.exceptions import WebDriverException, NoAlertPresentException, TimeoutException,NoSuchElementException
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.chrome.options import Options
from enum import Enum
import random
import requests

class MissionStateValue(Enum):
	INITIALIZING = 1
	WAITING_FOR_ROUTE = 2
	PLANNING = 3
	WAITING_FOR_ENGAGE = 4
	DRIVING = 5
	ARRIVED_GOAL = 6
	FINALIZING = 7

class VehicleStateValue(Enum):
	INITIALIZING = 1
	WAITING_FOR_ROUTE = 2
	PLANNING = 3
	WAITING_FOR_ENGAGE = 4
	DRIVING = 5
	ARRIVED_GOAL = 6
	FINALIZING = 7

class WebControlStateValue(Enum):
	INITIALIZING = 1
	WAITING_FOR_MISSION = 2
	DRIVING = 3
	WAITTING_FOR_PICKUP = 4
	TRANSIT = 5
	WAITTING_FOR_DROP = 6
	DROPED = 7
	READY = 8

class TwotruckWebDriver(Node):
	def __init__(self):
		super().__init__("Towtruck_Web_Driver")
		self.options = Options()
		self.options.add_argument("--start-fullscreen")
		self.driver = webdriver.Chrome(options=self.options)
		self.declare_parameter("username", "nontanan")
		self.declare_parameter("password", "1234")
		self.declare_parameter("agv_name", 'AGV3')
		self.declare_parameter("agv_num", 3)
		self.declare_parameter("main_server_ip", "192.168.1.14")
		self.declare_parameter("main_server_port", "5010")
		self.declare_parameter("main_web_ip", "192.168.1.14")
		self.declare_parameter("main_web_port", "5012")
		self.declare_parameter("loop_period_sec", 0.5)
		self.declare_parameter('debug_info', True)
		self.username = self.get_parameter("username").get_parameter_value().string_value
		self.password = self.get_parameter("password").get_parameter_value().string_value
		self.username_tester = 'admin4'
		self.password_tester = 'admin'
		self.agv_name = self.get_parameter("agv_name").get_parameter_value().string_value
		self.agv_num = self.get_parameter("agv_num").get_parameter_value().integer_value
		self.curr_mission = None
		self.main_server_ip = self.get_parameter("main_server_ip").get_parameter_value().string_value
		self.main_server_port = self.get_parameter("main_server_port").get_parameter_value().string_value
		self.main_web_ip = self.get_parameter("main_web_ip").get_parameter_value().string_value
		self.main_web_port = self.get_parameter("main_web_port").get_parameter_value().string_value
		self.loop_period_sec = self.get_parameter("loop_period_sec").get_parameter_value().double_value
		self.debug_info = self.get_parameter('debug_info').get_parameter_value().bool_value
		self.web_url = f'http://{self.main_web_ip}:{self.main_web_port}'
		self.vehicle_status = None
		self.has_process = False
		self.process_result = None
		self.main_loop = self.create_timer(timer_period_sec=self.loop_period_sec, callback=self.main)
		self.web_control_state = WebControlStateValue.INITIALIZING
		if self.debug_info:
			self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
		self.nav_xpath = {
			"Home": '/html/body/div/div/section/div/ul/li[1]/a',
			"Mission": '/html/body/div/div/section/div/ul/li[2]/a',
			"Truck": '/html/body/div/div/section/div/ul/li[3]/a',
			"Statistics": '/html/body/div/div/section/div/ul/li[4]/a',
			"Battery": '/html/body/div/div/section/div/ul/li[5]/a',
			"Alarm": '/html/body/div/div/section/div/ul/li[6]/a[1]',
			"Login": '/html/body/div/div/section/div/ul/li[7]/a'
		}

		self.pick_xpath = {
			"P01S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[1]',
			"P02S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[2]',
			"P03S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[3]',
			"P04S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[4]',
			"P05S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[5]',
			"P06S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[6]',
			"Back": '/html/body/div/div/section/section/div[2]/div/div[1]/button[9]',
			"Send": '/html/body/div/div/section/section/div[2]/div/div[2]/div/button'
		}

		self.drop_xpath = {
			"D01S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[1]',
			"D02S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[2]',
			"D03S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[3]',
			"D04S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[4]',
			"D05S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[5]',
			"D06S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[6]',
			"D07S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[7]',
			"D08S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[8]',
			"D09S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[9]',
			"D10S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[10]',
			"D11S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[11]',
			"D12S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[12]',
			"D13S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[13]',
			"D14S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[14]',
			"D15S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[15]',
			"D16S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[16]',
			"D17S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[17]',
			"D18S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[18]',
			"D19S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[19]',
			"D20S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[20]',
			"D21S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[21]',
			"D22S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[22]',
			"Send": '/html/body/div/div/section/section/div[2]/div/div[2]/div/button',
			"Back": '/html/body/div/div/section/section/div[2]/div/div[2]/button',
			"Delete": '/html/body/div/div/section/section/div[2]/div/div[2]/div/div[2]/div[2]/div[2]/div[3]/svg/path',
			"Confirm": '/html/body/div/div/section/section/div[1]/div/button'
		}
		self.create_btn_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
		self.choose_drop_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
		self.mission_box_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]'
		self.alert_mission_box_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[2]'

	def get_website(self):
		self.driver.get(f"{self.web_url}/home")
		self.wait_for_element('//input[@type="text"]')

	def wait_for_element(self, xpath, timeout=10, clickable=False):
		wait = WebDriverWait(self.driver, timeout)
		if clickable:
			wait.until(EC.element_to_be_clickable((By.XPATH, xpath)))
		else:
			wait.until(EC.presence_of_element_located((By.XPATH, xpath)))

	def login(self):
		self.wait_for_element('/html/body/div/div/section/section/section/div[3]/form/div[1]/input')
		username_input = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/div[1]/input')
		password_input = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/div[2]/input')
		username_input.send_keys(self.username)
		password_input.send_keys(self.password)
		login_button = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/button[1]')
		login_button.click()
		try:
			WebDriverWait(self.driver, 5).until(EC.alert_is_present())
			alert = self.driver.switch_to.alert
			print("Alert found:", alert.text)
			alert.accept()
			print("Alert accepted.")
		except TimeoutException:
			print("No alert appeared.")
		except NoAlertPresentException:
			print("No alert present exception.")

	def get_navbar(self):
		return {name: self.driver.find_element(By.XPATH, xpath) for name, xpath in self.nav_xpath.items()}
	
	def get_vehicle_velocity(self, vehicle='AGV2'):
		if vehicle == 'AGV2':
			agv2_vel_xpath = ''
	
	def get_vehicle_mission(self, vehicle='AGV2'):
		if vehicle == 'AGV2':
			agv_mission_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[2]/div[5]/p/span'
		elif vehicle == 'AGV1':
			agv_mission_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[1]/div[2]/div[5]/p/span'
		try:
			# Wait for the element to be present in DOM
			WebDriverWait(self.driver, 10).until(EC.presence_of_element_located((By.XPATH, agv_mission_xpath)))
			# Wait until the element has non-empty text
			WebDriverWait(self.driver, 10).until(lambda d: d.find_element(By.XPATH, agv_mission_xpath).text.strip() != "")
			element_text = self.driver.find_element(By.XPATH, agv_mission_xpath).text.lstrip()
			id = int(element_text.lstrip('#'))
			return id
		except TimeoutException:
			return "No mission assigned"
		except NoSuchElementException:
			return "Element not found"
	
	def get_vehicle_mode(self, vehicle='AGV2'):
		if vehicle == 'Agv2':
			agv2_mode_xpath = ''
	
	def get_vehicle_state(self, vehicle='AGV2'):
		if vehicle == 'AGV2':
			agv2_state_xpath = ''
	
	def get_vehicle_status(self, vehicle="AGV2"): #status online/offline
		try:
			if vehicle == 'AGV1':
				status_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[1]/div[1]/div[1]/div[3]'
			elif vehicle == 'AGV2':
				status_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[2]/div[1]/div[1]/div[3]'
			elif vehicle == 'AGV3':
				# status_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[1]/div[1]/div[3]'
				# status_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[1]/div[1]/div[2]'
				status_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[1]/div[1]/div[3]'
			else:
				return "Unlnown vehicle"
			self.wait_for_element(status_xpath)
			status_element = self.driver.find_element(By.XPATH, status_xpath)
			status_text = status_element.get_attribute("innerText").strip().lower()
			return status_text
		except NoSuchElementException:
			return "Element not found"
		except Exception as err:
			return f"Get vehicle status error: {err}"
	
	def get_vehicle_battery(self, vehicle='AGV2'):
		if vehicle == 'AGV2':
			agv2_battery_xpath = ''
	
	def get_mission_req(self, vehicle='ALL', status='ALL'):
		mission_url = f'{self.web_url}/fleet/missions'
		headers = {"Accept": "application/json",}

	def create_mission(self):
		# self.create_btn_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[2]/div[3]/button'
		self.wait_for_element(self.create_btn_xpath, clickable=True)
		create_mission_btn = self.driver.find_element(By.XPATH, self.create_btn_xpath)
		create_mission_btn.click()
		self.pick_point = self.random_points(data_dict=self.pick_xpath)
		print(f'Pick up: {self.pick_point}')
		self.wait_for_element(self.pick_xpath[self.pick_point], clickable=True)
		pickup = self.driver.find_element(By.XPATH, self.pick_xpath[self.pick_point])
		pickup.click()
		print(f'Confirm pickup {self.pick_point}')
		self.wait_for_element(self.pick_xpath["Send"], clickable=True)
		send = self.driver.find_element(By.XPATH, self.pick_xpath["Send"])
		send.click()
		self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
		confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
		confirm.click()

	def choose_drop_off(self):
		# self.choose_drop_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[2]/div[3]/button'
		self.wait_for_element(self.choose_drop_xpath, clickable=True)
		choose_drop_button = self.driver.find_element(By.XPATH, self.choose_drop_xpath)
		choose_drop_button.click()
		drop_points = self.random_points(data_dict=self.drop_xpath, type='drop')
		# drop_points = ['D20S', 'D15S', 'D21S', 'D22S']
		# drop_points = ['D01S', 'D04S', 'D06S', 'D12S']
		drop_points.extend(['Send', 'Confirm',])
		print(f'Drop points: {drop_points}')
		for drop in drop_points:
			try:
				print(f'Try to click {drop}')
				self.wait_for_element(self.drop_xpath[drop], clickable=True)
				drop_button = self.driver.find_element(By.XPATH, self.drop_xpath[drop])
				drop_button.click()
			except Exception as err:
				print(f'Error: {err}')
				continue
		# self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
		# confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
		# confirm.click()
	def check_drop(self):
		vehicle_info_box =f'/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[1]' #top-box-data
		mission_contrainer_box = f'/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[3]'
		alert_mission_box_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[2]'
		mission_id = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[5]/p/span'
		check_drop_box = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]'
		check_pickup = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[1]/div[1]'
		check_last_drop = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[3]/div'
		check_before_drop_box = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[2]'
		first_drop = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[2]/div[4]/div[2]'
		second_drop = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[2]/div[5]/div[2]'
		third_drop = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]/div[2]/div[6]/div[2]'
	
	def has_active_mission(self, mission_box_xpath):
		try:
			WebDriverWait(self.driver, 2).until(EC.presence_of_element_located((By.XPATH, mission_box_xpath)))
			mission_id = self.get_vehicle_mission(vehicle='AGV2')
			try:
				alert_element = self.driver.find_element(By.XPATH, self.alert_mission_box_xpath)
				has_mission_alert = True
				alert_text = alert_element.text.strip()
			except NoSuchElementException:
				has_mission_alert = False
				alert_text = None
			return True, mission_id, has_mission_alert, alert_text
		except TimeoutException:
			return False, None, False, None
		except Exception as has_active_mission_err:
			self.get_logger().fatal(f'Has active Mission Error: {has_active_mission_err}')
	
	def get_mission_process(self):
		mission_process_box_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[3]/div[2]/div[6]/div[1]'
		try:
			WebDriverWait(self.driver, 2).until(EC.presence_of_element_located((By.XPATH, mission_process_box_xpath)))
			center_box = self.driver.find_element(By.XPATH, mission_process_box_xpath)
			process_result = []
			try:
				pickup_circle = center_box.find_element(By.CLASS_NAME, 'circle-pickup')
				pickup_color = pickup_circle.value_of_css_property('background-color')
				pickup_result = self.rgba_result(rgba_str=pickup_color)
				process_result.append(('P01S', pickup_result))
			except NoSuchElementException:
				process_result.append(('P01S', None))
			drop_list = []
			station_boxes = center_box.find_elements(By.CLASS_NAME, 'stations-box')
			for box in station_boxes:
				try:
					label = box.find_element(By.CLASS_NAME, 'label-station').text.strip()
					circle = box.find_element(By.CLASS_NAME, 'circle-top-stations')
					color = circle.value_of_css_property('background-color') #rgba format
					color_result = self.rgba_result(rgba_str=color)
					drop_list.append((label, color_result))
				except NoSuchElementException:
					continue
			process_result.append(drop_list)
			try:
				goal_circle = center_box.find_element(By.CLASS_NAME, 'circle-goal')
				goal_color = goal_circle.value_of_css_property('background-color')
				goal_result = self.rgba_result(rgba_str=goal_color)
				process_result.append(('D05S', goal_result))
			except NoSuchElementException:
				process_result.append(('D05S', None))
			return True, process_result
		except TimeoutException:
			return False, []
		except Exception as e:
			self.get_logger().fatal(f'Get mission process status error: {e}')
			return False, []
	
	def rgba_result(self, rgba_str):
		rgba_str = rgba_str.strip()
		rgba_list = []
		if rgba_str.startswith("rgba(") and rgba_str.endswith(")"):
			values = rgba_str[5:-1].split(",")
			rgba_list = [int(v) if i < 3 else float(v) for i, v in enumerate(values)]
		
		if not rgba_list:
			return None
		else:
			if rgba_list[0] == 255 and rgba_list[1] == 106 and rgba_list[2] == 0:
				return True  # active
			elif rgba_list[0] == 135 and rgba_list[1] == 135 and rgba_list[2] == 135:
				return False  # inactive
			else:
				return rgba_list  # unknown color: return full list

	def drop_product(self):
		drop_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
		drop_product = self.driver.find_element(By.XPATH, drop_xpath)
		drop_product.click()
		self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
		confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
		confirm.click()

	def stop_vehicle(self):
		stop_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[1]/div[2]/button'
		stop_button = self.driver.find_element(By.XPATH, stop_xpath)
		stop_button.click()

	def continue_vehicle(self):
		continue_xpath = f'/html/body/div/div/section/section/div[{self.agv_num}]/div/button'
		continue_button = self.driver.find_element(By.XPATH, continue_xpath)
		continue_button.click()

	def logout(self):
		self.wait_for_element('/html/body/div/div/section/section/section/div[3]/div/button')
		logout_button = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/div/button')
		logout_button.click()
		self.get_logger().info(f'Logout Success')

	def close_website(self):
		self.driver.quit()
	
	def random_points(self, data_dict, type=''):
		data_list = list(data_dict.keys())  # convert to list for random choice
		data_list = [item for item in data_list if item not in ['Back','Send','Confirm']]
		if type == 'drop':
			route1 = ['D05S', 'D06S', 'D11S', 'D12S']
			route2 = ['D07S', 'D08S', 'D09S', 'D10S']
			route3 = ['D18S','D19S']
			route4 = ['D13S','D14S']
			drop_points = []
			count_num = random.randint(1, 4)
			for _ in range(count_num):
				if not data_list:
					break
				point = random.choice(data_list)
				if point in route1:
					data_list = [drop for drop in data_list if drop not in route2]
				elif point in route2:
					data_list = [drop for drop in data_list if drop not in route1]
				elif point in route3:
					data_list = [drop for drop in data_list if drop not in route4]
				elif point in route4:
					data_list = [drop for drop in data_list if drop not in route3]
				drop_points.append(point)
				data_list.remove(point)
			return drop_points
		else:
			return random.choice(data_list) if data_list else None

	
	def view_windows(self, vehicle='ALL'):
		all_agv_xpath = '/html/body/div/div/section/section/section[3]/div[1]/button[1]'
		agv1_xpath = '/html/body/div/div/section/section/section[3]/div[1]/button[2]'
		agv2_xpath = '/html/body/div/div/section/section/section[3]/div[1]/button[3]'
		if vehicle == 'AGV1':
			view_button = self.driver.find_element(By.XPATH, agv1_xpath)
			view_button.click()
		elif vehicle == 'AGV2':
			view_button = self.driver.find_element(By.XPATH, agv2_xpath)
			view_button.click()
		else:
			view_button = self.driver.find_element(By.XPATH, all_agv_xpath)
			view_button.click()

	def login_for_api(self, username, password):
		login_url = self.web_url + "/authentication/login"
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
	
	def logout_for_api(self, username):
		logout_url = self.web_url + "/authentication/logout"
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

	def main(self):
		self.vehicle_status = self.get_vehicle_status(vehicle='AGV3')
		if self.vehicle_status != 'offline':
			visible_btn, btn_text = self.waitting_for_button(self.create_btn_xpath)
			# self.get_logger().info(f'visible_button: {visible_btn}, button text: {btn_text}')
			has_mission_box, mission_id, has_mission_alert, alert_text = self.has_active_mission(self.mission_box_xpath)
			# self.get_logger().info(f'vehicle {self.agv_name} status: {self.vehicle_status}, btn: {btn_text}, has_mission_box: {has_mission_box}')
			if (btn_text=='create mission' and self.vehicle_status=='ready' and 
	   			(self.web_control_state==WebControlStateValue.INITIALIZING or self.web_control_state==WebControlStateValue.READY)):
				self.create_mission()
				self.web_control_state = WebControlStateValue.WAITTING_FOR_PICKUP
			# elif has_mission_box
			elif (self.vehicle_status=='wait command' and 
				self.web_control_state==WebControlStateValue.WAITTING_FOR_PICKUP and 
				alert_text == 'Choose a drop-off location'):
				self.choose_drop_off()
				self.web_control_state = WebControlStateValue.WAITTING_FOR_DROP
			elif ( self.web_control_state==WebControlStateValue.WAITTING_FOR_DROP):
				self.has_process, self.process_result = self.get_mission_process()
				goal_result = self.process_result[-1][1]
				if self.vehicle_status=='wait command' and btn_text=='drop product':
					self.drop_product()
					if goal_result:
						self.web_control_state=WebControlStateValue.READY
						self.has_process = False
					self.get_logger().info(f'Mission process: {self.has_process}, process_result: {self.process_result}')
			self.get_logger().debug(f'vehicle {self.agv_name} status: {self.vehicle_status}, web ctrl state: {self.web_control_state}, has_mission_alert: {has_mission_alert}, alert_text: {alert_text}')
			if self.has_process:
				self.get_logger().debug(f'process result: {self.process_result}')
				
	def waitting_for_button(self, button_xpath='', timeout=2):
		try:
			WebDriverWait(self.driver, timeout).until(EC.visibility_of_element_located((By.XPATH, button_xpath)))
			status_element = self.driver.find_element(By.XPATH, button_xpath)
			status_text = status_element.get_attribute("innerText").strip().lower()
			return True, status_text
		except TimeoutException:
			return False, 'Visible button timeout'
		except Exception as err:
			self.get_logger().fatal(f'Waitting button error: {err}')
			return False, f'Waitting button error: {err}'

	def wait_until_user_closes_browser(self):
		try:
			while True:
				if len(self.driver.window_handles) == 0:
					break
		except WebDriverException:
			pass

def test(args=None):
	rclpy.init(args=args)
	driver = TwotruckWebDriver()
	driver.get_website()
	print("Browser is open. Close the window to end the program.")
	driver.login()
	navbar = driver.get_navbar()
	navbar['Home'].click()
	driver.wait_for_element(driver.nav_xpath["Home"])
	try:
		# print('View AGV1')
		# driver.view_windows(vehicle='AGV1')
		# driver.create_mission()
		driver.choose_drop_off()
		# vehicle_mission = driver.get_vehicle_mission(vehicle='AGV1')
		# symbol, number = vehicle_mission[0], int(vehicle_mission[1:])
		# print(f'Vehicle mission id: {vehicle_mission}, type: {type(vehicle_mission)}, type num: {type(number)}')
		# driver.drop_product()
		driver.wait_until_user_closes_browser()
		print("Browser closed. Program exiting.")
		pass
	except Exception as err:
		print(f'Error: {err}')
	finally:
		driver.logout()
		driver.destroy_node()
		rclpy.shutdown()

def main(args=None):
	rclpy.init(args=args)
	driver = TwotruckWebDriver()
	driver.get_website()
	driver.login()
	navbar = driver.get_navbar()
	navbar['Home'].click()
	driver.wait_for_element(driver.nav_xpath['Home'])
	try:
		rclpy.spin(node=driver)
		driver.wait_until_user_closes_browser()
	except Exception as err:
		print(f"Node Error: {err}")
	finally:
		driver.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
