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

class ButtonWrapper:
	def __init__(self, element):
		self.element = element
	def press(self):
		self.element.click()
	def __repr__(self):
		return f"ButtonWrapper(text='{self.element.text.strip()}')"

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
		self.pickup_point = None
		self.drop_points = None
		self.main_loop = self.create_timer(timer_period_sec=self.loop_period_sec, callback=self.main)
		self.web_control_state = WebControlStateValue.INITIALIZING
		if self.debug_info:
			self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
		self.pickup_to_dropoffs = {
			"P01S": ['D01S','D01S','D03S','D04S','D05S','D06S','D07S','D08S','D09S','D10S','D11S','D12S','D13S','D14S','D15S','D17S','D19S'],
			"P02S": ['D01S','D01S','D03S','D04S','D05S','D06S','D07S','D08S','D09S','D10S','D11S','D12S','D13S','D14S','D15S','D17S','D19S'],
			"P03S": ['D01S','D01S','D03S','D04S','D05S','D06S','D07S','D08S','D09S','D10S','D11S','D12S','D13S','D14S','D15S','D17S','D19S'],
			"P04S": ['D15S','D20S','D21S','D22S'],
			"P05S": ['D01S','D01S','D03S','D04S','D05S','D06S','D07S','D08S','D09S','D10S','D11S','D12S','D13S','D14S','D15S','D17S','D19S'],
			"P06S": ['D16S','D20S','D22S'],
			"P07S": ['D21S'],
			"P08S": ['D21S']
		}
		self.conflict_pairs = [
			({"D07S","D08S","D09S","D10S"}, {"D05S","D06S","D11S","D12S"}),
			({"D13S","D14S","D20S"}, {"D18S","D19S"}),
			({"D21S"}, {"D22S"}),
		]
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
			"P07S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[7]',
			"P08S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[8]',
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
		self.agv_box_xpath = '/html/body/div/div/section/section/section[3]/div[1]'
		self.agv_display_xpath = '/html/body/div/div/section/section/section[3]/div[2]'

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
	
	def get_vehicle_velocity(self, agv_num=None):
		try:
			if agv_num == None:
				return 'Please assign the number of vehicles to the variable agv_num.'
			else:
				agv_vel_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[1]/div[2]/h1'
				agv_vel_unit_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[1]/div[2]/p'
			self.wait_for_element(agv_vel_xpath)
			vel_element = self.driver.find_element(By.XPATH, agv_vel_xpath)
			vel_text = vel_element.get_attribute('innerText').strip().lower()
			self.wait_for_element(agv_vel_unit_xpath)
			vel_unit_element = self.driver.find_element(By.XPATH, agv_vel_unit_xpath)
			vel_unit_text = vel_unit_element.get_attribute('innerText').strip().lower()
			return vel_text, vel_unit_text
		except NoSuchElementException:
			return "Mode element not found"
		except Exception as get_mode_err:
			err_msg = f'Get velocity error: {get_mode_err}'
			self.get_logger().error(err_msg)
			return err_msg
		
	def get_vehicle_mission(self, agv_num=None):
		try:
			if agv_num == None:
				return 'Please assign the number of vehicles to the variable agv_num.'
			else:
				agv_mission_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[2]/div[5]/p/span'
			WebDriverWait(self.driver, 10).until(EC.presence_of_element_located((By.XPATH, agv_mission_xpath)))
			WebDriverWait(self.driver, 10).until(lambda d: d.find_element(By.XPATH, agv_mission_xpath).text.strip() != "")
			element_text = self.driver.find_element(By.XPATH, agv_mission_xpath).text.lstrip()
			id = int(element_text.lstrip('#'))
			return id
		except TimeoutException:
			return "No mission assigned"
		except NoSuchElementException:
			return "Element not found"
	
	def get_vehicle_mode(self, agv_num=None):
		try:
			if agv_num == None:
				return 'Please assign the number of vehicles to the variable agv_num.'
			else:
				agv_mode_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[1]/div[1]/div[2]'
			self.wait_for_element(agv_mode_xpath)
			mode_element = self.driver.find_element(By.XPATH, agv_mode_xpath)
			mode_text = mode_element.get_attribute('innerText').strip().lower()
			return mode_text
		except NoSuchElementException:
			return "Mode element not found"
		except Exception as get_mode_err:
			err_msg = f'Get mode error: {get_mode_err}'
			self.get_logger().error(err_msg)
			return err_msg
	
	def get_vehicle_state(self, agv_num=''):
		agv2_state_xpath = ''
	
	def get_vehicle_status(self, agv_num=None): #status online/offline
		try:
			if agv_num == None:
				return 'Please assign the number of vehicles to the variable agv_num.'
			else:
				status_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[1]/div[1]/div[3]'
			self.wait_for_element(status_xpath)
			status_element = self.driver.find_element(By.XPATH, status_xpath)
			status_text = status_element.get_attribute("innerText").strip().lower()
			return status_text
		except NoSuchElementException:
			return "Element not found"
		except Exception as err:
			self.get_logger().error(f"Get vehicle status error: {err}")
			return 'get_status_err'
	
	def get_vehicle_battery(self, agv_num=None):
		try:
			if agv_num == None:
				return 'Please assign the number of vehicles to the variable agv_num.'
			else:
				agv_battery_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{agv_num}]/div[1]/div[1]/div[1]/div[2]/span'
			self.wait_for_element(agv_battery_xpath)
			battery_element = self.driver.find_element(By.XPATH, agv_battery_xpath)
			battery_text = battery_element.get_attribute('innerText').strip().lower()
			return battery_text
		except NoSuchElementException:
			return "Element not found"
		except Exception as err:
			self.get_logger().error(f"Get vehicle battery error: {err}")
			return 'get_battery_err'
	
	def get_mission_process(self, agv_num=None):
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
	
	def confirm_task(self):
		self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
		confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
		confirm.click()
	
	def has_agv_display(self, agv_num):
		pass

	def send_task(self):
		self.wait_for_element(self.pick_xpath["Send"], clickable=True)
		send = self.driver.find_element(By.XPATH, self.pick_xpath["Send"])
		send.click()

	def create_mission(self,):
		self.wait_for_element(self.create_btn_xpath, clickable=True)
		create_mission_btn = self.driver.find_element(By.XPATH, self.create_btn_xpath)
		create_mission_btn.click()
		pick_point = self.random_points(data_list=list(self.pickup_to_dropoffs.keys()))
		print(f'Pick up: {pick_point}')
		self.wait_for_element(self.pick_xpath[pick_point], clickable=True)
		pickup = self.driver.find_element(By.XPATH, self.pick_xpath[pick_point])
		pickup.click()
		print(f'Confirm pickup {pick_point}')
		return pick_point

	def choose_drop_off(self, drop_list):
		self.wait_for_element(self.choose_drop_xpath, clickable=True)
		choose_drop_button = self.driver.find_element(By.XPATH, self.choose_drop_xpath)
		choose_drop_button.click()
		drop_points = self.random_points(data_list=drop_list, type='drop')
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
		return drop_points
	
	def has_active_mission(self, mission_box_xpath):
		try:
			WebDriverWait(self.driver, 2).until(EC.presence_of_element_located((By.XPATH, mission_box_xpath)))
			mission_id = self.get_vehicle_mission(agv_num=self.agv_num)
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
	
	def rgba_result(self, rgba_str):
		rgba_str = rgba_str.strip()
		rgba_list = []
		if rgba_str.startswith("rgba(") and rgba_str.endswith(")"):
			values = rgba_str[5:-1].split(",")
			rgba_list = [int(v) if i < 3 else float(v) for i, v in enumerate(values)]
		if not rgba_list:
			return None
		def is_close(val1, val2, tol=10):
			return abs(val1 - val2) <= tol
		r, g, b = rgba_list[:3]
		if is_close(r, 255) and is_close(g, 106) and is_close(b, 0):
			return True  # active
		elif is_close(r, 135) and is_close(g, 135) and is_close(b, 135):
			return False  # inactive
		else:
			return rgba_list  # unknown color: return raw value

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
	
	def random_points(self, data_list, type=''):
		if type == 'drop':
			drop_points = []
			count_num = random.randint(1, 4)
			available_list = data_list.copy()
			for _ in range(count_num):
				if not available_list:
					break
				point = random.choice(available_list)
				drop_points.append(point)
				available_list.remove(point)
				for group1, group2 in self.conflict_pairs:
					if point in group1:
						available_list = [p for p in available_list if p not in group2]
					elif point in group2:
						available_list = [p for p in available_list if p not in group1]
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
	
	def get_agv_box(self):
		self.wait_for_element(self.agv_box_xpath)
		agv_box = self.driver.find_element(By.XPATH, self.agv_box_xpath)
		buttons = agv_box.find_elements(By.TAG_NAME, 'button')
		agv_buttons = {}
		for btn in buttons:
			label = btn.text.strip()
			agv_buttons[label] = ButtonWrapper(btn)
		return agv_buttons

	def main(self):
		self.vehicle_status = self.get_vehicle_status(agv_num=self.agv_num)
		agv_viel_btn = self.get_agv_box()
		# self.get_logger().info(str(agv_viel_btn))
		# agv_viel_btn['ALL'].press()
		# agv_viel_btn['AGV1'].press()
		# agv_viel_btn['AGV2'].press()
		if self.vehicle_status != 'offline' and self.vehicle_status != 'get_status_err':
			visible_btn, btn_text = self.waitting_for_button(self.create_btn_xpath)
			has_mission_box, mission_id, has_mission_alert, alert_text = self.has_active_mission(self.mission_box_xpath)
			if (btn_text=='create mission' and self.vehicle_status=='ready' and 
	   			(self.web_control_state==WebControlStateValue.INITIALIZING or self.web_control_state==WebControlStateValue.READY)):
				self.pickup_point = self.create_mission()
				self.send_task()
				self.confirm_task()
				self.web_control_state = WebControlStateValue.WAITTING_FOR_PICKUP
			elif (self.vehicle_status=='wait command' and 
				self.web_control_state==WebControlStateValue.WAITTING_FOR_PICKUP and 
				alert_text == 'Choose a drop-off location'):
				self.drop_points = self.choose_drop_off(drop_list=self.pickup_to_dropoffs[self.pickup_point])
				self.send_task()
				self.confirm_task()
				self.web_control_state = WebControlStateValue.WAITTING_FOR_DROP
			elif (self.web_control_state==WebControlStateValue.WAITTING_FOR_DROP):
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
		else:
			self.get_logger().error(f'vehicle {self.agv_name} status: {self.vehicle_status}')

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
