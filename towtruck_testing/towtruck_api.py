#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import threading
import csv
import math
from math import asin,atan2,cos,sin
from geometry_msgs.msg import PoseStamped ,Point , Quaternion
from std_msgs.msg import String, Bool
from towtruck_msgs.msg import RobotState, MissionStatus, RobotCommand, SystemRpt, SystemCmd, SensorEmergencyStamped, AlarmRpt, InitialPoseStatus
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_system_msgs.msg import AutowareState
from visualization_msgs.msg import MarkerArray
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, SetRoutePointsWithId 
# from autoware_auto_vehicle_msgs.msg import Engage as autoware_auto_vehicle_msgs_Engage
from autoware_auto_vehicle_msgs.msg import Engage
from diagnostic_msgs.msg import DiagnosticStatus
from functools import partial
from pathlib import Path
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import os
import subprocess
import yaml
import sys
import pandas as pd
import numpy as np
import time
import random
import Dijsktra_calculation
import sys,traceback

#====================================API=======================================================
from fastapi import FastAPI,status,HTTPException,Request,Response,Body,Depends,APIRouter
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, FileResponse #FileResponse -> return file instead
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from fastapi.middleware.cors import CORSMiddleware
from typing import List
from pydantic import BaseModel
import threading
import uvicorn
import requests
import json

app = FastAPI()

app.add_middleware(
	CORSMiddleware,
	allow_origins=["*"],
	allow_credentials=True,
	allow_methods=["*"],
	allow_headers=["*"],
)

class Item(BaseModel):
	id:int | None = None
	status:int | None = None
	vehicle_name:str | None = None
	paths:str | None = None
	nodes:str | None = None
	type:int| None = None
	transport_state:int| None = None

	# requester:str | None = None
	# mission_type:str | None = None
	# nodes:str | None = None
	# timestamp:str | None = None
	# dispatch_time:str | None = None
	# arriving_time:str | None = None
	# duration:str | None = None


class TowtruckApi(Node):
	def __init__(self):
		super().__init__("Towtruck_api")
		self.get_logger().info("Initial towtruck api")
		self.declare_parameter("main_server_ip",'192.168.1.14')
		self.declare_parameter("main_server_port",5000)
		self.declare_parameter("main_robot_ip",'192.168.1.34')
		self.declare_parameter("main_robot_port",5000)
		self.declare_parameter("csv_goal_path","data/BG/GoalPoints.csv")
		self.declare_parameter("simulation",True)
		self.declare_parameter("connect_to_server",True)
		self.declare_parameter("debug_info",True)
		self.main_server_ip = self.get_parameter("main_server_ip").get_parameter_value().string_value
		self.main_server_port = str(self.get_parameter("main_server_port").get_parameter_value().integer_value)
		self.main_robot_ip = self.get_parameter("main_robot_ip").get_parameter_value().string_value
		self.main_robot_port = str(self.get_parameter("main_robot_port").get_parameter_value().integer_value)
		self.csv_goal_path = self.get_parameter("csv_goal_path").get_parameter_value().string_value
		self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
		self.connect_to_server = self.get_parameter("connect_to_server").get_parameter_value().bool_value
		self.debug_info = self.get_parameter("debug_info").get_parameter_value().bool_value
		self.autoware = SystemRpt()
		self.goal_cmd = PoseStamped()

		self.DT = Dijsktra_calculation.Dijsktra(self.main_server_ip,self.main_server_port)

		self.received_system_rpt_time = None
		self.received_velocity_time = None
		self.current_node = "P01S"
		self.robot_state = RobotState()
		self.prev_robot_state = RobotState.STATE_ONLINE
		self.has_alarm = SystemRpt.ALARM_NONE
		self.has_obstacle = SystemRpt.NOT_DETECTED_OBSTACLE
		self.initial_state = InitialPoseStatus.STATE_NONE
		self.sensor_error_code = None
		self.prev_alarm_robot_state = None
		self.prev_pause_robot_state = None
		self.prev_obstacle_robot_state = None
		self.prev_autoware_state = None
		self.surround_obstacle_state = False
		self.horn_path = String()

		self.path_progression = list()
		self.pass_path = list()
		self.sub_mission = list()
		self.full_path = list()
		self.last_HB_timer = time.time()

#======================================AutowareService====================================================================
		self.engage_client = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_autonomous")
		self.overtake_client = self.create_client(SetRoutePointsWithId, "/planning/mission_planning/set_route_points_with_id")
		self.stop_client = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_stop")
		while not (self.engage_client.wait_for_service(timeout_sec=2.0) and self.stop_client.wait_for_service(timeout_sec=2.0) and self.overtake_client.wait_for_service(timeout_sec=2.0)) :
			self.get_logger().warn("Autoware service is not available ...")

#======================================ToAutoware=========================================================================
		self.goal_cmd_pub = self.create_publisher(PoseStamped,"/planning/mission_planning/goal",10)
		

#======================================FromAutoware======================================================================
		self.autoware_state_sub = self.create_subscription(AutowareState,"/autoware/state",self.AutowareStateCallback,10)
		self.surround_obstacle_marker_sub = self.create_subscription(MarkerArray,
										"/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/virtual_wall",
										self.surroundObstacleMarkerCallback,10)
#=======================================ToTowtruck========================================================================
		self.robot_state_pub = self.create_publisher(RobotState,"robot_state",10)
		self.robot_engage_pub = self.create_publisher(Engage,"/autoware/engage",10)
		self.engage_cmd = Engage()

#=======================================FromTowtruck======================================================================
		self.system_sub = self.create_subscription(SystemRpt,"/towtruck/system_rpt",self.systemCallback,10)
		self.obstacle_subscription = self.create_subscription(DiagnosticStatus ,"/planning/scenario_planning/status/stop_reason" ,self.obstacleCallback ,10)
		self.velocity_sub = self.create_subscription(VelocityReport,"/vehicle/status/velocity_status",self.velocityCallback,10)
		# self.alarm_sub = self.create_subscription(SensorEmergencyStamped,"/sensing/sensor_emergency",self.sensorAlarmCallback,10)
		self.emergency_sub = self.create_subscription(AlarmRpt,"/towtruck/alarm_rpt",self.emergencyCallback,10)
		self.initial_state_sub = self.create_subscription(InitialPoseStatus,"/towtruck_tools/initial_state",self.initialStateCallback,10)
		
		if self.simulation:
			self.tf_buffer = Buffer()
			self.tf_listener = TransformListener(self.tf_buffer, self)
			self.lookup_timer = self.create_timer(0.5,self.lookup2frame )
		else :
			self.position_subscription = self.create_subscription(PoseStamped, '/localization/pose_twist_fusion_filter/pose', self.positionCallback ,10)

		
		self.router = APIRouter()
		self.router.add_api_route("/check_HB", self.CheckHBCallback, methods=["GET"])
		self.router.add_api_route("/command/mission", self.MissionCallback, methods=["PUT"])
		self.router.add_api_route("/command/pause", self.PauseCallback, methods=["POST"])
		self.router.add_api_route("/command/continue", self.ContinueCallback, methods=["POST"])
		self.router.add_api_route("/command/next", self.NextCallback, methods=["POST"])
		self.router.add_api_route("/command/go_home", self.GoHomeCallback, methods=["POST"])
		
		self.api_path = "http://{}:{}".format(self.main_server_ip,self.main_server_port)

		r = requests_loop('get',self.api_path+"/fleet/vehicles")
		self.main_robot_ip = json.loads(r.text)['payload'][0]['ip_address']
		self.main_robot_port = json.loads(r.text)['payload'][0]['port']

		self.robot = {
			'name':json.loads(r.text)['payload'][0]['name'],
			'home': json.loads(r.text)['payload'][0]['home'],
			'node':None,
			'coordinate':'0.0,0.0,0.0',
			'state':RobotState.STATE_ONLINE,
			'battery':100,
			'ip_address':str(self.main_robot_ip),
			'mission_id':None,
			'port':str(self.main_robot_port),
			'velocity':None,
			'mode':SystemRpt.MANUAL_STATUS,
			'emergency_state':True,
			}



		self.robot_node_status={
			'reserving_node':[],
			'reserving_path':[],
			'releasing_node':[],
			'releasing_path':[],
			'reserving_next_path':[],
			'reserved':[],
			}

		self.cross_path = []
########################################### param for request #############################################
		self.robot_param = {
			"vehicle_name":self.robot['name']
		}
		self.reserve_path_param = {
			"paths":"",
			"vehicle_name":self.robot['name'],
		}
		self.release_path_param = {
			"start_node":"",
			"to_node":"",
			"vehicle_name":self.robot['name'],
		}
		self.reserve_node_param = {
			"node_name":"",
			"vehicle_name":self.robot['name'],
		}
		self.release_node_param = {
			"node_name":"",
			"vehicle_name":self.robot['name'],
		}
		self.reach25m_flag = False

		self.overtake_point = {
		'D04S':'D04A',
		'D05S':'D05A',
		'D06S':'D06A',
		'D07S':'D07A',
		'D08S':'D08A',
		'D09S':'D09A',
		'D10S':'D10A',
		'D11S':'D11A',
		'D12S':'D12A',
		'D17S':'D17A',
		'D18S':'D18A',
		'D19S':'D19A'}

###########################################################################################################

		self.mission = {
			"id":99,
			"status":0,
			"transport_state":0,
			"vehicle_name":self.robot['name'],
			"paths":"",
			"nodes":"",
			"type":1
			}
		##mission_status:#0-pending #1-accepted #2-started #3-terminated #4-rejected #5-canceled #6-failed
		##transport_state:#0-idle #1-loading #2-transit #3-unload #4-complete
		self.pose ={
			'x':0.0,
			'y':0.0,
			'yaw':0.0,
			'velocity':0.0,
			'score':0.0
		}
		
		if self.connect_to_server:
			passive_update_period = 1
			self.passive_update_timer = self.create_timer(passive_update_period, self.passive_update)

		
#=====================================GetRobotIpAdress=========================================================================
		


		self.timer_loop1 = self.create_timer(1,self.MainLoopCallback)
		self.FSM = FSM(self)

		##STATE
		self.FSM.AddState("SendGoal",SendGoal(self.FSM))
		self.FSM.AddState("Pause",Pause(self.FSM))
		self.FSM.AddState("Continue",Continue(self.FSM))
		self.FSM.AddState("Obstacle",Obstacle(self.FSM))
		self.FSM.AddState("ReachGoal",ReachGoal(self.FSM))
		self.FSM.AddState("GoHome",GoHome(self.FSM))
		self.FSM.AddState("Alarm",Alarm(self.FSM))
		self.FSM.AddState("StandBy",StandBy(self.FSM))
		self.FSM.AddState("AlarmReturnState",AlarmReturnState(self.FSM))
		self.FSM.AddState("Terminate",Terminate(self.FSM))

		##TRANSITIONS
		
		self.FSM.AddTransition("toSendGoal",Transition("SendGoal"))
		self.FSM.AddTransition("toPause",Transition("Pause"))
		self.FSM.AddTransition("toContinue",Transition("Continue"))
		self.FSM.AddTransition("toObstacle",Transition("Obstacle"))
		self.FSM.AddTransition("toReachGoal",Transition("ReachGoal"))
		self.FSM.AddTransition("toGoHome",Transition("GoHome"))
		self.FSM.AddTransition("toAlarm",Transition("Alarm"))
		self.FSM.AddTransition("toStandBy",Transition("StandBy"))
		self.FSM.AddTransition("toAlarmReturnState",Transition("AlarmReturnState"))
		self.FSM.AddTransition("toTerminate",Transition("Terminate"))

		self.robot_state.state = RobotState.STATE_ONLINE
		self.csv_goal_path = os.path.join(os.getcwd(),self.csv_goal_path)
		self.FSM.SetState("StandBy")
		self.Execute()
		self.GetNodePositionFromCsv()
		

	def Execute(self):
		self.FSM.Execute()

	def print_exc(self):
		exc_info=sys.exc_info()
		x=traceback.format_exception(*exc_info)
		print(x)

	def logger_exc(self):
		exc_info=sys.exc_info()
		x=traceback.format_exception(*exc_info)
		self.get_logger().error(f'{x}')

	def euler_from_quaternion(self,quaternion):
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w

		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = atan2(sinr_cosp, cosr_cosp)

		sinp = 2 * (w * y - z * x)
		pitch = asin(sinp)

		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = atan2(siny_cosp, cosy_cosp)
		return roll, pitch, yaw

	def lookup2frame (self) :
		try:
			t = self.tf_buffer.lookup_transform(
				"map",
				"base_link",
				rclpy.time.Time())
			_, _, yaw = self.euler_from_quaternion(t.transform.rotation)
			self.robot['coordinate'] = "{},{},{}".format(t.transform.translation.x,t.transform.translation.y,yaw)
			self.current_position = np.array([t.transform.translation.x,t.transform.translation.y])
		except Exception as txt:
			self.get_logger().warn(
				f'(API) Could not transform base_link to map: {txt}')
			
	def GetNodePositionFromCsv(self):
		self.goal_data_frame = pd.read_csv(self.csv_goal_path)

	def prepare_mission(self):
		if not self.pass_path:
			self.pass_path.append(self.robot['node'])
		mission_nodes = self.mission['nodes'].split(",")
		self.sub_mission,self.path,self.path_progression,_ = self.DT.CalPathWithVia(self.robot["node"],mission_nodes)
		if self.robot["node"] == self.sub_mission[0] and len(self.sub_mission)>1:
			self.sub_mission.pop(0)
		self.update_full_path()
		print(self.sub_mission,self.path)


		for item in self.path:
			if item == "Route Not Possible":
				self.missionStatusManager(RobotCommand.COMMAND_REJECTED)
				self.robotStateManager(RobotCommand.COMMAND_REJECTED)
				if self.debug_info:
					self.get_logger().error(f"Can not find a path")

	def prepare_overtake_mission(self,overtake_node):
		progress1=[]
		progress2=[]
		self.path_progression = [[]]
		_,_,progress1,_ = self.DT.CalPathWithVia(self.robot["node"],[overtake_node])
		_,_,progress2,_ = self.DT.CalPathWithVia(overtake_node,self.sub_mission)
		self.path_progression[0].extend(progress1[0])
		self.path_progression[0].extend(progress2[0])
		progress2.pop(0)
		if progress2:
			self.path_progression.extend(progress2)
		self.update_full_path()
		r = requests_loop('put',self.api_path+"/fleet/mission_update",self.mission)

	def GetGoalPointsFromCsv(self):
		goal_station = str(self.sub_mission[0])
		# self.horn_path.data = str(self.path).strip('[]').replace('\'','')
		df = pd.read_csv(self.csv_goal_path)
		df = df.loc[df['station'] == goal_station]
		if self.debug_info:
			self.get_logger().info(f'{df}')
			
		try:
			self.goal_cmd = PoseStamped()
			self.goal_cmd.header.frame_id = "map"
			self.goal_cmd.header.stamp = self.get_clock().now().to_msg()
			self.goal_cmd.pose.position.x = float(df['x'])
			self.goal_cmd.pose.position.y = float(df['y'])
			self.goal_cmd.pose.position.z = float(df['z'])
			self.goal_cmd.pose.orientation.x = float(df['qx'])
			self.goal_cmd.pose.orientation.y = float(df['qy'])
			self.goal_cmd.pose.orientation.z = float(df['qz'])
			self.goal_cmd.pose.orientation.w = float(df['qw'])
			

		except Exception as txt:
			self.missionStatusManager(RobotCommand.COMMAND_REJECTED)
			self.get_logger().error(f"GetGoalPointsFromCsv:={txt}")
			
		
	def MainLoopCallback(self):
		self.initialStateManager()
		self.autowareStateManager()
		self.obstacleStateManager()
		self.alarmStateManager()
		# if (self.robot["state"] == RobotState.STATE_RUNNING or
		# 	self.robot["state"] == RobotState.STATE_GOHOME):
		# if (self.robot["state"] == RobotState.STATE_RUNNING):
		# 	self.CheckCurrentNode()
		self.CheckCurrentNode()
		
		self.robot_state_pub.publish(self.robot_state)
		

	def MainLoop(self):
		pass

	def passive_update(self):
		if time.time()-self.last_HB_timer > 60:
			self.reset_robot()
		print('robot_node', self.robot['node'])
		print('sub_mission', self.sub_mission)
		print('path_progression', self.path_progression)
		# print('robot_node_status', self.robot_node_status)
		try:
			if self.debug_info:
				self.get_logger().info(f'RobotState:={self.robot["state"]},{self.robot["mission_id"]},MissionStatus:={self.mission["status"]},{self.mission["transport_state"]}')
			r = requests.put(self.api_path+"/fleet/vehicle_update",json.dumps(self.robot),timeout = 2)
		except Exception as e:
			self.get_logger().info(f'{e}')
			pass
	
	def CheckCurrentNode(self):
		# self.get_logger().info(f'{self.path}')
		try:
			if self.path_progression:
				for node in self.path_progression[0]:
					# print('check node:',self.path_progression)
					current_data_frame = self.goal_data_frame.loc[self.goal_data_frame['station'] == node] 
					sub_goal = np.array([float(current_data_frame["x"]),float(current_data_frame["y"])])
					dp_incoming = sub_goal - self.current_position
					abs_distance_incoming = np.linalg.norm(dp_incoming)
					# print(abs_distance_incoming)
					if abs_distance_incoming < 4.0:
						self.reach25m_flag = False
						self.robot_node_status['releasing_path'].append([self.robot["node"],self.path_progression[0][0]])
						self.robot["node"] = node
						while self.path_progression[0]:
							print('pop')
							if self.path_progression[0][0] == self.robot["node"]:
								self.pass_path.append(self.path_progression[0][0])
								self.path_progression[0].pop(0)
								break
							else:
								self.pass_path.append(self.path_progression[0][0])
								self.path_progression[0].pop(0)
						if self.path_progression[0] != []:
							self.robot_node_status['reserving_path'].append([self.robot["node"],self.path_progression[0][0]])

					current_node_frame = self.goal_data_frame.loc[self.goal_data_frame['station'] == self.robot['node']]
					sub_goal = np.array([float(current_node_frame["x"]),float(current_node_frame["y"])])
					dp_outgoing = sub_goal - self.current_position
					abs_distance_outgoing = np.linalg.norm(dp_outgoing)
					if abs_distance_outgoing > 25 and self.reach25m_flag == False:
						self.reach25m_flag = True
						self.robot_node_status['releasing_node'].append([self.robot["node"]])
						if [self.path_progression[0][0],self.path_progression[0][1]] in self.cross_path:
							self.robot_node_status['reserving_next_path'].append([self.path_progression[0][0],self.path_progression[0][1]])

					

			if self.robot_node_status['reserving_path']:
				try:
					self.reserve_path_param["paths"] = str([[self.robot_node_status['reserving_path'][0][0],self.robot_node_status['reserving_path'][0][1]]])
					reserve=requests.put(self.api_path+"/node/reserve_bypath",params = self.reserve_path_param)
					print(reserve.text)
					if json.loads(reserve.text)['detail'] != "Done":
						self.pub_disengage()
						if len(self.path_progression[0])>1 and self.robot_node_status['reserving_path'][0][1] in self.overtake_point.keys():
							self.reserve_path_param["paths"] = str([[self.robot_node_status['reserving_path'][0][0],self.overtake_point[self.robot_node_status['reserving_path'][0][1]]],[self.overtake_point[self.robot_node_status['reserving_path'][0][1]],self.path_progression[0][1]]])
							overtake_reserve=requests.put(self.api_path+"/node/reserve_bypath",params = self.reserve_path_param)
							print(self.reserve_path_param["paths"],overtake_reserve.text)
							if json.loads(overtake_reserve.text)['detail'] != "Done":
								self.pub_disengage()
							else:

								self.overtake_service_request([self.overtake_point[self.robot_node_status['reserving_path'][0][1]],self.path_progression[0][-1]])
								self.prepare_overtake_mission(self.overtake_point[self.robot_node_status['reserving_path'][0][1]])
								self.pub_engage()
								self.robot_node_status['reserving_path'].pop(0)
					else:
						self.pub_engage()
						self.robot_node_status['reserving_path'].pop(0)
				except:
					r="retrying reserve"
			if self.robot_node_status['releasing_path']:
				try:
					self.release_path_param["start_node"],self.release_path_param["to_node"] = self.robot_node_status['releasing_path'][0][0],self.robot_node_status['releasing_path'][0][1]
					release=requests.put(self.api_path+"/node/release_bypath",params = self.release_path_param)
					if json.loads(release.text)['detail'] == "Done":
						self.robot_node_status['releasing_path'].pop(0)
				except:
					pass

			if self.robot_node_status['releasing_node']:
				try:
					self.release_node_param["node_name"] = self.robot_node_status['releasing_node'][0][0]
					release=requests.put(self.api_path+"/node/release_bynode",params = self.release_node_param)
					if json.loads(release.text)['detail'] == "Done":
						self.robot_node_status['releasing_node'].pop(0)
				except:
					pass
			if self.robot_node_status['reserving_next_path']:
				try:
					self.reserve_path_param["paths"] = str([[self.robot_node_status['reserving_next_path'][0][0],self.robot_node_status['reserving_next_path'][0][1]]])
					reserve=requests.put(self.api_path+"/node/reserve_bypath",params = self.reserve_path_param)
					if json.loads(reserve.text)['detail'] != "Done":
						pass
					else:
						self.robot_node_status['reserving_next_path'].pop(0)
				except:
					r="retrying reserve next path"

		except Exception as txt:
			self.logger_exc()

	def SendGoalCommand(self):
		if self.robot['state'] != RobotState.STATE_WAITING or self.mission['transport_state'] == 2:
			self.prepare_mission()
		overtake_flag =False
		if self.mission['status'] != MissionStatus.STATE_REJECTED:
			# self.robot['mission_id'] = self.mission['id']
			self.robot['state'] = RobotState.STATE_RESERVING
			self.mission['paths'] = str(self.full_path).strip('[]').replace('\'','')
			self.mission["vehicle_assign"] = self.robot['name']
			if self.connect_to_server:
				r = requests_loop('put',self.api_path+"/fleet/mission_update",self.mission)
				try:
					retry_time = time.time()
					while True:  
						if time.time() - retry_time>1:
							retry_time = time.time()
							self.reserve_path_param["paths"] = str([[self.robot["node"],self.path_progression[0][0]]])
							reserve = requests_loop('put',self.api_path+"/node/reserve_bypath",param = self.reserve_path_param)
							if json.loads(reserve.text)['detail'] != "Done":
								if len(self.path_progression[0])>1 and self.path_progression[0][0] in self.overtake_point.keys():
									self.reserve_path_param["paths"] = str([[self.robot['node'],self.overtake_point[self.path_progression[0][0]]],[self.overtake_point[self.path_progression[0][0]],self.path_progression[0][1]]])
									overtake_reserve=requests.put(self.api_path+"/node/reserve_bypath",params = self.reserve_path_param)
									print(self.reserve_path_param["paths"],overtake_reserve.text)
									if json.loads(overtake_reserve.text)['detail'] != "Done":
										continue
									else:
										self.prepare_overtake_mission(self.overtake_point[self.path_progression[0][0]])
										self.overtake_service_request([self.path_progression[0][0],self.path_progression[0][-1]])
										overtake_flag =True
										print('overtake flag', overtake_flag)
										break
											
							else:
								self.overtake_service_request(self.path_progression[0])
								# self.goal_cmd_pub.publish(self.goal_cmd)
								break
				except:
					pass         
			
			publish_time = time.time()
			time_stamp = time.time()
			publish_goal_succeed = False
			while True:
				if not publish_goal_succeed:
					if self.autoware.state == SystemRpt.STATE_WAITING_FOR_ENGAGE or self.autoware.state == SystemRpt.STATE_PLANNING:
						publish_goal_succeed = True
						if self.debug_info:
							self.get_logger().info("Published goal")
					elif time.time() - publish_time > 1.0:
						if self.debug_info:
							self.get_logger().info("tried to publish goal ...")
						if overtake_flag :
							self.overtake_service_request([self.path_progression[0][0],self.path_progression[0][-1]])
						else:
							self.overtake_service_request(self.path_progression[0])
							# self.goal_cmd_pub.publish(self.goal_cmd)
						publish_time = time.time()
					elif self.autoware.state == SystemRpt.STATE_ARRIVAL_GOAL:
						break
				else:
					if time.time() - time_stamp > 1:
						time_stamp = time.time()
						if self.autoware.state == SystemRpt.STATE_DRIVING:
							if self.debug_info:
								self.get_logger().info("Engage!!")
							node.missionStatusManager(RobotCommand.COMMAND_GO)
							node.robotStateManager(RobotCommand.COMMAND_GO)
							break
						elif self.autoware.state == SystemRpt.STATE_WAITING_FOR_ENGAGE or self.autoware.state == SystemRpt.STATE_PLANNING:
							self.engage_service_request()
							if self.debug_info:
								self.get_logger().info("Tried to engage...")

	def SendGoalHomeCommand(self):
		self.goal_cmd_pub.publish(self.goal_cmd)
		publish_time = time.time()
		time_stamp = time.time()
		publish_goal_succeed = False
		while True:
			if not publish_goal_succeed:
				if self.autoware.state == SystemRpt.STATE_WAITING_FOR_ENGAGE or self.autoware.state == SystemRpt.STATE_PLANNING:
					publish_goal_succeed = True
					if self.debug_info:
						self.get_logger().info("Published goal")
				elif time.time() - publish_time > 1.0:
					if self.debug_info:
						self.get_logger().info("tried to publish goal ...")
					self.goal_cmd_pub.publish(self.goal_cmd)
					publish_time = time.time()
				elif self.autoware.state == SystemRpt.STATE_ARRIVAL_GOAL:
					break
			else:
				if time.time() - time_stamp > 1:
					time_stamp = time.time()
					if self.autoware.state == SystemRpt.STATE_DRIVING:
						if self.debug_info:
							self.get_logger().info("Engage!!")
						node.missionStatusManager(RobotCommand.COMMAND_GOHOME)
						node.robotStateManager(RobotCommand.COMMAND_GOHOME)
						break
					elif self.autoware.state == SystemRpt.STATE_WAITING_FOR_ENGAGE or self.autoware.state == SystemRpt.STATE_PLANNING:
						self.engage_service_request()
						if self.debug_info:
							self.get_logger().info("Tried to engage...")

	def sentTerminateCommand(self):
		node.missionStatusManager(RobotCommand.COMMAND_TERMINATED)
		node.robotStateManager(RobotCommand.COMMAND_TERMINATED)

	def update_full_path(self):
		self.full_path = []
		self.full_path.extend(self.pass_path)
		for node in self.path_progression:
			self.full_path.extend(node)
		self.mission['paths'] = str(self.full_path).strip('[]').replace('\'','')

					
	def sendPauseCommand(self):
		time_stamp = time.time()
		self.stop_service_request()
		while True:
			if time.time() - time_stamp > 1: 
				time_stamp = time.time()
				if self.autoware.state != SystemRpt.STATE_DRIVING:
					if self.debug_info:
						self.get_logger().info("Stop!!")
					if self.robot['state'] != RobotState.STATE_ALARM:
						node.robotStateManager(RobotCommand.COMMAND_PAUSE)
					break
				else:
					self.stop_service_request()
					if self.debug_info:
						self.get_logger().info("Tried to Pause...")

	def errorCodeToDescription(self,argument):
		match argument:
			case SystemRpt.ALARM_A01:
				return "{}|{}".format(SystemRpt.ALARM_A01_EN,SystemRpt.ALARM_A01_TH)
			case SystemRpt.ALARM_A03:
				return "{}|{}".format(SystemRpt.ALARM_A03_EN,SystemRpt.ALARM_A03_TH)
			case SystemRpt.ALARM_A04:
				return "{}|{}".format(SystemRpt.ALARM_A04_EN,SystemRpt.ALARM_A04_TH)
			case SystemRpt.ALARM_A02:
				return "{}|{}".format(SystemRpt.ALARM_A02_EN,SystemRpt.ALARM_A02_TH)
			case SystemRpt.ALARM_V01:
				return "{}|{}".format(SystemRpt.ALARM_V01_EN,SystemRpt.ALARM_V01_TH)
			case SystemRpt.ALARM_V02:
				return "{}|{}".format(SystemRpt.ALARM_V02_EN,SystemRpt.ALARM_V02_TH)
			case SystemRpt.ALARM_V11:
				return "{}|{}".format(SystemRpt.ALARM_V11_EN,SystemRpt.ALARM_V11_TH)
			case SystemRpt.ALARM_V12:
				return "{}|{}".format(SystemRpt.ALARM_V12_EN,SystemRpt.ALARM_V12_TH)
			case SystemRpt.ALARM_V13:
				return "{}|{}".format(SystemRpt.ALARM_V13_EN,SystemRpt.ALARM_V13_TH)
			case SystemRpt.ALARM_V21:
				return "{}|{}".format(SystemRpt.ALARM_V21_EN,SystemRpt.ALARM_V21_TH)
			case SystemRpt.ALARM_V31:
				return "{}|{}".format(SystemRpt.ALARM_V31_EN,SystemRpt.ALARM_V31_TH)
			case SystemRpt.ALARM_V32:
				return "{}|{}".format(SystemRpt.ALARM_V32_EN,SystemRpt.ALARM_V32_TH)
			case SystemRpt.ALARM_V33:
				return "{}|{}".format(SystemRpt.ALARM_V33_EN,SystemRpt.ALARM_V33_TH)
			case SystemRpt.ALARM_V40:
				return "{}|{}".format(SystemRpt.ALARM_V40_EN,SystemRpt.ALARM_V40_TH)
			case SystemRpt.ALARM_SL1:
				return "{}|{}".format(SystemRpt.ALARM_SL1_EN,SystemRpt.ALARM_SL1_TH)
			case SystemRpt.ALARM_SR1:
				return "{}|{}".format(SystemRpt.ALARM_SR1_EN,SystemRpt.ALARM_SR1_TH)
			case SystemRpt.ALARM_ST1:
				return "{}|{}".format(SystemRpt.ALARM_ST1_EN,SystemRpt.ALARM_ST1_TH)
			case SystemRpt.ALARM_SI1:
				return "{}|{}".format(SystemRpt.ALARM_SI1_EN,SystemRpt.ALARM_SI1_TH)
			case default:
				return "something error"
			
	def sendAlarmCommand(self):
		time_stamp = time.time()
		error_msg = list()
		error_description = String()
		self.stop_service_request()
		node.robotStateManager(RobotCommand.COMMAND_ALARM)
		for e in self.sensor_error_code:
			error_description = self.errorCodeToDescription(e)
			error_msg.append({"code": e ,"vehicle_name": self.robot['name']})
		if self.connect_to_server:
			r = requests.post(self.api_path+"/alarm/create",json.dumps(error_msg),timeout = 1)
		while True:
			if time.time() - time_stamp > 1: 
				time_stamp = time.time()
				if self.autoware.state != SystemRpt.STATE_DRIVING:
					if self.debug_info:
						self.get_logger().info("Alarm stop!!")
					break
				else:
					self.stop_service_request()
					if self.debug_info:
						self.get_logger().info("Tried to Alarm stop...")

	def sendAlarmReturnState(self):
		time_stamp = time.time()
		while True:
			if  (
				self.prev_alarm_robot_state == RobotState.STATE_RUNNING or
				self.prev_alarm_robot_state == RobotState.STATE_OBSTACLE
				):
				if time.time() - time_stamp > 1:
					time_stamp = time.time()
					pass
					if  (
						self.autoware.state == SystemRpt.STATE_DRIVING or
						self.autoware.state == SystemRpt.STATE_ARRIVAL_GOAL or
						self.autoware.state == SystemRpt.STATE_WAITING_FOR_ROUTE
						):
						if self.debug_info:
							self.get_logger().info("Engage!!")
						break
					elif self.autoware.state == SystemRpt.STATE_WAITING_FOR_ENGAGE or self.autoware.state == SystemRpt.STATE_PLANNING:
						self.engage_service_request()
						if self.debug_info:
							self.get_logger().info("Tried to engage...")


	def sendContinueCommand(self):
		if self.robot['state'] == RobotState.STATE_PAUSE:
			time_stamp = time.time()
			self.engage_service_request()
			while True:
				if time.time() - time_stamp > 1: 
					time_stamp = time.time()
					if self.autoware.state != SystemRpt.STATE_WAITING_FOR_ENGAGE:
						if self.debug_info:
							self.get_logger().info("Continue!!")
						node.robotStateManager(RobotCommand.COMMAND_CONTINUE)
						break
					else:
						self.engage_service_request()
						if self.debug_info:
							self.get_logger().info("Tried to Continue...")

				
	def engage_service_request(self):
		engage_request = ChangeOperationMode.Request()
		future = self.engage_client.call_async(engage_request)
		future.add_done_callback(partial(self.engage_callback_set))
		if self.debug_info:
			self.get_logger().info("Call service request engage....")

	def engage_callback_set(self,future:rclpy.Future):
		try:
			response = future.result()
			# print(response.status)
		except Exception as e:
			self.get_logger().error(f"Autoware engage service call failed : {e}")


	def overtake_service_request(self,node_list):
		overtake_request = SetRoutePointsWithId.Request()
		overtake_request.ids = node_list  # List of route point IDs
		overtake_request.mode = True  # Set the mode to True
		future = self.overtake_client.call_async(overtake_request)
		future.add_done_callback(self.overtake_response_callback)

	def overtake_response_callback(self, future):
		try:
			# Get the response from the service
			response = future.result()
			self.get_logger().info(f'Service response: {response}')
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
	
	def stop_service_request(self):
		stop_request = ChangeOperationMode.Request()
		future = self.stop_client.call_async(stop_request)
		if self.debug_info:
			self.get_logger().info("Call service request stop....")


	def robotStateManager(self,command):
		if command == RobotCommand.COMMAND_RETURN_COMMAND:
			if self.prev_robot_state == RobotState.STATE_ALARM:
				self.robot['state'] = self.prev_alarm_robot_state
			elif self.prev_robot_state == RobotState.STATE_OBSTACLE:
				self.robot['state'] = self.prev_obstacle_robot_state

		elif command == RobotCommand.COMMAND_ALARM:
			self.robot['state'] = RobotState.STATE_ALARM
			if self.prev_robot_state == RobotState.STATE_PAUSE:
				self.prev_alarm_robot_state = self.prev_pause_robot_state
			elif self.prev_robot_state == RobotState.STATE_OBSTACLE:
				self.prev_alarm_robot_state = self.prev_obstacle_robot_state
			else:
				self.prev_alarm_robot_state = self.prev_robot_state

		elif command == RobotCommand.COMMAND_PAUSE:
			self.robot['state'] = RobotState.STATE_PAUSE
			if self.prev_robot_state != RobotState.STATE_PAUSE:
				self.prev_pause_robot_state = self.prev_robot_state
			if self.prev_robot_state == RobotState.STATE_OBSTACLE:
				self.prev_pause_robot_state = self.prev_obstacle_robot_state

		elif command == RobotCommand.COMMAND_OBSTACLE:
			self.robot['state'] = RobotState.STATE_OBSTACLE
			self.prev_obstacle_robot_state = self.prev_robot_state

		elif command == RobotCommand.COMMAND_CONTINUE:
			self.robot['state'] = self.prev_pause_robot_state

		elif command == RobotCommand.COMMAND_REACHGOAL:
			if self.mission['type'] == 0:
				self.robot['state'] = RobotState.STATE_STANDBY
				self.robot['mission_id'] = None
			elif self.mission['type'] == 1:
				self.robot['state'] = RobotState.STATE_WAITING
			# if self.prev_robot_state == RobotState.STATE_GOHOME:
			# 	self.robot['state'] = RobotState.STATE_STANDBY
			# else:
			# 	self.robot['state'] = RobotState.STATE_WAITING

		elif command == RobotCommand.COMMAND_GO:
			self.robot['state'] = RobotState.STATE_RUNNING
		# elif command == RobotCommand.COMMAND_GOHOME:
		# 	self.robot['state'] = RobotState.STATE_GOHOME

		elif command == RobotCommand.COMMAND_STANDBY:
			self.robot['state'] = RobotState.STATE_STANDBY

		elif command == RobotCommand.COMMAND_TERMINATED:
			self.robot['state'] = RobotState.STATE_STANDBY
			self.robot['mission_id'] = None

		elif command == RobotCommand.COMMAND_REJECTED:
			self.robot['mission_id'] = None
		
		self.robot_state.state = self.robot["state"]
		self.robot_state.path = self.horn_path.data
		self.prev_robot_state = self.robot["state"]
		if self.debug_info:
			self.get_logger().info(f"robot state:={self.robot['state']}")

		if self.connect_to_server:
			r = requests.put(self.api_path+"/fleet/vehicle_update",json.dumps(self.robot),timeout = 2)
		
		
	def missionStatusManager(self,command):
		if command == RobotCommand.COMMAND_GO:
			if self.mission['status'] == MissionStatus.STATE_PENDING:
				self.mission['status'] = MissionStatus.STATE_STARTED
			elif self.mission['status'] == MissionStatus.STATE_STARTED:
				self.mission['transport_state'] = 3
		elif command == RobotCommand.COMMAND_REACHGOAL:
			if self.mission['type'] == 1 and len(self.mission['nodes'].split(","))==1:
				self.mission['transport_state'] = 1
				self.sub_mission.pop(0)
				self.path_progression.pop(0)
			elif self.mission['type'] == 0 and len(self.mission['nodes'].split(","))==1:
				self.mission['status'] = MissionStatus.STATE_TERMINATED
				self.mission['transport_state'] = 5
				self.sub_mission.pop(0)
				self.path_progression.pop(0)
				self.pass_path = []
			elif len(self.mission['nodes'].split(","))>1 and len(self.sub_mission) >= 1 and self.robot['node'] == self.sub_mission[0]:
				self.mission['transport_state'] = 4
				self.sub_mission.pop(0)		
				self.path_progression.pop(0)
		elif command == RobotCommand.COMMAND_REJECTED:
			self.mission['status'] = MissionStatus.STATE_REJECTED
		elif command == RobotCommand.COMMAND_TERMINATED:
			self.mission['status'] = MissionStatus.STATE_TERMINATED
			self.mission['transport_state'] = 5
			self.pass_path = []
			
			
		if self.debug_info:
			self.get_logger().info(f"mission status:={self.mission['status']}")  

		if self.connect_to_server:
			if self.robot['mission_id'] != None:
				r = requests_loop('put',self.api_path+"/fleet/mission_update",self.mission) 

	def initialStateManager(self):
		if self.robot['state'] == RobotState.STATE_ONLINE:
			# if self.initial_state == InitialPoseStatus.STATE_SUCCEED:
			if True:
				while True:
					self.reserve_node_param["node_name"] = self.robot["home"]
					reserve = requests_loop('put',self.api_path+"/node/reserve_bynode",param = self.reserve_node_param)
					if json.loads(reserve.text)['detail'] != "Done":
						continue
					else:
						self.robotStateManager(RobotCommand.COMMAND_STANDBY)
						self.robot['node'] = self.robot['home']
						break

	def autowareStateManager(self):
		if self.autoware.state != self.prev_autoware_state:
			self.prev_autoware_state = self.autoware.state
			if self.autoware.state == AutowareState.ARRIVED_GOAL:
				self.FSM.ToTransition("toReachGoal")
				self.FSM.Execute()

	def obstacleStateManager(self):
		if self.has_obstacle or self.surround_obstacle_state:
			if  (
				self.prev_robot_state != RobotState.STATE_OBSTACLE and
					(self.robot['state'] == RobotState.STATE_RUNNING or
					self.robot['state'] == RobotState.STATE_GOHOME
					)
				):
				self.FSM.ToTransition("toObstacle")
				self.FSM.Execute()
		elif self.prev_robot_state == RobotState.STATE_OBSTACLE:
			self.robotStateManager(RobotCommand.COMMAND_RETURN_COMMAND)

	def alarmStateManager(self):
		if self.has_alarm:
			if self.prev_robot_state != RobotState.STATE_ALARM:
				self.FSM.ToTransition("toAlarm")
				self.FSM.Execute()
		elif self.prev_robot_state == RobotState.STATE_ALARM:
			self.robotStateManager(RobotCommand.COMMAND_RETURN_COMMAND)
			self.FSM.ToTransition("toAlarmReturnState")
			self.FSM.Execute()

	def AutowareStateCallback(self,msg):
		self.autoware.state = msg.state
	
	def surroundObstacleMarkerCallback(self,msg:MarkerArray):
		self.surround_obstacle_state = True if len(msg.markers) > 0 else False

		

	def systemCallback(self,msg:SystemRpt):
		self.received_system_rpt_time = time.time()
		self.robot["battery"] = msg.battery
		self.has_alarm = msg.has_alarm
		if msg.drive_mode == SystemRpt.MODE_AUTO:
			self.robot["mode"] = SystemRpt.AUTO_STATUS
		else:
			self.robot["mode"] = SystemRpt.MANUAL_STATUS

		self.sensor_error_code = msg.error_code
	

	def velocityCallback(self,msg:VelocityReport):
		self.received_velocity_time = time.time()
		self.robot["velocity"] = msg.longitudinal_velocity * 3.6
	

	def positionCallback(self,msg:PoseStamped):
		self.received_pose_time = time.time()
		_, _, yaw = self.euler_from_quaternion(msg.pose.orientation)
		self.robot['coordinate'] = "{},{},{}".format(msg.pose.position.x,msg.pose.position.y,yaw)
		self.current_position = np.array([msg.pose.position.x,msg.pose.position.y])


	def obstacleCallback(self,msg:DiagnosticStatus):
		if msg.message:
			self.has_obstacle = SystemRpt.DETECTED_OBSTACLE
		else:
			self.has_obstacle = SystemRpt.NOT_DETECTED_OBSTACLE


	def emergencyCallback(self,msg:AlarmRpt):
		self.robot["emergency_state"] = msg.alarm_emergency_button
			
	def initialStateCallback(self,msg:InitialPoseStatus):
		self.initial_state = msg.status

	def pingTime(self,count=1, wait_sec=1):
		if os.system("ping -c 1 " + self.main_robot_ip +">/dev/null 2>&1") == 0:
			cmd = "ping -c {} -W {} {}".format(count, wait_sec, self.main_robot_ip).split(' ')
			try:
				output = subprocess.check_output(cmd).decode().strip()
				lines = output.split("\n")
				time = float(lines[1].split('=')[3].split()[0])
				return time
			except Exception as txt:
				self.get_logger().warn( f"ping time error:{txt}")
				return -1
						
	def CheckHBCallback(self):
		self.last_HB_timer = time.time()
		# self.get_logger().info("GetHB")

	def reset_robot(self):
		self.robot['mission_id'] = None
		self.robot['state'] = RobotState.STATE_ONLINE
		self.robot['node'] = None

	def MissionCallback(self,item:Item):
		if self.debug_info:
			self.get_logger().info("GetMission")
		print(item.dict()['id'] ,self.robot['mission_id'] ,item.dict()['transport_state'])
		if (not self.robot['mission_id']) or (item.dict()['id'] == self.robot['mission_id'] and item.dict()['transport_state'] == 2):
			self.mission = item.dict()
			self.robot['mission_id'] = self.mission['id']
			self.FSM.ToTransition("toSendGoal")
			self.FSM.Execute()
		else:
			return 99
		
		
	def PauseCallback(self):
		if self.debug_info:
			self.get_logger().info("GetPause")
		self.FSM.ToTransition("toPause")
		self.FSM.Execute()
		
		
	
	def ContinueCallback(self):
		if self.debug_info:
			self.get_logger().info("GetContinue")
		self.FSM.ToTransition("toContinue")
		self.FSM.Execute()

	def NextCallback(self):
		if self.debug_info:
			self.get_logger().info("GetNext")
		if self.robot['state'] == RobotState.STATE_WAITING and self.sub_mission != []:
			self.FSM.ToTransition("toSendGoal")
			self.FSM.Execute()
		elif self.robot['state'] == RobotState.STATE_WAITING and self.sub_mission == [] and self.mission['transport_state'] == 4 :
			self.FSM.ToTransition("toTerminate")
			self.FSM.Execute()
		
	def GoHomeCallback(self,item:Item):
		if self.debug_info:
			self.get_logger().info("GetGoHome")
		self.mission = item.dict()
		self.mission["position"] = ",{}".format(self.robot["home"])
		self.FSM.ToTransition("toGoHome")
		self.FSM.Execute()

	def pub_engage(self):
		print('pub_engage')
		self.engage_cmd.engage = True
		self.robot_engage_pub.publish(self.engage_cmd)

	def pub_disengage(self):
		print('pub_disengage')
		self.engage_cmd.engage = False
		self.robot_engage_pub.publish(self.engage_cmd)
		

def requests_loop(methods,path,args = '',param = None):
	loop_time = time.time()
	first_round = True
	while True:
		if (first_round == True or (time.time()-loop_time)>=2 ):
			if (first_round == False) :
				print ('retry requests_loop')
			try: 
				if (methods == 'put'):
					r=requests.put(path,json=args,timeout = 2, params=param)
					return r
				elif (methods == 'post'):
					r=requests.post(path,json=args,timeout = 2, params=param)
					return r
				elif (methods == 'get'):
					r=requests.get(path,timeout = 2, params=param)
					return r
				else:
					break
			except Exception as e:
				print('request exception:', e, '\n\t', methods, '\t', path)
				first_round = False
				loop_time = time.time()
				continue
		else:
			pass

def run_api_server(ip,port):
	uvicorn.run(app, host=ip, port=int(port), reload=False)
	
class Transition():
	def __init__(self,toState):
		self.toState=toState
		
	def Execute(self):
		print("Transitioning to " ,self.toState)


class State():
	def __init__(self,FSM):
		self.FSM = FSM
		self.timer = 0
		self.startTime = 0

	def Enter(self):
		self.timer = random.randint(0,5)
		self.startTime = time.time()

	def Execute(self):
		pass

	def Exit(self):
		pass

class SendGoal(State):
	def __init__(self,FSM):
		super(SendGoal,self).__init__(FSM)

	def Enter(self):
		print("Starting to sendGoal")
		super(SendGoal,self).Enter()
		# node.GetGoalPointsFromCsv()

	def Execute(self):
		print("Sending Goal..")
		t = threading.Thread(target=node.SendGoalCommand)
		t.start() 

	def Exit(self):
		print("Finished sendGoal")

class Pause(State):
	def __init__(self,FSM):
		super(Pause,self).__init__(FSM)

	def Enter(self):
		print("Starting to Pause")
		super(Pause,self).Enter()

	def Execute(self):
		print("Sending Pause...")
		t = threading.Thread(target = node.sendPauseCommand)
		t.start()
		

	def Exit(self):
		print("Finished Pause")

class Continue(State):
	def __init__(self,FSM):
		super(Continue,self).__init__(FSM)

	def Enter(self):
		print("Starting to Continue")
		super(Continue,self).Enter()

	def Execute(self):
		print("Sending Continue")
		t = threading.Thread(target = node.sendContinueCommand)
		t.start()

	def Exit(self):
		print("Finished Continue")

class Obstacle(State):
	def __init__(self,FSM):
		super(Obstacle,self).__init__(FSM)

	def Enter(self):
		print("Starting to Obstacle")
		super(Obstacle,self).Enter()

	def Execute(self):
		print("Sending Obstacle")
		node.robotStateManager(RobotCommand.COMMAND_OBSTACLE)

	def Exit(self):
		print("Finished Obstacle")

class ReachGoal(State):
	def __init__(self,FSM):
		super(ReachGoal,self).__init__(FSM)

	def Enter(self):
		print("Starting to ReachGoal")
		super(ReachGoal,self).Enter()

	def Execute(self):
		print("Sending ReachGoal")
		node.missionStatusManager(RobotCommand.COMMAND_REACHGOAL)
		node.robotStateManager(RobotCommand.COMMAND_REACHGOAL)

	def Exit(self):
		print("Finished ReachGoal")

class GoHome(State):
	def __init__(self,FSM):
		super(GoHome,self).__init__(FSM)

	def Enter(self):
		print("Starting to GoHome")
		super(GoHome,self).Enter()
		node.GetGoalPointsFromCsv()

	def Execute(self):
		print("Sending GoHome")
		t = threading.Thread(target=node.SendGoalHomeCommand)
		t.start() 

	def Exit(self):
		print("Finished GoHome")

class Alarm(State):
	def __init__(self,FSM):
		super(Alarm,self).__init__(FSM)

	def Enter(self):
		print("Starting to Alarm")
		super(Alarm,self).Enter()

	def Execute(self):
		print("Sending Alarm")
		t = threading.Thread(target=node.sendAlarmCommand)
		t.start()

	def Exit(self):
		print("Finished Alarm")

class AlarmReturnState(State):
	def __init__(self,FSM):
		super(AlarmReturnState,self).__init__(FSM)

	def Enter(self):
		print("Starting to AlarmReturnState")
		super(AlarmReturnState,self).Enter()

	def Execute(self):
		print("Sending AlarmReturnState")
		t = threading.Thread(target=node.sendAlarmReturnState)
		t.start()

	def Exit(self):
		print("Finished AlarmReturnState")

class StandBy(State):
	def __init__(self,FSM):
		super(StandBy,self).__init__(FSM)

	def Enter(self):
		print("Starting to StandBy")
		super(StandBy,self).Enter()

	def Execute(self):
		print("Sending StandBy")

	def Exit(self):
		print("Finished StandBy")

class Terminate(State):
	def __init__(self,FSM):
		super(Terminate,self).__init__(FSM)

	def Enter(self):
		print("Starting to Terminate")
		super(Terminate,self).Enter()

	def Execute(self):
		print("Sending Terminate")
		t = threading.Thread(target=node.sentTerminateCommand)
		t.start() 

	def Exit(self):
		print("Finished Terminate")

class FSM():
	def __init__(self,character):
		self.char = character
		self.states={}
		self.transitions={}
		self.curState=None
		self.prevState = None 
		self.trans=None

	def AddTransition(self,transName,transition):
		self.transitions[transName] = transition

	def AddState(self,stateName,state):
		self.states[stateName] = state

	def SetState(self,stateName):
		self.prevState = self.curState
		self.curState = self.states[stateName]

	def ToTransition(self,toTrans):
		self.trans = self.transitions[toTrans]

	def Execute(self):
		if(self.trans):
			self.curState.Exit()
			self.trans.Execute()
			self.SetState(self.trans.toState)
			self.curState.Enter()
			self.trans=None
		self.curState.Execute()


if __name__=='__main__':
	rclpy.init()
	node = TowtruckApi()
	app.include_router(node.router)
	api_thread = threading.Thread(target = run_api_server, args=[node.main_robot_ip,node.main_robot_port])
	api_thread.daemon = True  
	api_thread.start()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

