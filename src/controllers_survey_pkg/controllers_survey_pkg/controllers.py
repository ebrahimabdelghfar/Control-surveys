#!/usr/bin/env python3
import os
import time
import math
import cvxpy as cp
import carla.libcarla
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from matplotlib import patches
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32
from controllers_survey_pkg.mpc_pkg.mpc_setup import initialize_mpc_problem
from controllers_survey_pkg.controllers_param.mpc_param import MPCParams
from controllers_survey_pkg.controllers_param.general_param import GeneralParam
from controllers_survey_pkg.controllers_param.pure_pursuit_param import PursuitParam
from controllers_survey_pkg.controllers_param.stanely_param import StanelyParam

class Controller(Node):
	def __init__(self,show_animation_flag:bool,controller_type = "mpc") -> None:
		"""
		Controller class to control the vehicle using Pure Pursuit, Stanley Control and MPC
		:param show_animation_flag: Show the animation of the vehicle
		:param controller_type: Type of controller to use (mpc , pure_pursuit , stanley_control)
		"""
		super().__init__("control_node")
		self.car_control_msg = CarlaEgoVehicleControl()
		self.command_publisher = self.create_publisher(CarlaEgoVehicleControl,"/carla/ego_vehicle/vehicle_control_cmd",1)
		self.create_subscription(Odometry,"/carla/ego_vehicle/odometry",self.pose_callback,1)
		self.create_subscription(Float32,"/carla/ego_vehicle/speedometer",self.velocitySensor,1)
		#general Parameters
		self.xc = 0.0
		self.yc = 0.0
		self.yaw = 0.0
		self.vel = 0.0
		self.idx = 0
		self.v = 0.0 
		self.WB = GeneralParam().wheelbase
		self.dt = GeneralParam().DT  # time tick[s]
		self.waypoints = self.read_points()
		# Stanley Control Parameters
		self.__intial_pose_recived= False

		if(controller_type == "pure_pursuit"):
			self.create_timer(self.dt , self.pure_pursuit)
			self.__intial_pose_recived = False

		elif(controller_type == "stanley_control"):
			self.create_timer(self.dt , self.stanley_control)

		elif(controller_type == "mpc"):
			self.__mpc_built = False
			self.__mpc_initialized = False 
			self.acc_cmd, self.delta_cmd, self.velocity_cmd = 0.0, 0.0, 0.0
			self.zk = np.zeros((MPCParams().NO_STATES, 1))  # 
			self.uk = np.array([[self.acc_cmd, self.delta_cmd]]).T
			self.u_prev = np.zeros((MPCParams().NO_INPUTS, 1))
			self.xref = np.zeros((MPCParams().HORIZON + 1, MPCParams().NO_STATES))
			self.uref = np.zeros((MPCParams().HORIZON, MPCParams().NO_INPUTS))
			self.setupMpc()
			self.create_timer(self.dt,self.mpc_control)
			
		self.show_animation = show_animation_flag

	def getTheNexWayPoints(self,nearest_point_indx:int,waypoints:np.array,horizon:int):
		if nearest_point_indx >= len(waypoints)-horizon:
			concatenated_array = np.concatenate((waypoints[nearest_point_indx:],np.tile(waypoints[-1],(horizon-len(waypoints)+nearest_point_indx,1))),axis=0)
			for i in range(horizon):
				concatenated_array[i][-1] = self.calculateTargetYaw(nearest_point_indx+i,concatenated_array[i][0],concatenated_array[i][1]) 
			return concatenated_array
		else:
			arr = waypoints[nearest_point_indx:nearest_point_indx+horizon]
			for i in range(horizon):
				arr[i][-1] = self.calculateTargetYaw(nearest_point_indx+i,arr[i][0],arr[i][1])
			return arr

	def normalize(self, value,max_range,min_range):
		"""
		Normalize the value between the max and min range
		"""
		return (value - min_range) / (max_range - min_range)

	def pi2pi(self,angle):
		"""
		Normalize angle between [-pi, pi]
		"""
		return (angle + np.pi) % (2 * np.pi) - np.pi

	def velocitySensor(self,data:Float32):
		self.v = data.data

	def pose_callback(self,data:Odometry):
		"""
		Get current state of the vehicle
		"""
		self.xc = data.pose.pose.position.x
		self.yc = data.pose.pose.position.y
		
		# Convert Quaternions to Eulers
		qx = data.pose.pose.orientation.x
		qy = data.pose.pose.orientation.y
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		quaternion = (qx,qy,qz,qw)
		euler = euler_from_quaternion(quaternion)
		self.yaw = euler[2]
	
		if not self.__intial_pose_recived:
			self.__intial_pose_recived = True

	def read_points(self):
		"""
		CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
		"""
		file_name = 'wp_file.csv' #'racecar_walker.csv'
		file_path = file_name
		with open(file_path) as f:
			path_points = np.loadtxt(file_path, delimiter = ',')
		return path_points

	def find_distance(self,x1, y1):
		distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
		return distance

	def find_distance_index_based(self,idx):
		if idx >= len(self.waypoints):
			idx = len(self.waypoints) - 1
		x1 = float(self.waypoints[idx][0])
		y1 = float(self.waypoints[idx][1])
		distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
		return distance

	def find_nearest_waypoint(self):
		"""
		Get closest idx to the vehicle
		"""
		curr_xy = np.array([self.xc, self.yc])
		waypoints_xy = self.waypoints[:, :2]
		nearest_idx = np.argmin(np.sum((curr_xy - waypoints_xy)**2, axis=1))
		return nearest_idx - 1

	def idx_close_to_lookahead(self,idx,lookahead = 8.5):
		"""
		Get closest index to lookahead that is greater than the lookahead
		"""
		while self.find_distance_index_based(idx) < lookahead:
			idx += 1 
			if idx >= len(self.waypoints):
				break
		return idx - 1 

	def plot_arrow(self,x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
		"""
		Plot arrow
		"""
		if not isinstance(x, float):
			for ix, iy, iyaw in zip(x, y, yaw):
				self.plot_arrow(ix, iy, iyaw)
		else:
			plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc, ec=ec, head_width=width, head_length=width)
			plt.plot(x, y)
			patches.Rectangle((self.xc,self.yc), 0.35,0.2)

	def pure_pursuit(self):
		try:
			cx = self.waypoints[:, 0]; cy = self.waypoints[:, 1]
			nearest_idx = self.find_nearest_waypoint()
			idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx,PursuitParam().LOOKAHEAD_Pursuit) 
			target_x = float(self.waypoints[idx_near_lookahead][0])
			target_y = float(self.waypoints[idx_near_lookahead][1])

			"""
			PURE PURSUIT CONTROLLER
			"""
			# calculate alpha (angle between the goal point and the path point)
			x_delta = target_x - self.xc
			y_delta = target_y - self.yc
			alpha = np.arctan2(y_delta , x_delta) - self.yaw

			# front of the vehicle is 0 degrees right +90 and left -90 hence we need to convert our alpha
			if alpha > np.pi / 2:
				alpha -= np.pi
			if alpha < -np.pi / 2:
				alpha += np.pi

			# Set the lookahead distance depending on the speed
			lookahead = self.find_distance(target_x, target_y)
			steering_angle = np.arctan2((2 * self.WB * np.sin(alpha)) , lookahead)

			# Set max wheel turning angle
			if steering_angle > 1.22:
				steering_angle = 1.22
			elif steering_angle < - 1.22:
				steering_angle = -1.22

			# Publish messages
			self.car_control_msg.steer = - (steering_angle / 1.22)
			self.car_control_msg.throttle = 0.4
			self.command_publisher.publish(self.car_control_msg)
			# Plot map progression
			if self.show_animation:
				plt.cla()
				# For stopping simulation with the esc key.
				plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
				self.plot_arrow(float(self.xc), float(self.yc), float(self.yaw))
				plt.plot(cx, cy, "-r", label = "course")
				plt.plot(self.xc, self.yc, "-b", label = "trajectory")
				plt.plot(target_x, target_y, "xg", label = "target")
				plt.axis("equal")
				plt.grid(True)
				plt.title("Pure Pursuit Control" + str(1))
				plt.pause(0.001)

		except IndexError:
			print("PURE PURSUIT COMPLETE --> COMPLETED ALL WAYPOINTS")

	def calculate_cross_track_error(self, target_x, target_y):
		# Calculate the cross-track error as the perpendicular distance to the path
		dx = target_x - self.xc
		dy = target_y - self.yc
		cross_track_error = np.hypot(dx, dy)
		
		# Determine the sign of the cross-track error
		path_angle = np.arctan2(dy, dx)
		angle_diff = path_angle - self.yaw
		cross_track_error *= np.sign(np.sin(angle_diff))  # Positive if error is to the left, negative if to the right

		return cross_track_error

	def calculateTargetYaw(self,nearest_idx,target_x,target_y):
		#getting the slope of the path
		if nearest_idx < len(self.waypoints) - 1:
			next_target_x, next_target_y = self.waypoints[nearest_idx + 1][:2]
		else:
			next_target_x, next_target_y = target_x, target_y
		return np.arctan2(next_target_y - target_y, next_target_x - target_x)

	def stanley_control(self):
		cx = self.waypoints[:, 0]; cy = self.waypoints[:, 1]
		nearest_idx = self.find_nearest_waypoint()
		target_x = float(self.waypoints[nearest_idx][0])
		target_y = float(self.waypoints[nearest_idx][1])

		cross_track_error = self.calculate_cross_track_error(target_x,target_y)
		path_heading = self.calculateTargetYaw(nearest_idx,target_x,target_y) 
		
		# Heading error
		heading_error = path_heading - self.yaw
		heading_error = self.pi2pi(heading_error)  # Normalize to [-pi, pi]

		# Stanley steering control
		steering_angle = heading_error + np.arctan2(StanelyParam().k * cross_track_error , (StanelyParam().k_soft + self.v))
		
		# Constrain steering angle
		steering_angle = max(min(steering_angle, 1.22), -1.22)

		# Publish messages
		self.car_control_msg.steer = - (steering_angle / 1.22)
		self.car_control_msg.throttle = 0.4

		self.command_publisher.publish(self.car_control_msg)

		print("Steering Angle: ", self.car_control_msg.steer)
		print("velocity:", self.car_control_msg.throttle)
		
		if self.show_animation:
			plt.cla()
			# For stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
			self.plot_arrow(float(self.xc), float(self.yc), float(self.yaw))
			plt.plot(cx, cy, "-r", label = "course")
			plt.plot(self.xc, self.yc, "-b", label = "trajectory")
			plt.plot(target_x, target_y, "xg", label = "target")
			plt.axis("equal")
			plt.grid(True)
			plt.title("Pure Pursuit Control" + str(1))
			plt.pause(0.001)

	def setupMpc(self):
		"""
		Setup the Model Predictive
		"""
		self.MPCController = initialize_mpc_problem(   horizon = MPCParams().HORIZON,
                                                       sample_time = self.dt,
                                                       Q= MPCParams().Q , R = MPCParams().R,
                                                       Qf= MPCParams().Qf , Rd = MPCParams().Rd,
                                                       wheelbase=self.WB,
                                                       delta_min = - GeneralParam().MAX_STEER,
                                                       delta_max = GeneralParam().MAX_STEER,
                                                       vel_min = GeneralParam().MIN_SPEED,
                                                       vel_max = GeneralParam().MAX_SPEED,
                                                       acc_min = - GeneralParam().MAX_ACCEL,
                                                       acc_max = GeneralParam().MAX_ACCEL,
                                                       suppress_ipopt_output=True, model_type=MPCParams().model_type)
		
		self.__mpc_built = True

	def initialize_mpc_solver(self):
		# warmstart
		self.MPCController.mpc.x0 = self.zk
		self.MPCController.mpc.set_initial_guess()
		self.__mpc_initialized = True
		self.get_logger().info("######################## MPC initialized ##################################")

	def mpc_control(self):
		"""
		Model Predictive Control
		"""
		nearest_index = 0
		if self.__mpc_built and not self.__mpc_initialized and self.__intial_pose_recived:
			# initialize the MPC solver with the current state if it is already built
			num_tries = 5  # 1, 5, 10
			for _ in range(num_tries):
				# a kind of hacky way to fix not finding a solution the first time the solver is called
				self.initialize_mpc_solver()

		if self.__mpc_initialized:
			x = self.xc  
			y = self.yc
			vel = self.v
			psi = self.pi2pi(self.yaw)
			nearest_index = self.find_nearest_waypoint()
			nearest_index_local = self.idx_close_to_lookahead(nearest_index,MPCParams().LOOKAHEAD_DISTANCE_MPC)

			self.xref = self.getTheNexWayPoints(nearest_index_local,self.waypoints,MPCParams().HORIZON + 1)

			if self.__intial_pose_recived:
				self.zk[0, 0] = x
				self.zk[1, 0] = y
				self.zk[2, 0] = vel
				self.zk[3, 0] = psi

				self.u_prev = self.uk.copy()
				self.MPCController.reference_states[:, :] = self.xref
				u = self.MPCController.get_control(self.zk)
				self.acc_cmd = u[0, 0]
				self.delta_cmd = u[1, 0]

				self.car_control_msg.steer =  - self.delta_cmd / 1.22
				self.car_control_msg.throttle = 0.4
				self.command_publisher.publish(self.car_control_msg)
				
def main(args=None):
   rclpy.init(args=args)
   main_code  = Controller(show_animation_flag = False,controller_type=GeneralParam().controller)
   rclpy.spin(main_code)
   main_code.destroy_node() 
   rclpy.shutdown()

if __name__ == "__main__":
	main()