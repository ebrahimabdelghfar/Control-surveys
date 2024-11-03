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

class Controller(Node):
	def __init__(self,show_animation_flag:bool) -> None:
		super().__init__("pure_pursuit")
		self.car_control_msg = CarlaEgoVehicleControl()
		self.command_publisher = self.create_publisher(CarlaEgoVehicleControl,"/carla/ego_vehicle/vehicle_control_cmd",1)
		self.create_subscription(Odometry,"/carla/ego_vehicle/odometry",self.pose_callback,1)
		self.create_subscription(Float32,"/carla/ego_vehicle/speedometer",self.velocitySensor,1)
		self.create_timer(0.05,self.mpc_control)
		self.xc = 0.0
		self.yc = 0.0
		self.yaw = 0.0
		self.vel = 0.0
		self.idx = 0
		self.v = 0.0 
		self.waypoints = []
		self.v_prev_error = 0.0
		self.freqs = 2
		self.LOOKAHEAD = 8.5 # 1.5
		self.WB = 4.5
		self.pure_pursuit_flag = True
		self.show_animation = show_animation_flag
		self.k = 3.5  # Stanley control gain
		self.k_soft = 50.0
		# Initialize vehicle state and parameters
		self.N = 10  # Prediction horizon
		self.dt = 0.1  # Timestep for predictions
		self.Q = np.diag([1.0, 1.0, 0.5, 0.1])  # State cost weights
		self.R = np.diag([0.01, 0.01])  # Control cost weights
	
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

	def idx_close_to_lookahead(self,idx):
		"""
		Get closest index to lookahead that is greater than the lookahead
		"""
		while self.find_distance_index_based(idx) < self.LOOKAHEAD:
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
			self.waypoints = self.read_points()

			cx = self.waypoints[:, 0]; cy = self.waypoints[:, 1]

			nearest_idx = self.find_nearest_waypoint()
			idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx) 
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

			print("Steering Angle: ", steering_angle)
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

	def stanley_control(self):

		self.waypoints = self.read_points()
		cx = self.waypoints[:, 0]; cy = self.waypoints[:, 1]

		nearest_idx = self.find_nearest_waypoint()
		target_x = float(self.waypoints[nearest_idx][0])
		target_y = float(self.waypoints[nearest_idx][1])

		# Cross-track error
		dx = target_x - self.xc
		dy = target_y - self.yc
		cross_track_error = self.calculate_cross_track_error(target_x,target_y)

		if nearest_idx < len(self.waypoints) - 1:
			next_target_x, next_target_y = self.waypoints[nearest_idx + 1][:2]
		else:
			next_target_x, next_target_y = target_x, target_y
        
		path_heading = np.arctan2(next_target_y - target_y, next_target_x - target_x)
		
		# Heading error
		heading_error = path_heading - self.yaw
		heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))  # Normalize to [-pi, pi]

		# Stanley steering control
		steering_angle = heading_error + np.arctan2(self.k * cross_track_error , (self.k_soft + self.v))
		
		# Constrain steering angle
		steering_angle = max(min(steering_angle, 1.22), -1.22)

		# Publish messages
		self.car_control_msg.steer = - (steering_angle / 1.22)
		self.car_control_msg.throttle = 0.4

		self.command_publisher.publish(self.car_control_msg)

		print("Steering Angle: ", steering_angle)
		
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

	def mpc_control(self):
		# Predict future states using MPC
		# Define optimization variables
		x = cp.Variable((4, self.N + 1))
		u = cp.Variable((2, self.N))
	
		# Define constraints
		constraints = []
		constraints += [x[:, 0] == np.array([self.xc, self.yc, self.yaw, self.v])]
		constraints += [u[0, :] <= 1.22]
		constraints += [u[0, :] >= -1.22]
		constraints += [u[1, :] <= 0.4]
		constraints += [u[1, :] >= -0.4]

		# Define cost function
		cost = 0
		for t in range(self.N):
			cost += cp.quad_form(x[:, t] - np.array([self.waypoints[self.idx][0], self.waypoints[self.idx][1], 0, 0]), self.Q)
			cost += cp.quad_form(u[:, t], self.R)
			constraints += [x[:, t + 1] == self.dynamics(x[:, t], u[:, t])]
			self.idx += 1

		# Define optimization problem
		problem = cp.Problem(cp.Minimize(cost), constraints)

		# Solve optimization problem
		problem.solve()

		# Extract first control input
		steering_angle = u[0, 0].value
		throttle = u[1, 0].value

		# Publish messages
		self.car_control_msg.steer = steering_angle
		self.car_control_msg.throttle = throttle
		self.command_publisher.publish(self.car_control_msg)

	
def main(args=None):
   rclpy.init(args=args)
   main_code  = Controller(show_animation_flag = True)
   rclpy.spin(main_code)
   main_code.destroy_node() 
   rclpy.shutdown()

if __name__ == "__main__":
	main()