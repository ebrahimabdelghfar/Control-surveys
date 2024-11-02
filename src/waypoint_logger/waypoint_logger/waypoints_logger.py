#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from os.path import expanduser
from time import gmtime, strftime
import numpy as np
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.animation as animation


class WayPointLogger(Node):
	def __init__(self):
		super().__init__("Waypoint_logger_node")
		self.create_subscription(Odometry,"/carla/ego_vehicle/odometry",self.save_waypoint,1)
		self.ctr = 0 
		self.euler = None
		self.file_name='wp_file_stanly.csv'
		self.file = open(self.file_name,'w')
		print("saving to file pure_pursuit/waypoints with a name - ", self.file_name)
		
	def save_waypoint(self,data:Odometry):
		quaternion = np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		self.euler = euler_from_quaternion(quaternion)
		speed = LA.norm(np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z]),2)
			
		if data.twist.twist.linear.x > 0.:
			pass

		if self.ctr == 1:
			self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x, data.pose.pose.position.y, self.euler[2], speed))
			print(data.pose.pose.position.x, data.pose.pose.position.y)
			self.ctr = 0 
		else: 
			self.ctr +=1 
	
	def shutdown(self):
		self.file.close()
		print('Goodbye')

def main(args=None):
	rclpy.init(args=args)
	waypoint_logger = WayPointLogger()
	rclpy.spin(waypoint_logger)
	waypoint_logger.shutdown()
	waypoint_logger.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()