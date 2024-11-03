from rclpy.node import Node
import rclpy
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitailPoseSet(Node):
    def __init__(self):
        super().__init__("initate_default_pose")
        while ("set_initial_pose" not in self.get_node_names()):
            pass
        self.pub = self.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)
        self.initial_pose = [45.9,-216.3,1.57] #[x,y,heading angle in rad]
        self.pose_msg = PoseWithCovarianceStamped()
        #set msg
        self.pose_msg.pose.pose.position.x = self.initial_pose[0]
        self.pose_msg.pose.pose.position.y = self.initial_pose[1]
        quatrion = quaternion_from_euler(0,0,self.initial_pose[2])
        self.pose_msg.pose.pose.orientation.x = quatrion[0]
        self.pose_msg.pose.pose.orientation.y = quatrion[1]
        self.pose_msg.pose.pose.orientation.z = quatrion[2]
        self.pose_msg.pose.pose.orientation.w = quatrion[3]
    def pub_new(self):
        self.pub.publish(self.pose_msg)

def main(args = None):
    rclpy.init(args=args)
    nodee = InitailPoseSet()
    nodee.pub_new() 
    nodee.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__": 
    main()