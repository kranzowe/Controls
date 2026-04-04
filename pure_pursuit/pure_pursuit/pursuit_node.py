import rclpy
from rclpy.node import Node
import numpy as np
import threading

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav2d_msgs.msg import Path2D
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

from pure_pursuit import PurePursuitParams, pure_pursuit

class PursuitNode(Node):

    def __init__(self):
        super().__init__('targeting')
        # Path subscription
        self.path_callback_group = ReentrantCallbackGroup()
        self.path_sub = self.create_subscription(
            Path2D,
            '/pathfinder',
            self.listener_callback,
            1,
            callback_group=self.path_callback_group)
        self.path_lock = threading.Lock()
        self.path_num = 0
        
        # State subscription
        self.state_sub = self.create_subscription(
            Pose2D,
            '/state_estimation',
            self.state_est_callback,
            10)
        self.current_pose = Pose2D()

        # Command publisher
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.params = PurePursuitParams(
            lookahead_distance=3.0,
            wheelbase=0.17,
            desired_vel=2.0)

    def state_est_callback(self, state_msg):
        self.current_pose = [
            state_msg.x, state_msg.y, state_msg.theta, state_msg.v
        ]

    def path_callback(self, path_msg):
        # Zero controls when new path is loaded.
        command = Twist()
        self.command_pub.publish(command)

        path_id = None
        with self.path_lock:
            self.path_num = (self.path_num + 1) % 100
            path_id = self.path_num
        
        poses = path_msg.poses
        waypoints = [[pose.x, pose.y, pose.theta] for pose in poses]
        self.get_logger().info(f"Received new path with {len(waypoints)}")
        wp_idx = 0
        while self.path_num == path_id:
            control_input, _, wp_idx_inc = pure_pursuit(waypoints[wp_idx:], self.current_pose, self.params)
            wp_idx += wp_idx_inc
            cmd_msg = Twist()
            cmd_msg.linear.x = control_input[0]
            cmd_msg.angular.z = control_input[1]
            self.command_pub.publish(cmd_msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    pursuer = PursuitNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(pursuer)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException) as e:
        pursuer.get_logger().error(e)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
