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
        
        # Pose subscription
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/current_pose',
            self.pose_callback,
            10)
         # Velocity subscription
        self.vel_sub = self.create_subscription(
            Pose2D,
            '/ol_rates',
            self.vel_callback,
            10)
        self.current_state = np.zeros(4)  # x, y, theta, vel

        # Command publisher
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # How far ahead on the path to target (m)
        self.declare_parameter("lookahead_distance", 1.0)
        # Range near vehicle (m) to designate waypoints as "visited" and remove from consideration
        # Ideally the distance between lookahead_distance and pruning distance should be greater
        # than the distance between waypoints to prevent snapping.
        self.declare_parameter("pruning_distance", 0.2)
        # Velocity to target (m/s).
        self.declare_parameter("desired_vel", 1.0)
        # Vehicle property for steering mechanics (m)
        self.declare_parameter("wheelbase", 0.171)
        # Proportional velocity gain
        self.declare_parameter("Kp_v", 1.0)
        # Proportional steering gain
        self.declare_parameter("Kp_theta", 1.0)

    def pose_callback(self, state_msg):
        self.current_state[0] = state_msg.x
        self.current_state[1] = state_msg.y
        self.current_state[2] = state_msg.theta

    def vel_callback(self, vel_msg):
        self.current_state[3] = vel_msg.linear.x

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
            params = PurePursuitParams(
                lookahead_distance=self.get_parameter('lookahead_distance').value,
                wheelbase=self.get_parameter('wheelbase').value,
                desired_vel=self.get_parameter('desired_vel').value,
                pruning_distance=self.get_parameter('pruning_distance').value
            )
            control_input, _, wp_prune_idx = pure_pursuit(waypoints[wp_idx:], self.current_pose, params)
            wp_idx += wp_prune_idx
            cmd_msg = Twist()
            cmd_msg.linear.x = control_input[0] * self.get_parameter('Kp_v').value
            cmd_msg.angular.z = control_input[1] * self.get_parameter('Kp_theta').value
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
