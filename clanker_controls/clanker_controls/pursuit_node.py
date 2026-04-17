#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import threading

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from nav_2d_msgs.msg import Path2D
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from ament_index_python import get_package_share_directory
import os
import csv
import yaml
from types import SimpleNamespace


from ament_index_python import get_package_share_directory

from clanker_controls.pure_pursuit import PurePursuitParams, pure_pursuit

OL_MODEL_SUBPATH = "resource/ol_data.yaml"

class PursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        # Path subscription
        self.path_callback_group = ReentrantCallbackGroup()
        self.path_sub = self.create_subscription(
            Path2D,
            '/pathfinder',
            self.path_callback,
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
        self.viz_pub = self.create_publisher(MarkerArray, "waypoint_markers", 10)

        # for waypoints from a csv
        self.loaded_waypoints = SimpleNamespace()
        self.loaded_waypoints.ready = False

        self.pp_params = None

        self.waypoints = []
        self.wp_idx = 0


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

        self.declare_parameter("visualization_on", True)

        #declare parameters to load waypoints
        self.declare_parameter("load_waypoints", True)
        load_waypoints = self.get_parameter("load_waypoints").value
        self.declare_parameter("waypoints_file", "Course11_small_path.csv")

        self.load_ol_model()

        if(load_waypoints):
            self.load_waypoint_csv()

        #callback to reload params
        self.create_timer(1.0, self.param_cb_timer)
        self.create_timer(0.03, self.control_cb)


    def load_waypoint_csv(self):

        #try to load the file from the path
        share_path = get_package_share_directory("clanker_controls")
        filename = self.get_parameter("waypoints_file").value

        waypoints_filepath = os.path.join(share_path, "waypoints", filename)

        self.loaded_waypoints.x_pos = []
        self.loaded_waypoints.y_pos = []
        self.loaded_waypoints.theta = []

        #read in the csv
        with open(waypoints_filepath, mode='r', newline='') as file:
            reader = csv.reader(file)
            for row in reader:

                #check to make sure there is enough data
                if(len(row) < 3):
                    self.get_logger().warn("Waypoint data is invalid...")
                    
                    continue

                self.loaded_waypoints.x_pos.append(float(row[0]))
                self.loaded_waypoints.y_pos.append(float(row[1]))
                self.loaded_waypoints.theta.append(float(row[2]))

        waypoints = []
        for idx in range(0, len(self.loaded_waypoints.x_pos)):
            waypoints.append(np.array([self.loaded_waypoints.x_pos[idx], self.loaded_waypoints.y_pos[idx], self.loaded_waypoints.theta[idx]]))


        if(self.get_parameter("visualization_on").value):

            color_msg = ColorRGBA()
            color_msg.r = 1.0
            color_msg.a = 1.0

            msg = MarkerArray()
            for point in waypoints:
                marker_msg = Marker()
                marker_msg.pose.position.x = point[0]
                marker_msg.pose.position.y = point[1]
                marker_msg.pose.position.z = 0.0

                marker_msg.pose.orientation.z = point[2]

                marker_msg.type = 0
                marker_msg.action = 0
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.header.frame_id = "map"
                marker_msg.color = color_msg
                
                msg.markers.append(marker_msg)

            self.viz_pub.publish(msg)

        self.waypoints = waypoints

        self.loaded_waypoints.ready = True
            
    def pose_callback(self, state_msg):
        self.current_state[0] = state_msg.x
        self.current_state[1] = state_msg.y
        self.current_state[2] = state_msg.theta

    def vel_callback(self, vel_msg):
        self.current_state[3] = vel_msg.linear.x

    def path_callback(self, path_msg):
        pass

        # # Zero controls when new path is loaded.
        # path_id = None
        # with self.path_lock:
        #     self.path_num = (self.path_num + 1) % 100
        #     path_id = self.path_num
        
        # poses = path_msg.poses
        # waypoints = [[pose.x, pose.y, pose.theta] for pose in poses]
        # self.get_logger().info(f"Received new path with {len(waypoints)}")
        # wp_idx = 0
        # while self.path_num == path_id:
        #     params = PurePursuitParams(
        #         lookahead_distance=self.get_parameter('lookahead_distance').value,
        #         wheelbase=self.get_parameter('wheelbase').value,
        #         desired_vel=self.get_parameter('desired_vel').value,
        #         pruning_distance=self.get_parameter('pruning_distance').value,
        #         Kp_v=self.get_parameter('Kp_v').value,
        #         Kp_theta=self.get_parameter('Kp_theta').value,
        #         ol_model=self.ol_model
        #     )
        #     control_input, _, wp_prune_idx = pure_pursuit(waypoints[wp_idx:], self.current_pose, params)
        #     wp_idx += wp_prune_idx
        #     cmd_msg = Twist()
        #     cmd_msg.linear.x = control_input[0]
        #     cmd_msg.angular.z = control_input[1]
        #     self.command_pub.publish(cmd_msg)
        #     rclpy.spin_once(self, timeout_sec=0.1)
    
    def load_ol_model(self):

        #get the robo rover share directory
        robo_share_dir = get_package_share_directory("robo_rover")
        ol_path = os.path.join(robo_share_dir, OL_MODEL_SUBPATH)

        with open(ol_path, "r") as file:
            try:
                model_raw = yaml.safe_load(file)

            except:
                self.get_logger().error("Failed to open ol model!")
                return
            
        self.ol_model = SimpleNamespace()
        self.ol_model.velocity = SimpleNamespace()
        self.ol_model.steering = SimpleNamespace()
        self.ol_model.time_constant = model_raw["time_constant"]
        self.ol_model.velocity.ol_velocities = np.array(model_raw["velocity"]["steady_state_velocity"])
        self.ol_model.velocity.pwms = np.array(model_raw["velocity"]["pwm_values"])
        self.ol_model.steering.ol_radius = np.array(model_raw["angular"]["turn_radius_values"])
        self.ol_model.steering.pwms = np.array(model_raw["angular"]["pwm_values"])
        self.ol_model.steering.zero_idx = np.argmax(np.abs(self.ol_model.steering.ol_radius))

        self.ol_model_loaded = True

    def param_cb_timer(self):

        self.pp_params = PurePursuitParams(
            lookahead_distance=self.get_parameter('lookahead_distance').value,
            wheelbase=self.get_parameter('wheelbase').value,
            desired_vel=self.get_parameter('desired_vel').value,
            pruning_distance=self.get_parameter('pruning_distance').value,
            Kp_v=self.get_parameter('Kp_v').value,
            Kp_theta=self.get_parameter('Kp_theta').value,
            ol_model=self.ol_model
        )

    def control_cb(self):

        self.get_logger().info(f"{self.pp_params}")

        if(self.loaded_waypoints.ready == True and (not self.pp_params is None)):

            self.get_logger().info("hello")

            control_input, _, wp_prune_idx = pure_pursuit(self.waypoints[self.wp_idx:], self.current_state, self.pp_params, logger=self.get_logger())
            self.wp_idx += wp_prune_idx
            cmd_msg = Twist()
            cmd_msg.linear.x = control_input[0]
            cmd_msg.angular.z = control_input[1]
            self.command_pub.publish(cmd_msg)




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
