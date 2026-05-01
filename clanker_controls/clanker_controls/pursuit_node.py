#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import threading

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from rclpy.duration import Duration
from nav_2d_msgs.msg import Path2D
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from ament_index_python import get_package_share_directory
import os
import csv
import math
import yaml
from types import SimpleNamespace
import control

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import TransformStamped


from ament_index_python import get_package_share_directory

from clanker_controls.pure_pursuit import PurePursuitParams, pure_pursuit

OL_MODEL_SUBPATH = "resource/ol_data.yaml"

TF_TIMEOUT = 0.1

class PursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        # Path subscription
        self.path_callback_group = ReentrantCallbackGroup()
        self.path_lock = threading.Lock()
        self.path_num = 0
        
        # Pose subscription
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
         # Velocity subscription
        self.vel_sub = self.create_subscription(
            Twist,
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # How far ahead on the path to target (m)
        self.declare_parameter("lookahead_distance", 1.0)
        # Range near vehicle (m) to designate waypoints as "visited" and remove from consideration
        # Ideally the distance between lookahead_distance and pruning distance should be greater
        # than the distance between waypoints to prevent snapping.
        self.declare_parameter("pruning_distance", 0.2)
        # Velocity to target (m/s).
        self.declare_parameter("desired_vel", 0.5)
        # Vehicle property for steering mechanics (m)
        self.declare_parameter("wheelbase", 0.171)
        # Proportional velocity gain
        self.declare_parameter("Kp_v", 0.0)
        # Proportional steering gain
        self.declare_parameter("Kp_theta", 0.0)

        self.declare_parameter("visualization_on", True)

        #declare parameters to load waypoints
        self.declare_parameter("load_waypoints", True)
        load_waypoints = self.get_parameter("load_waypoints").value
        self.declare_parameter("waypoints_file", "ohmy_big_path.csv")

        self.declare_parameter("debug", False)
        self.debug_prints = self.get_parameter("debug").value

        self.ol_model = control.load_ol_model(self.get_logger())
        self.ol_model_loaded = (self.ol_model is not None)

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
            for idx, point in enumerate(waypoints):
                marker_msg = Marker()
                marker_msg.pose.position.x = point[0]
                marker_msg.pose.position.y = point[1]
                marker_msg.pose.position.z = 0.0

                marker_msg.pose.orientation.z = point[2]

                marker_msg.scale.x = 0.1
                marker_msg.scale.y = 0.1
                marker_msg.scale.z = 0.2

                marker_msg.type = 0
                marker_msg.action = 0
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.header.frame_id = "map"
                marker_msg.id = idx

                marker_msg.color = color_msg
                
                msg.markers.append(marker_msg)

            self.viz_pub.publish(msg)

        self.waypoints = waypoints

        self.loaded_waypoints.ready = True
            
    def pose_callback(self, state_msg):

        self.get_logger().warn("Interesting this is getting called, the pose should be broadcast over tf2, not published....")

        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = self.get_yaw_from_quat(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

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

        self.debug_prints = self.get_parameter("debug").value

    def control_cb(self):


        if(self.loaded_waypoints.ready == True and (not self.pp_params is None)):

            #must first determine the current state of the rover from tf2

            #get a transform 
            # transform : TransformStamped = self.lookup_tf_transform("map", "base_link")

            #ensure a valid transform
            # if(transform is None):
            #     return
            
            #convert to the yaw angle
            # yaw = self.get_yaw_from_quat(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

            #i believe this is the expected format
            #you'll need to take a look at how I'm setting the velocity here... the topic which vinnie subs to (to get velocity) only is published in the pwm control mode of the rover, however that is only published in pwm mode...
            #however slam def works better out of pwm mode... no good reason why but it does... so don;t change that
            #may need to get creative here idk?
            # state = [transform.transform.translation.x, transform.transform.translation.y, yaw, self.current_state[3]]
            state = self.current_state

            if(self.debug_prints):
                self.get_logger().info(f"Current state x: {state[0]}, y: {state[1]}, yaw: {state[2]}. Next waypoint x: {self.waypoints[self.wp_idx][0]}, y: {self.waypoints[self.wp_idx][1]}. Waypoint num {self.wp_idx}")
            
            #this is all og vinnie pure pursuit :) -> I think he tested this bit in the sim but very unsure
            control_input, _, wp_prune_idx = pure_pursuit(self.waypoints[self.wp_idx:], state, self.pp_params, logger=self.get_logger())
            self.wp_idx += wp_prune_idx
            cmd_msg = Twist()
            cmd_msg.linear.x = float(control_input[0])
            cmd_msg.angular.z = float(control_input[1])
            self.command_pub.publish(cmd_msg)

    def lookup_tf_transform(self, target_frame, source_frame):
        try:
            now = rclpy.time.Time()

            #lookup the transform in the buffer
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, now, timeout=Duration(seconds=TF_TIMEOUT, nanoseconds=0))

            return transform

        except TransformException as ex:

            self.get_logger().warn(f"Cannot transform from {target_frame} to {source_frame} as the transform has not be published")

        return None
    
    def get_yaw_from_quat(self, qx, qy, qz, qw):

        #did it this way as to avoid having to install and additional package on the pi... hopefully doesn't bite 

        return math.atan2(2 * (qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        

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
