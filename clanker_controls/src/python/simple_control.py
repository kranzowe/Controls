#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32



class SimpleController(Node):

    #set default values
    control_freq = 30

    dist_i_gain = 0.25
    dist_p_gain = 0.3
    dist_d_gain = 0.0

    angle_i_gain = 0.0
    angle_p_gain = 0.5
    angle_d_gain = 0.0

    #setpoint
    setpoint = np.array([1.0, 0.0])

    # states are defined as [dist, angle]
    states = np.array([0.0, 0.0])
    stamps =  np.array([-1.0, -1.0])
    prev_states =  np.array([0.0, 0.0])
    prev_stamps =  np.array([-1.0, -1.0])

    #integral state
    integral = np.array([0.0, 0.0])
    previous_tic_stamp = -1.0

    def __init__(self):
        super().__init__("simple_controller")

        #create the publisher
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        #sub to the sensor data
        self.create_subscription(Float32, "/ld_distance", self.wall_dist_cb, 10)
        self.create_subscription(Twist, "/control_target", self.paper_data_cb, 10)

        #declare the ros2 parameters
        self.declare_parameter("control_freq", self.control_freq)
        self.declare_parameter("dist_i_gain", self.dist_i_gain)
        self.declare_parameter("dist_p_gain", self.dist_p_gain)
        self.declare_parameter("dist_d_gain", self.dist_d_gain)
        self.declare_parameter("angle_i_gain", self.angle_i_gain)
        self.declare_parameter("angle_p_gain", self.angle_p_gain)
        self.declare_parameter("angled_gain", self.angle_d_gain)

        self.control_timer = self.create_timer(1/self.control_freq, self.tic_control)

        #start the update parameters timer
        self.create_timer(1.0, self.update_params)

        
    def tic_control(self):
        
        #if there isn't enough data yet, do not attempt to control
        if(np.sum(np.where(self.prev_stamps < 0)) > 0):

            return
        
        #get the current time
        tic_time = self.get_ros_time_as_double()

        #take derivative of states
        state_error_derivatives = (self.states - self.prev_states) / (self.stamps - self.prev_stamps)

        #determine the state error
        state_error = (self.states - self.setpoint)

        #update the state integral 
        if(self.previous_tic_stamp > 0):
            self.integral += (tic_time - self.previous_tic_stamp) * state_error

            if(state_error[0] > .35):
                self.integral[0] = 0
            elif(state_error[0] < -.35):
                self.integral[0] = 0


            self.get_logger().info(f"Integral {self.integral}")


        #determine the speed output
        speed = self.dist_p_gain * state_error[0] + self.dist_i_gain * self.integral[0] + self.dist_d_gain * state_error_derivatives[0]

        anglular_rate = self.angle_p_gain * state_error[1] + self.angle_i_gain * self.integral[1] + self.angle_d_gain * state_error_derivatives[1]

        self.previous_tic_stamp = tic_time

        #send out the message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = anglular_rate

        self.command_pub.publish(msg)

    def get_ros_time_as_double(self):
        #return the ros2 time as float
        return self.get_clock().now().seconds_nanoseconds()[1] * 1e-9 + self.get_clock().now().seconds_nanoseconds()[0]

    def wall_dist_cb(self, msg):
        
        #move the current states to the previous
        self.prev_states = self.states
        self.prev_stamps = self.stamps

        #set the state from the message
        self.states = np.array([msg.data, 0.0])

        #stamp the data
        current_time = self.get_ros_time_as_double()
        self.stamps = np.array([current_time, current_time])

    def paper_data_cb(self, msg):
        
        #move the current states to the previous
        self.prev_states = self.states
        self.prev_stamps = self.stamps

        #set the state from the message
        self.states =np.array([msg.linear.x, msg.angular.z])

        #stamp the data
        current_time = self.get_ros_time_as_double()
        self.stamps = np.array([current_time, current_time])


    def update_params(self):

        prev_control_freq = self.control_freq

        #update the ros2 parameters
        self.control_freq = self.get_parameter("control_freq").value

        self.dist_i_gain = self.get_parameter("dist_i_gain").value
        self.dist_p_gain = self.get_parameter("dist_p_gain").value
        self.dist_d_gain = self.get_parameter("dist_d_gain").value

        self.angle_i_gain = self.get_parameter("angle_i_gain").value
        self.angle_p_gain = self.get_parameter("angle_i_gain").value
        self.angle_d_gain = self.get_parameter("angle_i_gain").value

        #make an necessary updates
        if not (self.control_freq == prev_control_freq):
            self.control_timer.cancel()
            self.control_timer = self.create_timer(1/self.control_freq, self.tic_control)

        
def main(args=None):
    rclpy.init(args=args)

    simple_controller = SimpleController()

    rclpy.spin(simple_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()