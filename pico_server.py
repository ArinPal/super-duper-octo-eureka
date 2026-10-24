#!/usr/bin/env python3

# This python file runs a ROS 2-node of name waypoint_server which implements an action server to navigate the Swift Pico Drone to the given waypoints.
# You can use either PID or LQR controller to navigate the drone to the given waypoints.


import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# import the messages for action feedback and result
from swift_pico_interfaces.action import NavToWaypoint_Feedback, NavToWaypoint_Result

#import the action
from swift_pico_interfaces.action import NavToWaypoint

#pico control specific libraries
from swift_msgs.msg import SwiftMsgs
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error
from controller_msg.msg import PIDTune
from nav_msgs.msg import Odometry


class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_or_lqr_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.odometry_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.yaw = 0.0              # current yaw in degrees (updated from whycode)
        self.desired_yaw = -0.5     # target yaw angle (degrees)
        self.current_state = [0.0, 0.0, 0.0]
        self.desired_state = [0.0, 0.0, 0.0]
        self.drone_position = [0.0, 0.0, 0.0]
        self.xyz = [0.0, 0.0, 0.0, 0.0]
        self.dtime = 0

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1400

        #Initiate or declare other variables here depending upon whether you are implementing PID or LQR controller.
        # PID controller state variables
        self.current_state = [0.0, 0.0, 0.0]          # [x, y, z] from whycode
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.error_sum_limit = 5000.0               #change limit after testing
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]
        self.pos_error = Error()
        



        self.sample_time = 0.033 #put the appropriate value according to your controller

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        self.create_subscription(MarkerArray, '/whycode_node/markers', self.whycode_callback, 1)
        #Add other subscribers here
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10, callback_group=self.odometry_callback_group)

        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        self.action_server = ActionServer(self, NavToWaypoint, 'waypoint_navigation', self.execute_callback, callback_group=self.action_callback_group)

        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts

        
        self.publish_heartbeat()
        self.create_timer(0.1, self.publish_heartbeat)

        time.sleep(0.5)  # let the heartbeat reach the rosbag before arming
        self.arm()

        #define the function to be run inside the timer callback. This function will implement the PID or LQR algorithm
        
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_or_lqr_callback_group)

    def pid(self):
   
    
        #1
        error = [ self.current_state[0] - self.desired_state[0],   #Roll
                  self.current_state[1] - self.desired_state[1],   #Pitch	
                  self.current_state[2] - self.desired_state[2] ]  #Throttle
   
   
        #3
        PID_output = [ self.Kp[i] * error[i] + self.Ki[i] * self.error_sum[i] + (self.Kd[i] * (error[i] - self.prev_error[i])) for i in range(3) ]  #Roll, Pitch, Throttle	
   
        #4
        self.cmd.rc_roll = int(1500 - PID_output[0])
        self.cmd.rc_pitch = int(1500 - PID_output[1])
        self.cmd.rc_throttle = int(1500 + PID_output[2])
        
        #6
        self.cmd.rc_roll = int(max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll)))
        self.cmd.rc_pitch = int(max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch)))
        self.cmd.rc_throttle = int(max(self.min_values[2],min(self.max_values[2], self.cmd.rc_throttle)))
   
        #7 AND 8
        for i in range(3):
            self.prev_error[i] = error[i]
            self.error_sum[i] += error[i]*self.sample_time
            self.error_sum[i] = max(-self.error_sum_limit, min(self.error_sum_limit, self.error_sum[i]))
   
   
   
    #------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        # calculate throttle error, pitch error and roll error, then publish it accordingly
        self.pos_error.roll_error = error[0]
        self.pos_error.pitch_error = error[1]
        self.pos_error.throttle_error = error[2]
        self.pos_error_pub.publish(self.pos_error)
   


    def publish_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'pico_controller_active'
        self.heartbeat_pub.publish(hb)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)


    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)


    # Whycode callback function
    # The function gets executed each time whycode_node publishes its markers.
    def whycode_callback(self, msg):

        if not msg.markers:
            self.get_logger().warn("No markers received")
            return

        marker = msg.markers[0]
        position = marker.position.position
        self.current_state = [position.x, position.y, position.z]
        self.drone_position = self.current_state[:]

        # the yaw controller works in degrees, so convert here.
        self.yaw = math.degrees(marker.rotation.z)

        self.dtime = msg.header.stamp.sec

    # If you are using PID controller, then define callback function like altitide_set_pid to tune pitch, roll.
    #If you are using LQR controller, then define a functions which were given in the boiler plate code of task 1c and the ones which you have defined on yur own.

    # Callback function for /throttle_pid
    # This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
    def altitude_set_pid(self, alt):
            self.Kp[2] = alt.kp   # This is just for an example. You can change the ratio/fraction value accordingly
            self.Ki[2] = alt.ki 
            self.Kd[2] = alt.kd 
    
    def pitch_set_pid(self, pitch): 
            self.Kp[1] = pitch.kp 
            self.Ki[1] = pitch.ki 
            self.Kd[1] = pitch.kd 
    
    
    def roll_set_pid(self, roll):
            self.Kp[0] = roll.kp 
            self.Ki[0] = roll.ki 
            self.Kd[0] = roll.kd 
            

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)

   


    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.desired_state[0] = goal_handle.request.waypoint.position.x
        self.desired_state[1] = goal_handle.request.waypoint.position.y
        self.desired_state[2] = goal_handle.request.waypoint.position.z

        self.get_logger().info(f'New Waypoint Set: {self.desired_state}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        self.feedback_msg = NavToWaypoint_Feedback()
        
        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.

        while True:
			#current whycon poses
            self.feedback_msg.current_waypoint.pose.position.x = self.current_state[0]
            self.feedback_msg.current_waypoint.pose.position.y = self.current_state[1]
            self.feedback_msg.current_waypoint.pose.position.z = self.current_state[2]
            self.feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(self.feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.08) 

            # The value 0.08 is in meters used for as error range in Whycon coordinates for LQR controller. If you are using PID controller, it will become 0.8 as PID Whycon coordinates are in decimeters.

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        self.get_logger().info('Drone in sphere for 1st time')                        #you can choose to comment this out to get a better look at other logs

            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.get_logger().info('Drone in sphere')                                     #you can choose to comment this out to get a better look at other logs
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.get_logger().info('Drone out of sphere')                                 #you can choose to comment this out to get a better look at other logs
                        self.point_in_sphere_start_time = None

            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3:
                 break
                        

        goal_handle.succeed()

        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        self.result = NavToWaypoint_Result()

        self.result.hov_time = self.dtime - self.duration #this is the total time taken by the drone in trying to stabilize at a point
        return self.result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()

