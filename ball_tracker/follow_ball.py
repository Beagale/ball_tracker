# FROM SAV
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowBall(Node):

    def __init__(self):
        super().__init__('follow_ball')
        self.subscription = self.create_subscription(
            Point,
            '/detected_ball',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)  # Changed from 0.0 to 0.7
        self.declare_parameter("forward_chase_speed", 0.2)  # Changed from 0.0 to 0.2
        self.declare_parameter("search_angular_speed", 0.5)  # Changed from 0.0 to 0.5
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("center_deadzone", 0.05)  # NEW: deadzone for "centered" detection
        
        #TEST
        self.declare_parameter("zero_angular_override", False)  # If true, force angular.z to 0

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value

        #TEST
        self.zero_angular_override = self.get_parameter('zero_angular_override').get_parameter_value().bool_value

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

    def timer_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info('TARGET FOUND!!!')
            self.get_logger().info('Target X: {:.3f}, Dist: {:.3f}'.format(self.target_val, self.target_dist))
            
            # Apply deadzone - if ball is centered enough, don't rotate
            if abs(self.target_val) < self.center_deadzone:
                msg.angular.z = 0.0  # Go perfectly straight
                self.get_logger().info('Ball centered - going straight!')
            else:
                msg.angular.z = -self.angular_chase_multiplier * self.target_val
                self.get_logger().info('Adjusting angle: {:.3f}'.format(msg.angular.z))
                
            #TEST
            # Runtime override to force zero angular velocity (useful for debugging / testing)
            if self.zero_angular_override:
                if msg.angular.z != 0.0:
                    self.get_logger().info('zero_angular_override active - forcing angular.z to 0')
                msg.angular.z = 0.0
            
            msg.linear.x = self.forward_chase_speed
            
        else:
            self.get_logger().info('TARGET LOST!!!!!')
            msg.angular.z = self.search_angular_speed
            msg.linear.x = 0.0  # Stop moving forward when searching
            
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    rclpy.spin(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# UNCOMMENT EITHER (1) or (2) for testing

# (1) TO TEST: (MODIFIED FOR TESTING) Josh's follow_ball.py code
# CAN RUN THIS FIRST TO TEST THE REGULAR BALL TRACKING W/O NAVIGATIONAL ALGORITHM
# REVISED (NOT THE SAME CODE, CHANGED IF STATEMENT AND SPEED
# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Twist
# import time

# class FollowBall(Node):

#     def __init__(self):
#         super().__init__('follow_ball')
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.listener_callback,
#             10)
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)


#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.0) # 0.7
#         self.declare_parameter("forward_chase_speed", 0.0) # 0.1
#         self.declare_parameter("search_angular_speed", 0.0) # 0.5
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)


#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000

#     def timer_callback(self):
#         msg = Twist()
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.get_logger().info('TARGET FOUND!!!') # TEST
#             self.get_logger().info('Target: {}'.format(self.target_val))
#             print(self.target_dist)
#             # if (self.target_dist < self.max_size_thresh): # TEST: Remove the if statement to keep bot moving forward
#             msg.linear.x = self.forward_chase_speed
#             # msg.angular.z = -self.angular_chase_multiplier*self.target_val # TEST: to see if bot will still rotate
#             self.get_logger().info(f'ANgular speed: {msg.angular.z:.3f} rad/s') # TEST: to get the angular speed
#         else:
#             self.get_logger().info('TARGET LOST!!!!!') # TEST
#             self.get_logger().info('Target lost')
#             self.get_logger().info(f'ANgular speed: {msg.angular.z:.3f} rad/s') # TEST: to get the angular speed
#             # msg.angular.z = self.search_angular_speed # TEST: to see if bot will still rotate
#         self.publisher_.publish(msg)

#     def listener_callback(self, msg):
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1-f)
#         # self.target_dist = self.target_dist * f + msg.z * (1-f) # TEST: to remove the angular correcting
#         self.lastrcvtime = time.time()
#         # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()



# (2) TO TEST: Used the NEW code (Oct 25th), 
# CODE BELOW HAS BEEN MODIFIED TO INCLUDE A GRID SYSTEM (used Claude AI)

# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, Twist
# from nav_msgs.msg import Odometry
# import time
# import math


# class FollowBall(Node):
#     def __init__(self):
#         super().__init__('follow_ball')
        
#         # Ball detection subscription
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.listener_callback,
#             10)
        
#         # Odometry subscription for grid navigation
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10)
        
#         # Command velocity publisher
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         # Ball following parameters
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.1)
#         self.declare_parameter("search_angular_speed", 0.5)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
        
#         # Grid navigation parameters
#         self.declare_parameter("enable_grid_nav", True)
#         self.declare_parameter("grid_linear_speed", 0.2)
#         self.declare_parameter("grid_angular_speed", 0.5)
#         self.declare_parameter("position_tolerance", 0.15)
#         self.declare_parameter("angle_tolerance", 0.1)
#         self.declare_parameter("ball_lost_grid_timeout", 5.0)
        
#         # 360 rotation parameters
#         self.declare_parameter("rotation_duration", 12.6)  # Time for 360° (2π / angular_speed)
        
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        
#         self.enable_grid_nav = self.get_parameter('enable_grid_nav').get_parameter_value().bool_value
#         self.grid_linear_speed = self.get_parameter('grid_linear_speed').get_parameter_value().double_value
#         self.grid_angular_speed = self.get_parameter('grid_angular_speed').get_parameter_value().double_value
#         self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
#         self.ball_lost_grid_timeout = self.get_parameter('ball_lost_grid_timeout').get_parameter_value().double_value
#         self.rotation_duration = self.get_parameter('rotation_duration').get_parameter_value().double_value
        
#         # Tennis court grid points (half court)
#         # COORDINATES BELOW ARE FOR A ~ 12 (along y axis) by 11 (along x axis) m
#         # BUT FOR TESTING: maybe utilize 2 or 3 smaller coordinate points
#         self.grid_points = [
#             (3.4, 2.2),   # Front left
#             (3.4, 6.0),   # Center left
#             (3.4, 9.02),   # Back left
#             (5.5, 2.2),   # Front center
#             (5.5, 6.0),   # Center
#             (5.5, 9.02),   # Back center
#             (7.6, 2.2),   # Front right
#             (7.6, 6.0),   # Center right
#             (7.6, 9.02),   # Back right
#         ]
#         self.current_waypoint_index = 0
        
#         # Robot state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.odom_received = False
        
#         # Ball tracking state
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000
        
#         # Mode tracking
#         self.current_mode = 'SEARCH'  # 'FOLLOW', 'SEARCH', 'GRID', or 'ROTATING'
        
#         # 360 rotation state
#         self.is_rotating = False
#         self.rotation_start_time = 0.0
#         self.rotation_start_yaw = 0.0
        
#         # Timer
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         self.get_logger().info('Follow Ball with Grid Navigation and 360° Rotation initialized')
    
#     def odom_callback(self, msg):
#         """Process odometry data for grid navigation"""
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
        
#         # Extract yaw from quaternion
#         orientation_q = msg.pose.pose.orientation
#         siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
#         cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
#         self.odom_received = True
    
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
#         time_since_ball = current_time - self.lastrcvtime
        
#         # If currently doing 360 rotation at grid point
#         if self.is_rotating:
#             elapsed_rotation_time = current_time - self.rotation_start_time
            
#             # Check if ball detected during rotation
#             if time_since_ball < self.rcv_timeout_secs:
#                 self.get_logger().info('Ball detected during rotation! Switching to FOLLOW mode')
#                 self.is_rotating = False
#                 self.current_mode = 'FOLLOW'
#             # Check if rotation complete
#             elif elapsed_rotation_time >= self.rotation_duration:
#                 self.get_logger().info('360° rotation complete')
#                 self.is_rotating = False
#                 # Move to next waypoint
#                 self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.grid_points)
#                 next_x, next_y = self.grid_points[self.current_waypoint_index]
#                 self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index}: ({next_x:.1f}, {next_y:.1f})')
#             else:
#                 # Continue rotating
#                 if self.current_mode != 'ROTATING':
#                     self.current_mode = 'ROTATING'
#                 msg.angular.z = self.grid_angular_speed
#                 progress = (elapsed_rotation_time / self.rotation_duration) * 100
#                 self.get_logger().info(f'Rotating at grid point... {progress:.1f}% complete')
        
#         # Determine current mode if not rotating
#         elif time_since_ball < self.rcv_timeout_secs:
#             # Ball detected recently - FOLLOW mode
#             if self.current_mode != 'FOLLOW':
#                 self.get_logger().info('>>> FOLLOW BALL MODE <<<')
#                 self.current_mode = 'FOLLOW'
            
#             self.get_logger().info('Target: {}'.format(self.target_val))
            
#             # Always move forward when following
#             msg.linear.x = self.forward_chase_speed
#             msg.angular.z = -self.angular_chase_multiplier * self.target_val
        
#         elif time_since_ball < self.ball_lost_grid_timeout:
#             # Ball lost recently - SEARCH mode (rotate in place)
#             if self.current_mode != 'SEARCH':
#                 self.get_logger().info('>>> SEARCH MODE (rotating) <<<')
#                 self.current_mode = 'SEARCH'
            
#             self.get_logger().info('Searching for ball...')
#             msg.angular.z = self.search_angular_speed
        
#         else:
#             # Ball lost for too long - GRID mode
#             if self.enable_grid_nav and self.odom_received:
#                 if self.current_mode != 'GRID':
#                     self.get_logger().info('>>> GRID NAVIGATION MODE <<<')
#                     self.current_mode = 'GRID'
                
#                 msg = self.navigate_grid()
#             else:
#                 # Grid nav disabled, keep searching
#                 self.get_logger().info('Grid nav disabled - continuing search')
#                 msg.angular.z = self.search_angular_speed
        
#         self.publisher_.publish(msg)
    
#     def navigate_grid(self):
#         """Navigate to grid waypoints"""
#         msg = Twist()
        
#         # Get current target waypoint
#         target_x, target_y = self.grid_points[self.current_waypoint_index]
        
#         # Calculate distance and angle to target
#         dx = target_x - self.current_x
#         dy = target_y - self.current_y
#         distance = math.sqrt(dx**2 + dy**2)
#         target_angle = math.atan2(dy, dx)
        
#         # Calculate angle error
#         angle_error = target_angle - self.current_yaw
#         # Normalize angle to [-pi, pi]
#         angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
#         # Check if waypoint reached
#         if distance < self.position_tolerance:
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}: ({target_x:.1f}, {target_y:.1f})')
#             self.get_logger().info('Starting 360° rotation to search for ball...')
            
#             # Start 360 rotation
#             self.is_rotating = True
#             self.rotation_start_time = time.time()
#             self.rotation_start_yaw = self.current_yaw
            
#             # Return rotation command
#             msg.angular.z = self.grid_angular_speed
        
#         # If not facing the target, rotate first
#         elif abs(angle_error) > self.angle_tolerance:
#             msg.angular.z = self.grid_angular_speed if angle_error > 0 else -self.grid_angular_speed
#             self.get_logger().info(f'Grid: Rotating (error={angle_error:.2f} rad)')
        
#         # Move forward toward target
#         else:
#             msg.linear.x = self.grid_linear_speed
#             msg.angular.z = 0.5 * angle_error  # Minor corrections while moving
#             self.get_logger().info(f'Grid: Moving forward (dist={distance:.2f}m)')
        
#         return msg
    
#     def listener_callback(self, msg):
#         """Process detected ball data"""
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1-f)
#         self.target_dist = self.target_dist * f + msg.z * (1-f)
#         self.lastrcvtime = time.time()


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


