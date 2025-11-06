# FROM SAV: NOV 5

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import time
import math


class FollowBall(Node):
    def __init__(self):
        super().__init__('follow_ball')
       
        # Subscriptions
        self.create_subscription(Point, '/detected_ball', self.listener_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
       
        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
       
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rcv_timeout_secs', 1.0),
                ('angular_chase_multiplier', 0.7),
                ('forward_chase_speed', 0.1),
                ('search_angular_speed', 0.4),
                ('max_size_thresh', 0.1),
                ('filter_value', 0.9),
                ('enable_grid_nav', True),
                ('grid_linear_speed', 0.15),
                ('grid_angular_speed', 0.4),
                ('position_tolerance', 0.20),
                ('angle_tolerance', 0.10),
                ('ball_lost_grid_timeout', 5.0),
                ('rotation_duration', 12.0),
                ('angle_kp', 1.0),
                ('grid_x_limit', 4.0),
                ('grid_y_limit', 4.0)
            ]
        )

        # Get parameters
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.filter_value = self.get_parameter('filter_value').value
        self.enable_grid_nav = self.get_parameter('enable_grid_nav').value
        self.grid_linear_speed = self.get_parameter('grid_linear_speed').value
        self.grid_angular_speed = self.get_parameter('grid_angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.ball_lost_grid_timeout = self.get_parameter('ball_lost_grid_timeout').value
        self.rotation_duration = self.get_parameter('rotation_duration').value
        self.angle_kp = self.get_parameter('angle_kp').value
        self.grid_x_limit = self.get_parameter('grid_x_limit').value
        self.grid_y_limit = self.get_parameter('grid_y_limit').value

        # Grid path (closed loop)
        self.grid_points = [
            (1.0, 1.0),
            (1.0, 3.0),
            (3.0, 3.0),
            (3.0, 1.0),
            (1.0, 1.0),
            (0.0, 0.0)
        ]
        self.current_waypoint_index = 0

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Ball tracking
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        # Mode tracking
        self.current_mode = 'SEARCH'
        self.is_rotating = False
        self.rotation_start_time = 0.0

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('✅ Follow Ball + Grid Navigation Node Started')

    # --- Odometry ---
    def odom_callback(self, msg):
        """Extract pose (x, y, yaw) from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    # --- Timer loop ---
    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        time_since_ball = current_time - self.lastrcvtime

        # Safety: stop if out of bounds
        if abs(self.current_x) > self.grid_x_limit or abs(self.current_y) > self.grid_y_limit:
            self.get_logger().warn(f'⚠️ Out of grid bounds! (x={self.current_x:.2f}, y={self.current_y:.2f}) — stopping.')
            self.publisher_.publish(Twist())
            return

        # --- 360° rotation mode ---
        if self.is_rotating:
            elapsed = current_time - self.rotation_start_time
            if time_since_ball < self.rcv_timeout_secs:
                self.get_logger().info('🎾 Ball detected during rotation → FOLLOW mode')
                self.is_rotating = False
                self.current_mode = 'FOLLOW'
            elif elapsed >= self.rotation_duration:
                self.get_logger().info('✅ Rotation complete, moving to next waypoint')
                self.is_rotating = False
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.grid_points)
            else:
                msg.angular.z = self.grid_angular_speed
                self.current_mode = 'ROTATING'

        # --- Follow mode ---
        elif time_since_ball < self.rcv_timeout_secs:
            if self.current_mode != 'FOLLOW':
                self.get_logger().info('🎾 FOLLOW BALL MODE')
            msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier * self.target_val
            self.current_mode = 'FOLLOW'

        # --- Search mode ---
        elif time_since_ball < self.ball_lost_grid_timeout:
            if self.current_mode != 'SEARCH':
                self.get_logger().info('🔍 SEARCH MODE (spinning)')
            msg.angular.z = self.search_angular_speed
            self.current_mode = 'SEARCH'

        # --- Grid navigation ---
        elif self.enable_grid_nav and self.odom_received:
            if self.current_mode != 'GRID':
                self.get_logger().info('🧭 GRID NAVIGATION MODE')
            msg = self.navigate_grid()
            self.current_mode = 'GRID'

        self.publisher_.publish(msg)

    # --- Grid Navigation Controller ---
    def navigate_grid(self):
        msg = Twist()
        target_x, target_y = self.grid_points[self.current_waypoint_index]

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        angle_error = math.atan2(math.sin(target_angle - self.current_yaw), math.cos(target_angle - self.current_yaw))

        if distance < self.position_tolerance:
            self.get_logger().info(f'✅ Reached waypoint {self.current_waypoint_index}: ({target_x:.1f}, {target_y:.1f})')
            self.is_rotating = True
            self.rotation_start_time = time.time()
            msg.angular.z = self.grid_angular_speed
            return msg

        if abs(angle_error) > self.angle_tolerance:
            msg.angular.z = self.angle_kp * angle_error
            # TEST self.get_logger().info(f'↻ Rotating to face waypoint (error={math.degrees(angle_error):.1f}°)')
        else:
            msg.linear.x = self.grid_linear_speed
            msg.angular.z = 0.5 * angle_error
            # TEST self.get_logger().info(f'➡ Moving to waypoint {self.current_waypoint_index} (dist={distance:.2f} m)')

        return msg

    # --- Ball Detection Callback ---
    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = FollowBall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#####################################

# FROM SAV (OCT 31, RETURN TO PREVIOUS GRID POINT)
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
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
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
        
#         # NEW: Distance limit for ball following
#         self.declare_parameter("max_follow_distance", 3.0)  # Maximum distance from grid point
        
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
#         self.max_follow_distance = self.get_parameter('max_follow_distance').get_parameter_value().double_value
#         self.rotation_duration = self.get_parameter('rotation_duration').get_parameter_value().double_value
        
#         # Tennis court grid points (half court)
#         # COORDINATES BELOW ARE FOR A ~ 12 (along y axis) by 11 (along x axis) m
#         # BUT FOR TESTING: maybe utilize 2 or 3 smaller coordinate points
#         self.grid_points = [
#             (1.0, 1.0), # TESTING: using a small grid
#             (3.0, 1.0),
#             (3.0, 3.0),
#             (1.0, 3.0),
#             # (3.4, 2.2),   # Front left
#             # (3.4, 6.0),   # Center left
#             # (3.4, 9.02),   # Back left
#             # (5.5, 2.2),   # Front center
#             # (5.5, 6.0),   # Center
#             # (5.5, 9.02),   # Back center
#             # (7.6, 2.2),   # Front right
#             # (7.6, 6.0),   # Center right
#             # (7.6, 9.02),   # Back right
#         ]
#         self.current_waypoint_index = 0
        
#         # Robot state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.odom_received = False
        
#         # NEW: Track grid point position when starting to follow ball
#         self.follow_start_x = 0.0
#         self.follow_start_y = 0.0
#         self.is_returning_to_grid = False
        
#         # Ball tracking state
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000
        
#         # Mode tracking
#         self.current_mode = 'SEARCH'  # 'FOLLOW', 'SEARCH', 'GRID', 'ROTATING', or 'RETURN_TO_GRID'
        
#         # 360 rotation state
#         self.is_rotating = False
#         self.rotation_start_time = 0.0
#         self.rotation_start_yaw = 0.0
        
#         # Timer
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         self.get_logger().info('Follow Ball with Grid Navigation and Distance Limit initialized')
    
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
    
#     def get_distance_from_follow_start(self):
#         """Calculate distance from where ball following started"""
#         dx = self.current_x - self.follow_start_x
#         dy = self.current_y - self.follow_start_y
#         return math.sqrt(dx**2 + dy**2)
    
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
#         time_since_ball = current_time - self.lastrcvtime
        
#         # If currently returning to grid point after exceeding distance limit
#         if self.is_returning_to_grid:
#             msg = self.return_to_grid_point()
#             if not self.is_returning_to_grid:  # Check if return complete
#                 # Move to next waypoint
#                 self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.grid_points)
#                 next_x, next_y = self.grid_points[self.current_waypoint_index]
#                 self.get_logger().info(f'Moving to next waypoint {self.current_waypoint_index}: ({next_x:.1f}, {next_y:.1f})')
        
#         # If currently doing 360 rotation at grid point
#         elif self.is_rotating:
#             elapsed_rotation_time = current_time - self.rotation_start_time
            
#             # Check if ball detected during rotation
#             if time_since_ball < self.rcv_timeout_secs:
#                 self.get_logger().info('Ball detected during rotation! Switching to FOLLOW mode')
#                 self.is_rotating = False
#                 self.current_mode = 'FOLLOW'
#                 # Store starting position for distance tracking
#                 self.follow_start_x = self.current_x
#                 self.follow_start_y = self.current_y
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
        
#         # Determine current mode if not rotating or returning
#         elif time_since_ball < self.rcv_timeout_secs:
#             # Ball detected recently - FOLLOW mode
#             if self.current_mode != 'FOLLOW':
#                 self.get_logger().info('>>> FOLLOW BALL MODE <<<')
#                 self.current_mode = 'FOLLOW'
#                 # Store starting position when entering follow mode from another mode
#                 self.follow_start_x = self.current_x
#                 self.follow_start_y = self.current_y
            
#             # Check if exceeded distance limit
#             distance_from_start = self.get_distance_from_follow_start()
            
#             if distance_from_start > self.max_follow_distance:
#                 self.get_logger().warn(f'Exceeded max follow distance ({distance_from_start:.2f}m > {self.max_follow_distance}m)')
#                 self.get_logger().info('Returning to previous grid point...')
#                 self.is_returning_to_grid = True
#                 self.current_mode = 'RETURN_TO_GRID'
#                 msg = self.return_to_grid_point()
#             else:
#                 # Continue following ball
#                 self.get_logger().info(f'Target: {self.target_val:.3f}, Distance from start: {distance_from_start:.2f}m')
                
#                 # Always move forward when following
#                 msg.linear.x = self.forward_chase_speed
#                 msg.angular.z = -self.angular_chase_multiplier * self.target_val
        
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
#             # else: # TEST: GET RID OF GRid nav disabled
#             #     # Grid nav disabled, keep searching
#             #     self.get_logger().info('Grid nav disabled - continuing search')
#             #     msg.angular.z = self.search_angular_speed
        
#         self.publisher_.publish(msg)
    
#     def return_to_grid_point(self):
#         """Return to the grid point where ball following started"""
#         msg = Twist()
        
#         # Calculate distance and angle to return point
#         dx = self.follow_start_x - self.current_x
#         dy = self.follow_start_y - self.current_y
#         distance = math.sqrt(dx**2 + dy**2)
#         target_angle = math.atan2(dy, dx)
        
#         # Calculate angle error
#         angle_error = target_angle - self.current_yaw
#         # Normalize angle to [-pi, pi]
#         angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
#         # Check if returned to grid point
#         if distance < self.position_tolerance:
#             self.get_logger().info(f'Returned to grid point at ({self.follow_start_x:.1f}, {self.follow_start_y:.1f})')
#             self.is_returning_to_grid = False
#             return msg  # Stop moving
        
#         # If not facing the return point, rotate first
#         if abs(angle_error) > self.angle_tolerance:
#             msg.angular.z = self.grid_angular_speed if angle_error > 0 else -self.grid_angular_speed
#             self.get_logger().info(f'Return: Rotating (error={angle_error:.2f} rad)')
        
#         # Move forward toward return point
#         else:
#             msg.linear.x = self.grid_linear_speed
#             msg.angular.z = 0.5 * angle_error  # Minor corrections while moving
#             self.get_logger().info(f'Return: Moving back (dist={distance:.2f}m)')
        
#         return msg
    
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
            
#             # Store this position as potential follow start point
#             self.follow_start_x = self.current_x
#             self.follow_start_y = self.current_y
            
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

######################################################

# FROM SAV (OCT 30, WILL MOVE FORWARD MORE WIP)
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import time
# import math

# class FollowBall(Node):

#     def __init__(self):
#         super().__init__('follow_ball')
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.listener_callback,
#             10)
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         self.declare_parameter("search_angular_speed", 0.5)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)
       
#         # Extra forward movement after reaching the ball
#         self.declare_parameter("extra_forward_distance", 0.5)
#         self.declare_parameter("extra_forward_speed", 0.1)
#         self.declare_parameter("use_odometry_for_extra", False)
       
#         # Scan rotation after extra forward movement
#         self.declare_parameter("scan_after_extra", True)  # NEW: Enable scan after extra move
#         self.declare_parameter("scan_duration", 3.0)  # NEW: How long to scan (seconds)
#         self.declare_parameter("scan_angular_speed", 0.5)  # NEW: Rotation speed during scan
       
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
       
#         # Extra forward movement parameters
#         self.extra_forward_distance = self.get_parameter('extra_forward_distance').get_parameter_value().double_value
#         self.extra_forward_speed = self.get_parameter('extra_forward_speed').get_parameter_value().double_value
#         self.use_odometry_for_extra = self.get_parameter('use_odometry_for_extra').get_parameter_value().bool_value
       
#         # Scan parameters
#         self.scan_after_extra = self.get_parameter('scan_after_extra').get_parameter_value().bool_value
#         self.scan_duration = self.get_parameter('scan_duration').get_parameter_value().double_value
#         self.scan_angular_speed = self.get_parameter('scan_angular_speed').get_parameter_value().double_value

#         timer_period = 0.1
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000

#         # Extra-move state
#         self.extra_move_active = False
#         self.extra_move_start_time = 0.0
#         self.extra_move_start_x = 0.0
#         self.extra_move_start_y = 0.0
       
#         # Scan state (NEW)
#         self.scan_active = False
#         self.scan_start_time = 0.0
       
#         # Odometry state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.odom_received = False

#         # Create odom subscription only if requested
#         # if self.use_odometry_for_extra:
#         #     self.odom_sub = self.create_subscription(
#         #         Odometry,
#         #         '/odom',
#         #         self.odom_callback,
#         #         10)

#     def timer_callback(self):
#         msg = Twist()
       
#         # Priority 1: If scanning for more balls after extra move
#         if self.scan_active:
#             elapsed = time.time() - self.scan_start_time
#             self.get_logger().info(f'SCANNING FOR MORE BALLS: {elapsed:.1f}s / {self.scan_duration:.1f}s')
           
#             if elapsed >= self.scan_duration:
#                 # Scan complete
#                 self.get_logger().info('Scan complete - resuming normal operation')
#                 self.scan_active = False
#                 msg.angular.z = 0.0
#                 msg.linear.x = 0.0
#             else:
#                 # Continue scanning rotation
#                 msg.angular.z = self.scan_angular_speed
#                 msg.linear.x = 0.0
           
#             self.publisher_.publish(msg)
#             return
       
#         # Priority 2: If executing extra forward movement
#         if self.extra_move_active:
#             msg.angular.z = 0.0
#             msg.linear.x = self.extra_forward_speed

#             # Check completion
#             if self.use_odometry_for_extra and self.odom_received:
#                 dx = self.current_x - self.extra_move_start_x
#                 dy = self.current_y - self.extra_move_start_y
#                 moved = math.hypot(dx, dy)
#                 self.get_logger().info(f'Extra move: moved={moved:.3f} / {self.extra_forward_distance:.3f} m')
#                 if moved >= self.extra_forward_distance:
#                     self.get_logger().info('Extra forward distance reached (odom)')
#                     self.extra_move_active = False
#                     msg.linear.x = 0.0
#                     # Start scan if enabled
#                     if self.scan_after_extra:
#                         self.scan_active = True
#                         self.scan_start_time = time.time()
#                         self.get_logger().info('Starting scan for more balls...')
#             else:
#                 elapsed = time.time() - self.extra_move_start_time
#                 duration = self.extra_forward_distance / max(self.extra_forward_speed, 1e-6)
#                 self.get_logger().info(f'Extra move: elapsed={elapsed:.2f}s / {duration:.2f}s')
#                 if elapsed >= duration:
#                     self.get_logger().info('Extra forward distance reached (timed)')
#                     self.extra_move_active = False
#                     msg.linear.x = 0.0
#                     # Start scan if enabled
#                     if self.scan_after_extra:
#                         self.scan_active = True
#                         self.scan_start_time = time.time()
#                         self.get_logger().info('Starting scan for more balls...')
           
#             self.publisher_.publish(msg)
#             return
       
#         # Priority 3: Normal ball chasing behavior
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.get_logger().info('TARGET FOUND!!!')
#             self.get_logger().info('Target X: {:.3f}, Dist: {:.3f}'.format(self.target_val, self.target_dist))
           
#             # Apply deadzone - if ball is centered enough, don't rotate
#             if abs(self.target_val) < self.center_deadzone:
#                 msg.angular.z = 0.0
#                 self.get_logger().info('Ball centered - going straight!')
#             else:
#                 msg.angular.z = -self.angular_chase_multiplier * self.target_val
#                 self.get_logger().info('Adjusting angle: {:.3f}'.format(msg.angular.z))

#             # If the target is within the size threshold, trigger the extra forward move
#             if (self.target_dist >= self.max_size_thresh) and (not self.extra_move_active):
#                 self.get_logger().info(f'Target within size threshold ({self.target_dist:.3f} >= {self.max_size_thresh:.3f}) - starting extra forward move')
#                 self.extra_move_active = True
#                 self.extra_move_start_time = time.time()
#                 if self.use_odometry_for_extra:
#                     if not self.odom_received:
#                         self.get_logger().warning('use_odometry_for_extra True but no odom received; falling back to timed extra move')
#                         self.use_odometry_for_extra = False
#                     else:
#                         self.extra_move_start_x = self.current_x
#                         self.extra_move_start_y = self.current_y

#             msg.linear.x = self.forward_chase_speed
           
#         else:
#             self.get_logger().info('TARGET LOST!!!!!')
#             msg.angular.z = self.search_angular_speed
#             msg.linear.x = 0.0
           
#         self.publisher_.publish(msg)

#     def listener_callback(self, msg):
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1-f)
#         self.target_dist = self.target_dist * f + msg.z * (1-f)
#         self.lastrcvtime = time.time()
       
#         # If we detect a ball during scan, interrupt the scan
#         if self.scan_active:
#             self.get_logger().info('Ball detected during scan - interrupting scan!')
#             self.scan_active = False

#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         self.odom_received = True


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

##############################################################################################################################################################################
# FROM SAV (OCT 29, WORKING SOMEWHAT)

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
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10) # self.create_publisher(Twist, '/cmd_vel', 10)

#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)  # Changed from 0.0 to 0.7
#         self.declare_parameter("forward_chase_speed", 0.2)  # Changed from 0.0 to 0.2
#         self.declare_parameter("search_angular_speed", 0.5)  # Changed from 0.0 to 0.5
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)  # NEW: deadzone for "centered" detection

#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value

#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000

#     def timer_callback(self):
#         msg = Twist()
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.get_logger().info('TARGET FOUND!!!')
#             self.get_logger().info('Target X: {:.3f}, Dist: {:.3f}'.format(self.target_val, self.target_dist))
            
#             # Apply deadzone - if ball is centered enough, don't rotate
#             if abs(self.target_val) < self.center_deadzone:
#                 msg.angular.z = 0.0  # Go perfectly straight
#                 self.get_logger().info('Ball centered - going straight!')
#             else:
#                 msg.angular.z = -self.angular_chase_multiplier * self.target_val
#                 self.get_logger().info('Adjusting angle: {:.3f}'.format(msg.angular.z))
            
#             msg.linear.x = self.forward_chase_speed
            
#         else:
#             # TEST!
#             # msg.angular.z = 0.0  # Go perfectly straight
#             # sleep(1)

#             self.get_logger().info('TARGET LOST!!!!!')
#             msg.angular.z = self.search_angular_speed
#             msg.linear.x = 0.0  # Stop moving forward when searching
            
#         self.publisher_.publish(msg)

#     def listener_callback(self, msg):
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

#############################################################################################################################################################

# CODE BELOW HAS GRID IMPLEMENETATION

# TO TEST: (Oct 25th), 
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
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10) #'/cmd_vel', 10)
        
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
#             (1.0, 1.0),
#             (1.0, 3.0),
#             (3.0, 3.0),
#             (3.0, 1.0),
#             # (1.0, 1.0),
#             # (3.4, 2.2),   # Front left
#             # (3.4, 6.0),   # Center left
#             # (3.4, 9.02),   # Back left
#             # (5.5, 2.2),   # Front center
#             # (5.5, 6.0),   # Center
#             # (5.5, 9.02),   # Back center
#             # (7.6, 2.2),   # Front right
#             # (7.6, 6.0),   # Center right
#             # (7.6, 9.02),   # Back right
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
#             # TEST
#             # else:
#             #     # Grid nav disabled, keep searching
#             #     self.get_logger().info('Grid nav disabled - continuing search')
#             #     msg.angular.z = self.search_angular_speed
        
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
#         # self.lastrcvtime = time.time() # TEST: Remove detection for now


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


