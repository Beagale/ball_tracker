# FROM ABBY (NOV 11) should reorient itself correctly to move to next point

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
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        # Original parameters
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.2)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("center_deadzone", 0.05)
        
        # New parameters for square grid navigation
        self.declare_parameter("square_side_duration", 5.0)  # Time to travel one side (seconds)
        self.declare_parameter("rotation_duration", 3)  # Time to rotate 90 degrees (seconds)
        self.declare_parameter("search_duration", 15.5)  # Time to search at each corner (seconds)
        self.declare_parameter("grid_forward_speed", 0.2)  # Speed when moving between corners
        self.declare_parameter("grid_angular_speed", 0.5)  # Speed when rotating at corners
        
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
        
        self.square_side_duration = self.get_parameter('square_side_duration').get_parameter_value().double_value
        self.rotation_duration = self.get_parameter('rotation_duration').get_parameter_value().double_value
        self.search_duration = self.get_parameter('search_duration').get_parameter_value().double_value
        self.grid_forward_speed = self.get_parameter('grid_forward_speed').get_parameter_value().double_value
        self.grid_angular_speed = self.get_parameter('grid_angular_speed').get_parameter_value().double_value
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        
        # State machine for square grid navigation
        self.state = "MOVING_TO_CORNER"  # States: MOVING_TO_CORNER, ROTATING, SEARCHING, COLLECTING, RETURNING, REORIENTING
        self.current_corner = 0  # 0, 1, 2, 3 for the four corners
        self.state_start_time = time.time()
        self.collection_start_pos = None  # Store where we started collecting
        self.return_duration = 0.0  # Time it took to collect (for returning)
        self.reorientation_angle = 0.0  # Angle turned during collection (to reverse)
        
    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed = current_time - self.state_start_time
        
        # Check if ball is detected
        ball_detected = (time.time() - self.lastrcvtime < self.rcv_timeout_secs)
        
        # State machine
        if self.state == "MOVING_TO_CORNER":
            self.get_logger().info(f'Moving to corner {self.current_corner}, elapsed: {elapsed:.2f}s')
            msg.linear.x = self.grid_forward_speed
            msg.angular.z = 0.0
            
            # Check if ball detected while moving
            if ball_detected:
                self.get_logger().info('Ball detected while moving! Starting collection.')
                self.state = "COLLECTING"
                self.collection_start_pos = current_time
                self.return_duration = 0.0
                self.reorientation_angle = 0.0  # Reset angle tracking
            elif elapsed >= self.square_side_duration:
                # Reached corner, start rotating
                self.state = "ROTATING"
                self.state_start_time = current_time
                
        elif self.state == "ROTATING":
            self.get_logger().info(f'Rotating at corner {self.current_corner}, elapsed: {elapsed:.2f}s')
            msg.linear.x = 0.0
            msg.angular.z = self.grid_angular_speed
            
            # Check if ball detected while rotating
            if ball_detected:
                self.get_logger().info('Ball detected while rotating! Starting collection.')
                self.state = "COLLECTING"
                self.collection_start_pos = current_time
                self.return_duration = 0.0
                self.reorientation_angle = 0.0  # Reset angle tracking
            elif elapsed >= self.rotation_duration:
                # Finished rotating, start searching
                self.state = "SEARCHING"
                self.state_start_time = current_time
                
        elif self.state == "SEARCHING":
            self.get_logger().info(f'Searching at corner {self.current_corner}, elapsed: {elapsed:.2f}s')
            msg.linear.x = 0.0
            msg.angular.z = self.search_angular_speed
            
            if ball_detected:
                self.get_logger().info('Ball found during search! Starting collection.')
                self.state = "COLLECTING"
                self.collection_start_pos = current_time
                self.return_duration = 0.0
                self.reorientation_angle = 0.0  # Reset angle tracking
            elif elapsed >= self.search_duration:
                # No ball found, move to next corner
                self.current_corner = (self.current_corner + 1) % 4
                self.state = "MOVING_TO_CORNER"
                self.state_start_time = current_time
                self.get_logger().info(f'Moving to next corner: {self.current_corner}')
                
        elif self.state == "COLLECTING":
            self.get_logger().info('COLLECTING BALL!!!')
            self.get_logger().info('Target X: {:.3f}, Dist: {:.3f}'.format(self.target_val, self.target_dist))
            
            if ball_detected:
                # Track collection duration
                self.return_duration = current_time - self.collection_start_pos
                
                # Apply deadzone - if ball is centered enough, don't rotate
                if abs(self.target_val) < self.center_deadzone:
                    msg.angular.z = 0.0
                    self.get_logger().info('Ball centered - going straight!')
                else:
                    msg.angular.z = -self.angular_chase_multiplier * self.target_val
                    # Track the angular velocity to estimate total rotation
                    self.reorientation_angle += msg.angular.z * 0.1  # Accumulate angle (angular_vel * dt)
                    self.get_logger().info('Adjusting angle: {:.3f}'.format(msg.angular.z))
                
                msg.linear.x = self.forward_chase_speed
                
                # Check if ball is close enough (collected)
                if self.target_dist > self.max_size_thresh:
                    self.get_logger().info('Ball collected! Returning to position.')
                    self.state = "RETURNING"
                    self.state_start_time = current_time
            else:
                # Lost the ball during collection, return
                self.get_logger().info('Lost ball during collection! Returning to position.')
                self.state = "RETURNING"
                self.state_start_time = current_time
                
        elif self.state == "RETURNING":
            self.get_logger().info(f'Returning to grid position, elapsed: {elapsed:.2f}s')
            # Move backward for the same duration as collection
            msg.linear.x = -self.grid_forward_speed
            msg.angular.z = 0.0
            
            if elapsed >= self.return_duration:
                # Returned, now need to reorient
                self.state = "REORIENTING"
                self.state_start_time = current_time
                self.get_logger().info(f'Returned! Reorienting by {-self.reorientation_angle:.3f} radians')
                
        elif self.state == "REORIENTING":
            self.get_logger().info(f'Reorienting to face next corner, elapsed: {elapsed:.2f}s')
            # Rotate in opposite direction to undo collection rotation
            # Calculate how long to rotate based on accumulated angle
            reorient_duration = abs(self.reorientation_angle) / self.grid_angular_speed
            
            if self.reorientation_angle > 0:
                msg.angular.z = -self.grid_angular_speed  # Rotate opposite direction
            elif self.reorientation_angle < 0:
                msg.angular.z = self.grid_angular_speed
            else:
                msg.angular.z = 0.0
                
            msg.linear.x = 0.0
            
            if elapsed >= reorient_duration or abs(self.reorientation_angle) < 0.01:
                # Finished reorienting, continue to next corner
                self.current_corner = (self.current_corner + 1) % 4
                self.state = "MOVING_TO_CORNER"
                self.state_start_time = current_time
                self.get_logger().info(f'Reoriented! Moving to next corner: {self.current_corner}')
        
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

#############################################

# FROM ABBY (NOV 10)

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
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Original parameters
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         self.declare_parameter("search_angular_speed", 0.5)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)
        
#         # New parameters for square grid navigation
#         self.declare_parameter("square_side_duration", 5.0)  # Time to travel one side (seconds)
#         self.declare_parameter("rotation_duration", 3)  # Time to rotate 90 degrees (seconds)
#         self.declare_parameter("search_duration", 15.5)  # Time to search at each corner (seconds)
#         self.declare_parameter("grid_forward_speed", 0.2)  # Speed when moving between corners
#         self.declare_parameter("grid_angular_speed", 0.5)  # Speed when rotating at corners
        
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
        
#         self.square_side_duration = self.get_parameter('square_side_duration').get_parameter_value().double_value
#         self.rotation_duration = self.get_parameter('rotation_duration').get_parameter_value().double_value
#         self.search_duration = self.get_parameter('search_duration').get_parameter_value().double_value
#         self.grid_forward_speed = self.get_parameter('grid_forward_speed').get_parameter_value().double_value
#         self.grid_angular_speed = self.get_parameter('grid_angular_speed').get_parameter_value().double_value
        
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000
        
#         # State machine for square grid navigation
#         self.state = "MOVING_TO_CORNER"  # States: MOVING_TO_CORNER, ROTATING, SEARCHING, COLLECTING, RETURNING
#         self.current_corner = 0  # 0, 1, 2, 3 for the four corners
#         self.state_start_time = time.time()
#         self.collection_start_pos = None  # Store where we started collecting
#         self.return_duration = 0.0  # Time it took to collect (for returning)
        
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
#         elapsed = current_time - self.state_start_time
        
#         # Check if ball is detected
#         ball_detected = (time.time() - self.lastrcvtime < self.rcv_timeout_secs)
        
#         # State machine
#         if self.state == "MOVING_TO_CORNER":
#             self.get_logger().info(f'Moving to corner {self.current_corner}, elapsed: {elapsed:.2f}s')
#             msg.linear.x = self.grid_forward_speed
#             msg.angular.z = 0.0
            
#             # Check if ball detected while moving
#             if ball_detected:
#                 self.get_logger().info('Ball detected while moving! Starting collection.')
#                 self.state = "COLLECTING"
#                 self.collection_start_pos = current_time
#                 self.return_duration = 0.0
#             elif elapsed >= self.square_side_duration:
#                 # Reached corner, start rotating
#                 self.state = "ROTATING"
#                 self.state_start_time = current_time
                
#         elif self.state == "ROTATING":
#             self.get_logger().info(f'Rotating at corner {self.current_corner}, elapsed: {elapsed:.2f}s')
#             msg.linear.x = 0.0
#             msg.angular.z = self.grid_angular_speed
            
#             # Check if ball detected while rotating
#             if ball_detected:
#                 self.get_logger().info('Ball detected while rotating! Starting collection.')
#                 self.state = "COLLECTING"
#                 self.collection_start_pos = current_time
#                 self.return_duration = 0.0
#             elif elapsed >= self.rotation_duration:
#                 # Finished rotating, start searching
#                 self.state = "SEARCHING"
#                 self.state_start_time = current_time
                
#         elif self.state == "SEARCHING":
#             self.get_logger().info(f'Searching at corner {self.current_corner}, elapsed: {elapsed:.2f}s')
#             msg.linear.x = 0.0
#             msg.angular.z = self.search_angular_speed
            
#             if ball_detected:
#                 self.get_logger().info('Ball found during search! Starting collection.')
#                 self.state = "COLLECTING"
#                 self.collection_start_pos = current_time
#                 self.return_duration = 0.0
#             elif elapsed >= self.search_duration:
#                 # No ball found, move to next corner
#                 self.current_corner = (self.current_corner + 1) % 4
#                 self.state = "MOVING_TO_CORNER"
#                 self.state_start_time = current_time
#                 self.get_logger().info(f'Moving to next corner: {self.current_corner}')
                
#         elif self.state == "COLLECTING":
#             self.get_logger().info('COLLECTING BALL!!!')
#             self.get_logger().info('Target X: {:.3f}, Dist: {:.3f}'.format(self.target_val, self.target_dist))
            
#             if ball_detected:
#                 # Track collection duration
#                 self.return_duration = current_time - self.collection_start_pos
                
#                 # Apply deadzone - if ball is centered enough, don't rotate
#                 if abs(self.target_val) < self.center_deadzone:
#                     msg.angular.z = 0.0
#                     self.get_logger().info('Ball centered - going straight!')
#                 else:
#                     msg.angular.z = -self.angular_chase_multiplier * self.target_val
#                     self.get_logger().info('Adjusting angle: {:.3f}'.format(msg.angular.z))
                
#                 msg.linear.x = self.forward_chase_speed
                
#                 # Check if ball is close enough (collected)
#                 if self.target_dist > self.max_size_thresh:
#                     self.get_logger().info('Ball collected! Returning to position.')
#                     self.state = "RETURNING"
#                     self.state_start_time = current_time
#             else:
#                 # Lost the ball during collection, return
#                 self.get_logger().info('Lost ball during collection! Returning to position.')
#                 self.state = "RETURNING"
#                 self.state_start_time = current_time
                
#         elif self.state == "RETURNING":
#             self.get_logger().info(f'Returning to grid position, elapsed: {elapsed:.2f}s')
#             # Move backward for the same duration as collection
#             msg.linear.x = -self.grid_forward_speed
#             msg.angular.z = 0.0
            
#             if elapsed >= self.return_duration:
#                 # Returned, continue to next corner
#                 self.current_corner = (self.current_corner + 1) % 4
#                 self.state = "MOVING_TO_CORNER"
#                 self.state_start_time = current_time
#                 self.get_logger().info(f'Returned! Moving to next corner: {self.current_corner}')
        
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

######################

# FROM SAV (NOV 10) SQUARE CODE W/ ROTATION AT EACH POINT AND GOES TOWARDS BALL

#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point
# import time
# import math
# import math as _math

# class SquareGridWithCollection(Node):
#     def __init__(self):
#         super().__init__('square_grid_with_collection')

#         # Publisher for robot velocity commands
#         self.cmd_topic = '/diff_cont/cmd_vel_unstamped'
#         self.publisher_ = self.create_publisher(Twist, self.cmd_topic, 10)

#         # Subscriber for ball detection
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.ball_callback,
#             10)

#         # Parameters
#         self.declare_parameter("linear_speed", 0.2)
#         self.declare_parameter("angular_speed", 0.5)
#         self.declare_parameter("square_side_length", 1.0)
#         self.declare_parameter("turn_angle", 90.0)
#         self.declare_parameter("detection_spins", 3)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("approach_speed", 0.15)
#         self.declare_parameter("ball_collection_distance", 0.3)
#         self.declare_parameter("center_deadzone", 0.05)
#         self.declare_parameter("ball_timeout", 1.0)
#         # small safety param: ignore unrealistically small z (might be sensor artifact)
#         self.declare_parameter("min_valid_z", 0.01)

#         self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
#         self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
#         self.square_side_length = self.get_parameter('square_side_length').get_parameter_value().double_value
#         self.turn_angle = self.get_parameter('turn_angle').get_parameter_value().double_value
#         self.detection_spins = self.get_parameter('detection_spins').get_parameter_value().integer_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.approach_speed = self.get_parameter('approach_speed').get_parameter_value().double_value
#         self.ball_collection_distance = self.get_parameter('ball_collection_distance').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
#         self.ball_timeout = self.get_parameter('ball_timeout').get_parameter_value().double_value
#         self.min_valid_z = self.get_parameter('min_valid_z').get_parameter_value().double_value

#         # State machine
#         self.state = "FORWARD"
#         self.side_count = 0
#         self.spin_count = 0
#         self.start_time = None
#         self.movement_duration = 0.0

#         # Ball detection variables
#         self.ball_detected = False
#         self.ball_x = 0.0
#         self.ball_z = float('inf')
#         self.last_ball_time = 0.0

#         # Position tracking for return
#         self.original_position_distance = 0.0
#         self.return_start_time = None

#         # Timer
#         self.timer_period = 0.1  # seconds
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)

#         # Logging header
#         self.get_logger().info('=' * 60)
#         self.get_logger().info('Square Grid with Ball Collection Node Started!')
#         self.get_logger().info(f'Publishing cmd_vel to: {self.cmd_topic}')
#         self.get_logger().info(f'Square side length: {self.square_side_length}m')
#         self.get_logger().info(f'Detection spins per corner: {self.detection_spins}')
#         self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
#         self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
#         self.get_logger().info(f'Approach speed: {self.approach_speed} m/s')
#         self.get_logger().info(f'Collection distance: {self.ball_collection_distance}m')
#         self.get_logger().info('=' * 60)

#     def ball_callback(self, msg: Point):
#         """Callback when ball is detected"""
#         # Defensive: if msg.z invalid -> ignore
#         if msg is None:
#             return

#         # Some detectors may publish tiny or negative z; treat those as invalid
#         if msg.z is None or _math.isnan(msg.z) or msg.z <= 0.0:
#             self.get_logger().debug(f'Ignored invalid z from detector: {msg.z}')
#             return

#         self.ball_detected = True
#         self.ball_x = float(msg.x)
#         self.ball_z = float(msg.z)
#         self.last_ball_time = time.time()

#         self.get_logger().debug(f'BALL CALLBACK: X={self.ball_x:.3f} Z={self.ball_z:.3f} (state={self.state})')

#     def timer_callback(self):
#         """Main control loop executed on timer"""
#         msg = Twist()
#         now = time.time()

#         # Helper small fns
#         def publish_and_log(tag=''):
#             # Publish twist and small status line for debugging
#             self.publisher_.publish(msg)
#             self.get_logger().debug(f'[FINAL CMD] state={self.state:15s} linear={msg.linear.x:.3f} angular={msg.angular.z:.3f} {tag}')

#         # If detection is stale, consider ball lost
#         if self.ball_detected and (now - self.last_ball_time > self.ball_timeout):
#             self.get_logger().info(f'[GLOBAL] Ball detection timed out ({now - self.last_ball_time:.2f}s).')
#             self.ball_detected = False
#             self.ball_x = 0.0
#             self.ball_z = float('inf')

#         # ---------- STATE: FORWARD ----------
#         if self.state == "FORWARD":
#             if self.start_time is None:
#                 self.start_time = now
#                 self.movement_duration = self.square_side_length / max(1e-6, self.linear_speed)
#                 self.get_logger().info(f'[FORWARD] Starting side {self.side_count + 1}/4 ({self.square_side_length}m)')

#             elapsed = now - self.start_time
#             if elapsed < self.movement_duration:
#                 msg.linear.x = self.linear_speed
#                 msg.angular.z = 0.0
#                 publish_and_log()
#                 return
#             else:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[FORWARD complete]')
#                 self.state = "WAITING"
#                 self.start_time = now
#                 self.movement_duration = 0.5
#                 self.spin_count = 0
#                 self.ball_detected = False
#                 return

#         # ---------- STATE: WAITING ----------
#         if self.state == "WAITING":
#             elapsed = now - self.start_time
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[WAITING]')
#                 return
#             else:
#                 self.get_logger().info('[WAITING] Starting DETECT_BALL')
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 return

#         # ---------- STATE: DETECT_BALL ----------
#         if self.state == "DETECT_BALL":
#             if self.start_time is None:
#                 self.start_time = now
#                 full_rotation_rad = 2.0 * math.pi
#                 self.movement_duration = full_rotation_rad / max(1e-6, self.angular_speed)
#                 self.get_logger().info(f'[DETECT_BALL] Starting spin {self.spin_count + 1}/{self.detection_spins}')

#             elapsed = now - self.start_time

#             # If ball detected and fresh
#             if self.ball_detected:
#                 # move to align state
#                 self.get_logger().info('[DETECT_BALL] BALL DETECTED, ALIGNING!')
#                 self.state = "ALIGN_TO_BALL"
#                 self.start_time = None
#                 # ensure we start with fresh tracking counters for approach
#                 self.original_position_distance = 0.0
#                 publish_and_log('[DETECT_BALL -> ALIGN]')
#                 return

#             # otherwise continue spinning
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = self.angular_speed
#                 publish_and_log('[DETECT_BALL spinning]')
#                 return
#             else:
#                 # spin finished
#                 self.spin_count += 1
#                 self.get_logger().info(f'[DETECT_BALL] Spin {self.spin_count}/{self.detection_spins} complete')
#                 if self.spin_count >= self.detection_spins:
#                     self.get_logger().info('[DETECT_BALL] All spins complete. No ball found.')
#                     self.state = "WAITING_BEFORE_TURN"
#                     self.start_time = now
#                     self.movement_duration = 0.5
#                     publish_and_log('[no-ball]')
#                     return
#                 else:
#                     self.start_time = None
#                     publish_and_log('[DETECT_BALL next spin]')
#                     return

#         # ---------- STATE: ALIGN_TO_BALL ----------
#         if self.state == "ALIGN_TO_BALL":
#             # if we lost the ball mid-align
#             if not self.ball_detected:
#                 self.get_logger().warn('[ALIGN_TO_BALL] Ball lost — returning to detection.')
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 publish_and_log('[lost ball]')
#                 return

#             # Sanity check on ball_z
#             if self.ball_z < self.min_valid_z:
#                 self.get_logger().warn(f'[ALIGN_TO_BALL] Ignoring suspicious small z={self.ball_z:.4f}. Treating as lost.')
#                 self.ball_detected = False
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 publish_and_log('[bad z]')
#                 return

#             centered = abs(self.ball_x) < self.center_deadzone
#             self.get_logger().info(f'[ALIGN_TO_BALL] Ball X={self.ball_x:.3f} centered={centered}')
#             if centered:
#                 # ball roughly centered; move to approach
#                 self.get_logger().info('[ALIGN_TO_BALL] Centered — starting approach.')
#                 self.state = "APPROACH_BALL"
#                 # mark when approach started so distance integration works
#                 self.start_time = now
#                 # distance traveled toward ball so far (we compute during approach)
#                 self.original_position_distance = 0.0
#                 publish_and_log('[aligned -> approach]')
#                 return
#             else:
#                 # rotate towards ball; negative sign depends on detector coordinate convention
#                 msg.linear.x = 0.0
#                 msg.angular.z = -self.angular_chase_multiplier * float(self.ball_x)
#                 publish_and_log('[ALIGNING]')
#                 return

#         # ---------- STATE: APPROACH_BALL ----------
#         if self.state == "APPROACH_BALL":
#             # lost check
#             if not self.ball_detected:
#                 self.get_logger().warn('[APPROACH_BALL] Ball lost — back to detection.')
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 publish_and_log('[lost during approach]')
#                 return

#             # Sanity check on ball_z
#             if self.ball_z < self.min_valid_z:
#                 self.get_logger().warn(f'[APPROACH_BALL] Suspicious small z={self.ball_z:.4f}. Cancelling approach.')
#                 self.ball_detected = False
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 publish_and_log('[bad z]')
#                 return

#             # If already within collection distance -> collect
#             if self.ball_z <= self.ball_collection_distance:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[APPROACH_BALL -> COLLECT]')
#                 self.get_logger().info('[APPROACH_BALL] BALL REACHED! Starting collection.')
#                 self.state = "COLLECT_BALL"
#                 self.start_time = now
#                 self.movement_duration = 2.0
#                 # Prevent re-triggering while collecting
#                 self.ball_detected = False
#                 return

#             # Not reached: move forward while doing small angular corrections
#             # Angular correction proportional to error
#             if abs(self.ball_x) < self.center_deadzone:
#                 msg.angular.z = 0.0
#             else:
#                 msg.angular.z = - (self.angular_chase_multiplier * 0.5) * float(self.ball_x)

#             msg.linear.x = self.approach_speed

#             # Accumulate traveled distance using actual timer period
#             # Protect against missing start_time
#             if self.start_time is None:
#                 self.start_time = now
#             # integrate distance by timer_period (we call timer every timer_period)
#             self.original_position_distance += abs(self.approach_speed) * self.timer_period

#             self.get_logger().info(f'[APPROACH_BALL] Approaching: z={self.ball_z:.3f}m x={self.ball_x:.3f} dist_traveled={self.original_position_distance:.3f}m')
#             publish_and_log('[approaching]')
#             return

#         # ---------- STATE: COLLECT_BALL ----------
#         if self.state == "COLLECT_BALL":
#             elapsed = now - self.start_time if self.start_time is not None else 0.0
#             msg.linear.x = 0.0
#             msg.angular.z = 0.0
#             if elapsed >= self.movement_duration:
#                 self.get_logger().info('[COLLECT_BALL] Collection complete — returning to position.')
#                 self.state = "RETURN_TO_POSITION"
#                 self.return_start_time = now
#                 publish_and_log('[collected -> return]')
#                 return
#             else:
#                 # occasional collecting log
#                 if int(elapsed * 10) % 10 == 0:
#                     self.get_logger().info(f'[COLLECT_BALL] Collecting... {elapsed:.1f}/{self.movement_duration:.1f}s')
#                 publish_and_log('[collecting]')
#                 return

#         # ---------- STATE: RETURN_TO_POSITION ----------
#         if self.state == "RETURN_TO_POSITION":
#             elapsed = now - self.return_start_time if self.return_start_time else 0.0
#             distance_to_travel = self.original_position_distance
#             if distance_to_travel <= 0.0:
#                 self.get_logger().info('[RETURN] Nothing to return — go to detect')
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 publish_and_log('[return skip]')
#                 return

#             time_needed = distance_to_travel / max(1e-6, self.approach_speed)
#             if elapsed < time_needed:
#                 # back up
#                 msg.linear.x = -self.approach_speed
#                 msg.angular.z = 0.0
#                 remaining = max(0.0, distance_to_travel - elapsed * self.approach_speed)
#                 if int(elapsed * 10) % 10 == 0:
#                     self.get_logger().info(f'[RETURN] Backing up... {remaining:.2f}m remaining')
#                 publish_and_log('[returning]')
#                 return
#             else:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[return done]')
#                 self.get_logger().info('[RETURN] Returned to original position. Resuming detection.')
#                 # resume detection spins to look for next ball
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 self.spin_count = 0
#                 self.ball_detected = False
#                 return

#         # ---------- STATE: WAITING_BEFORE_TURN ----------
#         if self.state == "WAITING_BEFORE_TURN":
#             elapsed = now - self.start_time if self.start_time else 0.0
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[waiting before turn]')
#                 return
#             else:
#                 self.get_logger().info('[WAITING_BEFORE_TURN] Ready to turn')
#                 self.state = "TURNING"
#                 self.start_time = None
#                 return

#         # ---------- STATE: TURNING ----------
#         if self.state == "TURNING":
#             if self.start_time is None:
#                 self.start_time = now
#                 turn_angle_rad = math.radians(self.turn_angle)
#                 self.movement_duration = turn_angle_rad / max(1e-6, self.angular_speed)
#                 self.get_logger().info(f'[TURNING] Starting {self.turn_angle}° turn...')

#             elapsed = now - self.start_time
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = self.angular_speed
#                 publish_and_log('[turning]')
#                 return
#             else:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 publish_and_log('[turn complete]')
#                 self.side_count += 1
#                 if self.side_count >= 4:
#                     self.get_logger().info('SQUARE COMPLETED! Starting next square...')
#                     self.side_count = 0
#                 else:
#                     self.get_logger().info(f'[TURNING] Complete. Next side {self.side_count + 1}/4')
#                 self.state = "FORWARD"
#                 self.start_time = None
#                 return

#         # Default publish final cmd if no state matched
#         publish_and_log('[default end]')

# def main(args=None):
#     rclpy.init(args=args)
#     node = SquareGridWithCollection()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('\nShutting down gracefully...')
#         # stop robot
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         node.publisher_.publish(msg)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#################################################

# FROM SAV (NOV 10), goes in a square and rotates at each point

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point
# import time
# import math

# class SquareGridWithDetection(Node):
#     def __init__(self):
#         super().__init__('square_grid_with_detection')
        
#         # Publisher for robot velocity commands
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Subscriber for ball detection
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.ball_callback,
#             10)
        
#         # Parameters for square movement
#         self.declare_parameter("linear_speed", 0.2)  # m/s forward speed
#         self.declare_parameter("angular_speed", 0.5)  # rad/s turning speed
#         self.declare_parameter("square_side_length", 1.0)  # meters
#         self.declare_parameter("turn_angle", 90.0)  # degrees
#         self.declare_parameter("detection_spins", 3)  # Number of 360° spins
        
#         self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
#         self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
#         self.square_side_length = self.get_parameter('square_side_length').get_parameter_value().double_value
#         self.turn_angle = self.get_parameter('turn_angle').get_parameter_value().double_value
#         self.detection_spins = self.get_parameter('detection_spins').get_parameter_value().integer_value
        
#         # State machine variables
#         self.state = "FORWARD"  # States: FORWARD, WAITING, DETECT_BALL, TURNING
#         self.side_count = 0  # Track which side of square we're on (0-3)
#         self.spin_count = 0  # Track how many 360° spins completed
#         self.start_time = None
#         self.movement_duration = 0.0
        
#         # Ball detection variables
#         self.ball_detected = False
#         self.ball_position = None
        
#         # Timer for control loop
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         self.get_logger().info('Square Grid with Ball Detection Node Started!')
#         self.get_logger().info(f'Square side length: {self.square_side_length}m')
#         self.get_logger().info(f'Will perform {self.detection_spins} detection spins after each side')
        
#     def ball_callback(self, msg):
#         """Callback when ball is detected"""
#         self.ball_detected = True
#         self.ball_position = msg
#         self.get_logger().info(f'Ball detected at X: {msg.x:.3f}, Z: {msg.z:.3f}')
        
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
        
#         if self.state == "FORWARD":
#             if self.start_time is None:
#                 # Starting a new forward movement
#                 self.start_time = current_time
#                 self.movement_duration = self.square_side_length / self.linear_speed
#                 self.get_logger().info(f'Moving forward - Side {self.side_count + 1}/4')
            
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 # Continue moving forward
#                 msg.linear.x = self.linear_speed
#                 msg.angular.z = 0.0
#             else:
#                 # Finished moving forward, transition to ball detection
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.publisher_.publish(msg)
                
#                 self.state = "WAITING"
#                 self.start_time = current_time
#                 self.movement_duration = 0.5  # Wait 0.5 seconds before detecting
#                 self.spin_count = 0  # Reset spin counter
#                 self.ball_detected = False  # Reset ball detection flag
#                 self.get_logger().info('Forward movement complete, waiting before detection...')
                
#         elif self.state == "WAITING":
#             # Brief pause before next action
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#             else:
#                 # Transition to ball detection mode
#                 self.state = "DETECT_BALL"
#                 self.start_time = None
#                 self.get_logger().info('Entering ball detection mode...')
                
#         elif self.state == "DETECT_BALL":
#             if self.start_time is None:
#                 # Starting a new 360° spin
#                 self.start_time = current_time
#                 # Calculate time for one full rotation (360° = 2π radians)
#                 full_rotation_rad = 2 * math.pi
#                 self.movement_duration = full_rotation_rad / self.angular_speed
#                 self.get_logger().info(f'Detection spin {self.spin_count + 1}/{self.detection_spins}')
            
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 # Continue spinning to detect ball
#                 msg.linear.x = 0.0
#                 msg.angular.z = self.angular_speed  # Spin counterclockwise
#             else:
#                 # Finished one 360° spin
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.publisher_.publish(msg)
                
#                 self.spin_count += 1
                
#                 if self.spin_count >= self.detection_spins:
#                     # Completed all detection spins
#                     self.get_logger().info(f'Detection complete. Ball detected: {self.ball_detected}')
                    
#                     # Transition to WAITING before turning
#                     self.state = "WAITING_BEFORE_TURN"
#                     self.start_time = current_time
#                     self.movement_duration = 0.5
#                 else:
#                     # Do another detection spin
#                     self.start_time = None
                    
#         elif self.state == "WAITING_BEFORE_TURN":
#             # Brief pause before turning
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#             else:
#                 # Transition to turning
#                 self.state = "TURNING"
#                 self.start_time = None
                
#         elif self.state == "TURNING":
#             if self.start_time is None:
#                 # Starting a 90° turn
#                 self.start_time = current_time
#                 turn_angle_rad = math.radians(self.turn_angle)
#                 self.movement_duration = turn_angle_rad / self.angular_speed
#                 self.get_logger().info(f'Turning {self.turn_angle} degrees...')
            
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 # Continue turning
#                 msg.linear.x = 0.0
#                 msg.angular.z = self.angular_speed
#             else:
#                 # Finished turning
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.publisher_.publish(msg)
                
#                 self.side_count += 1
                
#                 if self.side_count >= 4:
#                     # Completed one full square
#                     self.get_logger().info('Square completed! Starting next square...')
#                     self.side_count = 0
                
#                 # Transition back to forward movement
#                 self.state = "FORWARD"
#                 self.start_time = None
        
#         self.publisher_.publish(msg)
    
#     def stop_robot(self):
#         """Helper method to stop the robot"""
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     square_grid = SquareGridWithDetection()
    
#     try:
#         rclpy.spin(square_grid)
#     except KeyboardInterrupt:
#         square_grid.get_logger().info('Shutting down...')
#         square_grid.stop_robot()
#     finally:
#         square_grid.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



####################################################

# FROM SAV (NOV 10), goes in a square

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time
# import math

# class SquareGrid(Node):
#     def __init__(self):
#         super().__init__('square_grid')
        
#         # Publisher for robot velocity commands
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Parameters for square movement
#         self.declare_parameter("linear_speed", 0.2)  # m/s forward speed
#         self.declare_parameter("angular_speed", 0.5)  # rad/s turning speed
#         self.declare_parameter("square_side_length", 1.0)  # meters
#         self.declare_parameter("turn_angle", 90.0)  # degrees
        
#         self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
#         self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
#         self.square_side_length = self.get_parameter('square_side_length').get_parameter_value().double_value
#         self.turn_angle = self.get_parameter('turn_angle').get_parameter_value().double_value
        
#         # State machine variables
#         self.state = "FORWARD"  # States: FORWARD, TURNING, WAITING
#         self.side_count = 0  # Track which side of square we're on (0-3)
#         self.start_time = None
#         self.movement_duration = 0.0
        
#         # Timer for control loop
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         self.get_logger().info('Square Grid Node Started!')
#         self.get_logger().info(f'Square side length: {self.square_side_length}m')
        
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
        
#         if self.state == "FORWARD":
#             if self.start_time is None:
#                 # Starting a new forward movement
#                 self.start_time = current_time
#                 self.movement_duration = self.square_side_length / self.linear_speed
#                 self.get_logger().info(f'Moving forward - Side {self.side_count + 1}/4')
            
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 # Continue moving forward
#                 msg.linear.x = self.linear_speed
#                 msg.angular.z = 0.0
#             else:
#                 # Finished moving forward, transition to turning
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.publisher_.publish(msg)
                
#                 self.state = "WAITING"
#                 self.start_time = current_time
#                 self.movement_duration = 0.5  # Wait 0.5 seconds before turning
#                 self.get_logger().info('Forward movement complete, waiting...')
                
#         elif self.state == "WAITING":
#             # Brief pause before turning
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#             else:
#                 # Transition to turning
#                 self.state = "TURNING"
#                 self.start_time = None
                
#         elif self.state == "TURNING":
#             if self.start_time is None:
#                 # Starting a new turn
#                 self.start_time = current_time
#                 turn_angle_rad = math.radians(self.turn_angle)
#                 self.movement_duration = turn_angle_rad / self.angular_speed
#                 self.get_logger().info(f'Turning {self.turn_angle} degrees...')
            
#             elapsed = current_time - self.start_time
            
#             if elapsed < self.movement_duration:
#                 # Continue turning (positive angular.z = counterclockwise)
#                 msg.linear.x = 0.0
#                 msg.angular.z = self.angular_speed
#             else:
#                 # Finished turning
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.publisher_.publish(msg)
                
#                 self.side_count += 1
                
#                 if self.side_count >= 4:
#                     # Completed one full square
#                     self.get_logger().info('Square completed! Starting next square...')
#                     self.side_count = 0
                
#                 # Transition back to forward movement
#                 self.state = "FORWARD"
#                 self.start_time = None
        
#         self.publisher_.publish(msg)
    
#     def stop_robot(self):
#         """Helper method to stop the robot"""
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     square_grid = SquareGrid()
    
#     try:
#         rclpy.spin(square_grid)
#     except KeyboardInterrupt:
#         square_grid.get_logger().info('Shutting down...')
#         square_grid.stop_robot()
#     finally:
#         square_grid.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#################################################################

# FROM ABBY: Using dead reckoning, robot goes in a square around the court
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Twist
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
        
#         # Velocity command publisher
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Ball-following parameters
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         self.declare_parameter("search_angular_speed", 0.5)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)
        
#         # Grid patrol parameters
#         self.declare_parameter("patrol_speed", 0.3)
#         self.declare_parameter("turn_speed", 0.8)
#         self.declare_parameter("move_duration", 3.33)  # Time to move 1 meter (1m / 0.3m/s = 3.33s)
#         self.declare_parameter("turn_duration", 2.0)  # Time to turn ~90 degrees
#         self.declare_parameter("return_speed", 0.3)  # Speed when returning to patrol path
#         self.declare_parameter("return_turn_speed", 0.8)  # Turn speed when returning
#         self.declare_parameter("position_tolerance", 0.15)  # How close to get to saved position (meters)
#         self.declare_parameter("angle_tolerance", 0.3)  # How close to match saved angle (radians, ~17 degrees)
        
#         # Get parameter values
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
#         self.patrol_speed = self.get_parameter('patrol_speed').get_parameter_value().double_value
#         self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
#         self.move_duration = self.get_parameter('move_duration').get_parameter_value().double_value
#         self.turn_duration = self.get_parameter('turn_duration').get_parameter_value().double_value
#         self.return_speed = self.get_parameter('return_speed').get_parameter_value().double_value
#         self.return_turn_speed = self.get_parameter('return_turn_speed').get_parameter_value().double_value
#         self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
#         # Timer for main control loop
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Ball tracking variables
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000
        
#         # Grid patrol state machine
#         # Patrol pattern: creates a square
#         self.patrol_sequence = [
#             'forward',      # Move forward
#             'turn_left',    # Turn left 90°
#             'forward',      # Move forward
#             'turn_left',    # Turn left 90°
#             'forward',      # Move forward
#             'turn_left',    # Turn left 90°
#             'forward',      # Move forward
#             'turn_left',    # Turn left 90° (back to start orientation)
#         ]
        
#         self.current_patrol_step = 0
#         self.action_start_time = time.time()
#         self.patrol_state = 'idle'  # 'idle', 'moving_forward', 'turning'
#         self.mode = 'patrol'  # 'chase', 'patrol', 'returning'
        
#         # Dead reckoning position tracking (approximate)
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0  # 0 = East, π/2 = North, π = West, -π/2 = South
#         self.last_update_time = time.time()
        
#         # Save position when interrupted by ball detection
#         self.saved_x = 0.0
#         self.saved_y = 0.0
#         self.saved_yaw = 0.0
#         self.saved_patrol_step = 0
#         self.saved_patrol_state = 'idle'
#         self.saved_action_start_time = time.time()
#         self.position_saved = False

#     def update_dead_reckoning(self, linear_vel, angular_vel):
#         """Update estimated position based on commanded velocities."""
#         current_time = time.time()
#         dt = current_time - self.last_update_time
#         self.last_update_time = current_time
        
#         # Update orientation
#         self.robot_yaw += angular_vel * dt
#         self.robot_yaw = self.normalize_angle(self.robot_yaw)
        
#         # Update position
#         self.robot_x += linear_vel * math.cos(self.robot_yaw) * dt
#         self.robot_y += linear_vel * math.sin(self.robot_yaw) * dt

#     def normalize_angle(self, angle):
#         """Normalize angle to [-pi, pi]."""
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle

#     def return_to_position(self):
#         """Navigate back to saved position and orientation."""
#         msg = Twist()
        
#         # Calculate distance and angle to saved position
#         dx = self.saved_x - self.robot_x
#         dy = self.saved_y - self.robot_y
#         distance = math.sqrt(dx**2 + dy**2)
#         angle_to_target = math.atan2(dy, dx)
#         angle_error = self.normalize_angle(angle_to_target - self.robot_yaw)
        
#         # Check if we're close enough to the position
#         if distance < self.position_tolerance:
#             # Now align orientation
#             orientation_error = self.normalize_angle(self.saved_yaw - self.robot_yaw)
            
#             if abs(orientation_error) < self.angle_tolerance:
#                 # Successfully returned! Resume patrol
#                 self.get_logger().info('[RETURN] ✓ Position and orientation restored! Resuming patrol.')
#                 self.mode = 'patrol'
#                 self.current_patrol_step = self.saved_patrol_step
#                 self.patrol_state = self.saved_patrol_state
#                 self.action_start_time = time.time()  # Reset action timer
#                 self.position_saved = False
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#             else:
#                 # Rotate to match saved orientation
#                 self.get_logger().info(f'[RETURN] Aligning orientation: {math.degrees(orientation_error):.1f}° off')
#                 msg.linear.x = 0.0
#                 turn_speed = max(0.2, min(self.return_turn_speed, abs(orientation_error) * 1.5))
#                 msg.angular.z = turn_speed if orientation_error > 0 else -turn_speed
#         else:
#             # Navigate back to position
#             if abs(angle_error) > 0.3:  # ~17 degrees
#                 # Turn towards target position
#                 self.get_logger().info(f'[RETURN] Turning towards saved position: {distance:.2f}m away')
#                 msg.linear.x = 0.0
#                 turn_speed = max(0.2, min(self.return_turn_speed, abs(angle_error) * 1.5))
#                 msg.angular.z = turn_speed if angle_error > 0 else -turn_speed
#             else:
#                 # Move towards target position
#                 self.get_logger().info(f'[RETURN] Moving to saved position: {distance:.2f}m remaining')
#                 msg.linear.x = self.return_speed
#                 msg.angular.z = 1.0 * angle_error  # Small correction
        
#         return msg

#     def grid_patrol(self):
#         """Execute grid-based patrol using timed movements."""
#         msg = Twist()
#         current_time = time.time()
#         elapsed_time = current_time - self.action_start_time
        
#         # Get current action from sequence
#         current_action = self.patrol_sequence[self.current_patrol_step]
        
#         if current_action == 'forward':
#             # Execute forward movement
#             if self.patrol_state == 'idle':
#                 self.patrol_state = 'moving_forward'
#                 self.action_start_time = current_time
#                 self.get_logger().info(f'[PATROL] Step {self.current_patrol_step + 1}/{len(self.patrol_sequence)}: Moving forward for {self.move_duration:.1f}s')
            
#             if self.patrol_state == 'moving_forward':
#                 if elapsed_time < self.move_duration:
#                     # Continue moving forward
#                     msg.linear.x = self.patrol_speed
#                     msg.angular.z = 0.0
#                 else:
#                     # Movement complete, advance to next step
#                     msg.linear.x = 0.0
#                     msg.angular.z = 0.0
#                     self.patrol_state = 'idle'
#                     self.current_patrol_step = (self.current_patrol_step + 1) % len(self.patrol_sequence)
#                     self.get_logger().info('[PATROL] Forward movement complete')
        
#         elif current_action == 'turn_left':
#             # Execute left turn
#             if self.patrol_state == 'idle':
#                 self.patrol_state = 'turning'
#                 self.action_start_time = current_time
#                 self.get_logger().info(f'[PATROL] Step {self.current_patrol_step + 1}/{len(self.patrol_sequence)}: Turning left for {self.turn_duration:.1f}s')
            
#             if self.patrol_state == 'turning':
#                 if elapsed_time < self.turn_duration:
#                     # Continue turning
#                     msg.linear.x = 0.0
#                     msg.angular.z = self.turn_speed
#                 else:
#                     # Turn complete, advance to next step
#                     msg.linear.x = 0.0
#                     msg.angular.z = 0.0
#                     self.patrol_state = 'idle'
#                     self.current_patrol_step = (self.current_patrol_step + 1) % len(self.patrol_sequence)
#                     self.get_logger().info('[PATROL] Turn complete')
        
#         elif current_action == 'turn_right':
#             # Execute right turn
#             if self.patrol_state == 'idle':
#                 self.patrol_state = 'turning'
#                 self.action_start_time = current_time
#                 self.get_logger().info(f'[PATROL] Step {self.current_patrol_step + 1}/{len(self.patrol_sequence)}: Turning right for {self.turn_duration:.1f}s')
            
#             if self.patrol_state == 'turning':
#                 if elapsed_time < self.turn_duration:
#                     # Continue turning
#                     msg.linear.x = 0.0
#                     msg.angular.z = -self.turn_speed
#                 else:
#                     # Turn complete, advance to next step
#                     msg.linear.x = 0.0
#                     msg.angular.z = 0.0
#                     self.patrol_state = 'idle'
#                     self.current_patrol_step = (self.current_patrol_step + 1) % len(self.patrol_sequence)
#                     self.get_logger().info('[PATROL] Turn complete')
        
#         return msg

#     def timer_callback(self):
#         msg = Twist()
        
#         # Check if ball was recently detected
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             # CHASE MODE: Ball detected
#             if self.mode != 'chase':
#                 self.get_logger().info('=== BALL DETECTED - SWITCHING TO CHASE MODE ===')
#                 # Save current position and patrol state
#                 self.saved_x = self.robot_x
#                 self.saved_y = self.robot_y
#                 self.saved_yaw = self.robot_yaw
#                 self.saved_patrol_step = self.current_patrol_step
#                 self.saved_patrol_state = self.patrol_state
#                 self.saved_action_start_time = self.action_start_time
#                 self.position_saved = True
#                 self.get_logger().info(f'[SAVE] Saved position: ({self.saved_x:.2f}, {self.saved_y:.2f}), yaw: {math.degrees(self.saved_yaw):.1f}°, step: {self.saved_patrol_step + 1}')
#                 self.mode = 'chase'
            
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
#             # Ball lost
#             if self.mode == 'chase' and self.position_saved:
#                 # Just lost the ball - switch to return mode
#                 self.get_logger().info('=== BALL LOST - RETURNING TO PATROL PATH ===')
#                 self.mode = 'returning'
            
#             if self.mode == 'returning':
#                 # RETURN MODE: Navigate back to saved position
#                 msg = self.return_to_position()
#             else:
#                 # PATROL MODE: Execute normal patrol
#                 if self.mode != 'patrol':
#                     self.get_logger().info('=== RESUMING PATROL MODE ===')
#                     self.mode = 'patrol'
                
#                 msg = self.grid_patrol()
        
#         # Update dead reckoning
#         self.update_dead_reckoning(msg.linear.x, msg.angular.z)
        
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

#########################

# FROM ABBY: Editing SAV AND ABBY (NOV 8) code
# Import ROS2 Python client library - allows us to use ROS2 features
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, Twist
# from nav_msgs.msg import Odometry
# import time
# import math

# class FollowBall(Node):
#     def __init__(self):
#         super().__init__('follow_ball')
        
#         # Subscriptions
#         self.subscription = self.create_subscription(
#             Point, '/detected_ball', self.listener_callback, 10)
#         self.odom_subscription = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10)
        
#         # Publisher for velocity commands
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Parameters for chase mode
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)

#         # Parameters for patrol mode
#         self.declare_parameter("patrol_speed", 0.3)
#         self.declare_parameter("patrol_angular_speed", 0.6)
#         self.declare_parameter("goal_tolerance", 0.3)
#         self.declare_parameter("angle_tolerance", 0.2)  # tighter turn tolerance

#         # Get parameter values
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
#         self.patrol_speed = self.get_parameter('patrol_speed').get_parameter_value().double_value
#         self.patrol_angular_speed = self.get_parameter('patrol_angular_speed').get_parameter_value().double_value
#         self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
#         # Timer (main control loop)
#         self.timer = self.create_timer(0.1, self.timer_callback)
        
#         # Ball tracking variables
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000  # start in patrol mode
        
#         # Patrol waypoints
#         self.patrol_points = [
#             {'x': 0.0, 'y': 0.0},
#             {'x': 1.0, 'y': 0.0},
#             {'x': 1.0, 'y': 1.0},
#             {'x': 0.0, 'y': 1.0}
#         ]
#         self.current_patrol_index = 0
#         self.waypoint_reached_time = 0.0
#         self.waypoint_cooldown = 2.0
        
#         # Robot pose
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0
#         self.odom_initialized = False
        
#         self.mode = 'patrol'

#     def odom_callback(self, msg):
#         """Update robot's current position from odometry."""
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y
        
#         orientation_q = msg.pose.pose.orientation
#         siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
#         cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
#         self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
#         self.odom_initialized = True

#     def normalize_angle(self, angle):
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle

#     def patrol_behavior(self):
#         """Navigate to patrol waypoints."""
#         if not self.odom_initialized:
#             self.get_logger().info('Waiting for odometry data...')
#             return Twist()
        
#         goal = self.patrol_points[self.current_patrol_index]
#         dx = goal['x'] - self.robot_x
#         dy = goal['y'] - self.robot_y
#         distance = math.sqrt(dx**2 + dy**2)
#         angle_to_goal = math.atan2(dy, dx)
#         angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)
        
#         msg = Twist()
#         time_since_waypoint = time.time() - self.waypoint_reached_time
#         can_switch_waypoint = time_since_waypoint > self.waypoint_cooldown
        
#         if distance < self.goal_tolerance and can_switch_waypoint:
#             self.get_logger().info(f'✓ Reached waypoint {self.current_patrol_index + 1}/4 at ({goal["x"]:.1f}, {goal["y"]:.1f})')
#             self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
#             self.waypoint_reached_time = time.time()
#             msg.linear.x = 0.0
#             msg.angular.z = 0.0
#             next_goal = self.patrol_points[self.current_patrol_index]
#             self.get_logger().info(f'→ Next target: waypoint {self.current_patrol_index + 1}/4 at ({next_goal["x"]:.1f}, {next_goal["y"]:.1f})')
#         else:
#             if abs(angle_error) > self.angle_tolerance:
#                 # Turn in place until facing waypoint
#                 turn_speed = min(self.patrol_angular_speed, abs(angle_error))
#                 msg.angular.z = turn_speed if angle_error > 0 else -turn_speed
#                 msg.linear.x = 0.0
#                 self.get_logger().info(f'↻ Turning to waypoint {self.current_patrol_index + 1}/4 | Angle error: {math.degrees(angle_error):.1f}° | Distance: {distance:.2f}m')
#             else:
#                 # Move toward waypoint with small steering correction
#                 msg.linear.x = self.patrol_speed
#                 msg.angular.z = 0.8 * angle_error  # gentler steering
#                 self.get_logger().info(f'→ Moving to waypoint {self.current_patrol_index + 1}/4 | Distance: {distance:.2f}m | Angle: {math.degrees(angle_error):.1f}°')
        
#         return msg

#     def chase_behavior(self):
#         """Follow the detected ball."""
#         msg = Twist()
#         self.get_logger().info('TARGET FOUND!!!')
#         self.get_logger().info(f'Target X: {self.target_val:.3f}, Dist: {self.target_dist:.3f}')
        
#         if abs(self.target_val) < self.center_deadzone:
#             msg.angular.z = 0.0
#             self.get_logger().info('Ball centered - going straight!')
#         else:
#             msg.angular.z = -self.angular_chase_multiplier * self.target_val
#             self.get_logger().info(f'Adjusting angle: {msg.angular.z:.3f}')
        
#         msg.linear.x = self.forward_chase_speed
#         return msg

#     def timer_callback(self):
#         """Main control loop - switches between chase and patrol modes."""
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.mode = 'chase'
#             msg = self.chase_behavior()
#         else:
#             self.mode = 'patrol'
#             msg = self.patrol_behavior()
        
#         self.publisher_.publish(msg)

#     def listener_callback(self, msg):
#         """Update ball position when detected."""
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1 - f)
#         self.target_dist = self.target_dist * f + msg.z * (1 - f)
#         self.lastrcvtime = time.time()


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# SAV AND ABBY (NOV 7)
# Import ROS2 Python client library - allows us to use ROS2 features
# import rclpy
# # Import the base Node class - our robot controller will inherit from this
# from rclpy.node import Node
# # Import message types: Point (ball position), Twist (velocity commands)
# from geometry_msgs.msg import Point, Twist
# # Import Odometry message - tells us robot's position and orientation
# from nav_msgs.msg import Odometry
# # Import time module - used to track when we last saw the ball
# import time
# # Import math module - used for trigonometry and angle calculations
# import math

# # Define our main class - it inherits from Node (makes it a ROS2 node)
# class FollowBall(Node):
#     # Constructor - runs when we create a FollowBall object
#     def __init__(self):
#         # Call the parent class (Node) constructor with our node's name
#         super().__init__('follow_ball')
        
#         # Create a subscription to listen for ball detections
#         # This listens to the '/detected_ball' topic
#         self.subscription = self.create_subscription(
#             Point,                    # Message type we expect (x, y, z coordinates)
#             '/detected_ball',         # Topic name where ball positions are published
#             self.listener_callback,   # Function to call when we receive a message
#             10)                       # Queue size - stores up to 10 messages if we're busy
        
#         # Create a subscription to listen for robot position updates
#         # This tells us where the robot is and which direction it's facing
#         self.odom_subscription = self.create_subscription(
#             Odometry,                 # Message type containing position and orientation
#             '/odom',        # Topic name where odometry is published
#             self.odom_callback,       # Function to call when we get position updates
#             10)                       # Queue size
        
#         # Create a publisher to send velocity commands to the robot
#         # This is how we make the robot move
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Declare parameters for ball-following behavior
#         # These can be changed without recompiling the code
        
#         # How long to wait (in seconds) before deciding we've lost the ball
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         # How aggressively to turn when chasing (higher = sharper turns)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         # How fast to move forward when chasing the ball (m/s)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         # Maximum ball size threshold (not currently used in this version)
#         self.declare_parameter("max_size_thresh", 0.1)
#         # Smoothing filter value (0-1, higher = more smoothing)
#         self.declare_parameter("filter_value", 0.9)
#         # Deadzone for "centered" - if ball is this close to center, go straight
#         self.declare_parameter("center_deadzone", 0.05)
        
#         # Declare parameters for patrol behavior
#         # How fast to move when patrolling (m/s)
#         self.declare_parameter("patrol_speed", 0.3)
#         # How fast to turn when navigating to waypoints (rad/s)
#         self.declare_parameter("patrol_angular_speed", 0.6)
#         # How close we need to get to a waypoint before moving to the next one (meters)
#         self.declare_parameter("goal_tolerance", 0.3)
#         # Minimum angle alignment before moving forward (radians)
#         # self.declare_parameter("angle_tolerance", 0.5) # TEST 0.2)
#         self.declare_parameter("angle_tolerance", 0.2)
        
#         # Get the actual parameter values and store them in instance variables
#         # .get_parameter() retrieves the parameter
#         # .get_parameter_value() gets its value
#         # .double_value converts it to a floating-point number
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
#         self.patrol_speed = self.get_parameter('patrol_speed').get_parameter_value().double_value
#         self.patrol_angular_speed = self.get_parameter('patrol_angular_speed').get_parameter_value().double_value
#         self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
#         # Set up a timer that calls timer_callback every 0.1 seconds (10 Hz)
#         # This is our main control loop
#         timer_period = 0.1  # seconds (100 milliseconds)
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Initialize variables for tracking the ball
#         # target_val: horizontal position of ball (-1 left, 0 center, +1 right)
#         self.target_val = 0.0
#         # target_dist: distance to the ball (z coordinate)
#         self.target_dist = 0.0
#         # lastrcvtime: timestamp of when we last saw the ball
#         # Initialize to far in the past so robot starts in patrol mode
#         self.lastrcvtime = time.time() - 10000
        
#         # Define patrol waypoints - 4 corners of a 2m x 2m square
#         # Each point is a dictionary with x and y coordinates
#         self.patrol_points = [
#             {'x': 0.0, 'y': 0.0},    # Bottom-left corner (starting point)
#             {'x': 1.0, 'y': 0.0},    # Bottom-right corner
#             {'x': 1.0, 'y': 1.0},    # Top-right corner
#             {'x': 0.0, 'y': 1.0}     # Top-left corner
#         ]
#         # Track which waypoint we're currently heading toward (0-3)
#         self.current_patrol_index = 0
#         # Add a cooldown to prevent rapid waypoint switching
#         self.waypoint_reached_time = 0.0
#         self.waypoint_cooldown = 2.0  # Wait 2 seconds before considering next waypoint
        
#         # Initialize variables for tracking robot's current position
#         self.robot_x = 0.0           # Robot's x position in meters
#         self.robot_y = 0.0           # Robot's y position in meters
#         self.robot_yaw = 0.0         # Robot's rotation angle in radians (which way it's facing)
#         self.odom_initialized = False # Flag: have we received odometry data yet?
        
#         # Track what the robot is currently doing
#         self.mode = 'patrol'  # Can be 'chase' (following ball) or 'patrol' (searching)

#     # Callback function - runs every time we receive odometry data
#     def odom_callback(self, msg):
#         """Update robot's current position from odometry."""
#         # Extract x and y position from the message
#         self.robot_x = msg.pose.pose.position.x # * 10
#         self.robot_y = msg.pose.pose.position.y # * 10
        
#         # Convert quaternion orientation to yaw angle (rotation around z-axis)
#         # Quaternions are a complex way to represent 3D rotations (x, y, z, w)
#         # We need to convert to Euler angles to get simple rotation in radians
#         orientation_q = msg.pose.pose.orientation
#         # Formula to convert quaternion to yaw angle
#         siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
#         cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
#         # atan2 gives us the angle in radians
#         self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
#         # Mark that we've successfully received odometry data
#         self.odom_initialized = True

#     # Helper function to keep angles in the range [-π, π] (or [-180°, 180°])
#     def normalize_angle(self, angle):
#         """Normalize angle to [-pi, pi]."""
#         # If angle is greater than π (180°), subtract 2π (360°) until it's in range
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         # If angle is less than -π (-180°), add 2π (360°) until it's in range
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle

#     # Function that controls robot movement during patrol mode
#     def patrol_behavior(self):
#         """Navigate to patrol waypoints."""
#         # If we haven't received position data yet, can't navigate
#         if not self.odom_initialized:
#             self.get_logger().info('Waiting for odometry data...')
#             return Twist()  # Return empty command (no movement)
        
#         # Get the current target waypoint from our patrol list
#         goal = self.patrol_points[self.current_patrol_index]
        
#         # Calculate the difference between goal and current position
#         dx = goal['x'] - self.robot_x  # Horizontal difference
#         dy = goal['y'] - self.robot_y  # Vertical difference
#         # Use Pythagorean theorem to find straight-line distance to goal
#         distance = math.sqrt(dx**2 + dy**2)
#         # Calculate what angle we need to face to point at the goal
#         angle_to_goal = math.atan2(dy, dx)
#         # Calculate the difference between where we're facing and where we need to face
#         angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)
        
#         # Create an empty Twist message to fill with velocity commands
#         msg = Twist()
        
#         # Check if enough time has passed since reaching last waypoint (prevents oscillation)
#         time_since_waypoint = time.time() - self.waypoint_reached_time
#         can_switch_waypoint = time_since_waypoint > self.waypoint_cooldown
        
#         # Check if we're close enough to the waypoint (within tolerance)
#         if distance < self.goal_tolerance and can_switch_waypoint:
#             # We've reached this waypoint! Move to the next one
#             self.get_logger().info(f'✓ Reached waypoint {self.current_patrol_index + 1}/4 at ({goal["x"]:.1f}, {goal["y"]:.1f})')
#             # Move to next waypoint, wrapping around to 0 after reaching waypoint 3
#             # The % operator gives remainder, so (3+1)%4 = 0
#             self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
#             self.waypoint_reached_time = time.time()  # Record when we reached it
#             # Stop moving while we transition
#             msg.linear.x = 0.0   # No forward movement
#             msg.angular.z = 0.0  # No rotation
            
#             next_goal = self.patrol_points[self.current_patrol_index]
#             self.get_logger().info(f'→ Next target: waypoint {self.current_patrol_index + 1}/4 at ({next_goal["x"]:.1f}, {next_goal["y"]:.1f})')
#         else:
#             # We haven't reached the waypoint yet - navigate toward it
#             # Check if we're facing the wrong direction
#             if abs(angle_error) > self.angle_tolerance:
#                 # Turn in place without moving forward
#                 # Use proportional control for smoother turns
#                 turn_speed = max(0.3, min(self.patrol_angular_speed, abs(angle_error) * 2.0))
#                 msg.angular.z = turn_speed if angle_error > 0 else -turn_speed
#                 msg.linear.x = 0.0  # Don't move forward while turning
#                 # Log what we're doing
#                 self.get_logger().info(f'↻ Turning to waypoint {self.current_patrol_index + 1}/4 | Angle error: {math.degrees(angle_error):.1f}° | Distance: {distance:.2f}m')
#             else:
#                 # We're facing roughly the right direction - move forward
#                 msg.linear.x = self.patrol_speed  # Move at patrol speed
#                 # Apply small correction to stay on course (proportional control)
#                 # msg.angular.z = 1.5 * angle_error  # Gentle steering adjustment
#                 # Apply a gentler, capped proportional correction to avoid oscillation
#                 ang = 0.8 * angle_error
#                 # cap angular velocity to configured patrol angular speed
#                 ang = max(-self.patrol_angular_speed, min(self.patrol_angular_speed, ang))
#                 msg.angular.z = ang

#                 self.get_logger().info(f'→ Moving to waypoint {self.current_patrol_index + 1}/4 | Distance: {distance:.2f}m | Angle: {math.degrees(angle_error):.1f}°')
        
#         # Return the velocity command we've created
#         return msg

#     # Function that controls robot movement when chasing a ball
#     def chase_behavior(self):
#         """Follow the detected ball."""
#         # Create empty velocity command
#         msg = Twist()
#         # Log that we found the ball
#         self.get_logger().info('TARGET FOUND!!!')
#         self.get_logger().info(f'Target X: {self.target_val:.3f}, Dist: {self.target_dist:.3f}')
        
#         # Check if ball is centered enough (within deadzone)
#         # target_val is negative when ball is left, positive when right
#         if abs(self.target_val) < self.center_deadzone:
#             # Ball is centered - drive straight toward it
#             msg.angular.z = 0.0  # No turning
#             self.get_logger().info('Ball centered - going straight!')
#         else:
#             # Ball is off to the side - turn to face it
#             # Negative sign because positive target_val (ball on right) needs negative angular.z (turn right)
#             msg.angular.z = -self.angular_chase_multiplier * self.target_val
#             self.get_logger().info(f'Adjusting angle: {msg.angular.z:.3f}')
        
#         # Always move forward when chasing
#         msg.linear.x = self.forward_chase_speed
#         # Return the velocity command
#         return msg

#     # Main control loop - runs every 0.1 seconds (called by the timer)
#     def timer_callback(self):
#         """Main control loop - switches between chase and patrol modes."""
#         # Check how long it's been since we last saw the ball
#         # If recent enough, enter chase mode
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.mode = 'chase'  # Switch to chase mode
#             msg = self.chase_behavior()  # Get chase commands
#         else:
#             # Ball hasn't been seen recently - enter patrol mode
#             self.mode = 'patrol'  # Switch to patrol mode
#             msg = self.patrol_behavior()  # Get patrol commands
        
#         # Send the velocity command to the robot
#         self.publisher_.publish(msg)

#     # Callback function - runs every time we receive a ball detection message
#     def listener_callback(self, msg):
#         """Update ball position when detected."""
#         # Get the filter value (how much to smooth the data)
#         f = self.filter_value
#         # Apply exponential smoothing filter to reduce noise
#         # New value = (old value × filter) + (new measurement × (1-filter))
#         # With f=0.9: 90% old value, 10% new measurement (very smooth)
#         self.target_val = self.target_val * f + msg.x * (1 - f)  # Horizontal position
#         self.target_dist = self.target_dist * f + msg.z * (1 - f)  # Distance
#         # Record the current time so we know when we last saw the ball
#         self.lastrcvtime = time.time()


# # Main function - entry point of the program
# def main(args=None):
#     # Initialize the ROS2 Python client library
#     rclpy.init(args=args)
#     # Create an instance of our FollowBall node
#     follow_ball = FollowBall()
#     # Keep the node running and processing callbacks until interrupted
#     # This handles all subscriptions, timers, etc.
#     rclpy.spin(follow_ball)
#     # Clean up and destroy the node when we're done
#     follow_ball.destroy_node()
#     # Shut down the ROS2 Python client library
#     rclpy.shutdown()


# # Standard Python idiom - only run main() if this file is executed directly
# # (not if it's imported as a module)
# if __name__ == '__main__':
#     main()





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
        
#         # Odometry subscription for position tracking
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10)
        
#         self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
#         # Ball-following parameters
#         self.declare_parameter("rcv_timeout_secs", 1.0)
#         self.declare_parameter("angular_chase_multiplier", 0.7)
#         self.declare_parameter("forward_chase_speed", 0.2)
#         self.declare_parameter("max_size_thresh", 0.1)
#         self.declare_parameter("filter_value", 0.9)
#         self.declare_parameter("center_deadzone", 0.05)
        
#         # Patrol parameters
#         # self.declare_parameter("patrol_speed", 0.2)
#         # self.declare_parameter("patrol_angular_speed", 0.5)
#         # self.declare_parameter("goal_tolerance", 0.2)  # meters
#         self.declare_parameter("patrol_speed", 0.3)           # Increased from 0.2
#         self.declare_parameter("patrol_angular_speed", 0.6)   # Increased from 0.5
#         self.declare_parameter("goal_tolerance", 0.3)         # Increased from 0.2
#         self.declare_parameter("angle_tolerance", 0.2)        # NEW parameter added
        
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
#         self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
#         self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
#         self.center_deadzone = self.get_parameter('center_deadzone').get_parameter_value().double_value
#         self.patrol_speed = self.get_parameter('patrol_speed').get_parameter_value().double_value
#         self.patrol_angular_speed = self.get_parameter('patrol_angular_speed').get_parameter_value().double_value
#         # self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
#         self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value  # NEW
        
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Ball tracking variables
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000
        
#         # Patrol waypoints (2 meters apart in a square pattern)
#         self.patrol_points = [
#             {'x': 0.0, 'y': 0.0},
#             {'x': 2.0, 'y': 0.0},
#             {'x': 2.0, 'y': 2.0},
#             {'x': 0.0, 'y': 2.0}
#         ]
#         # self.current_patrol_index = 0
#         self.current_patrol_index = 0
#         # Add a cooldown to prevent rapid waypoint switching
#         self.waypoint_reached_time = 0.0
#         self.waypoint_cooldown = 2.0  # Wait 2 seconds before considering next waypoint
        
#         # Robot position and orientation
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0
#         self.odom_initialized = False
        
#         # State tracking
#         self.mode = 'patrol'  # 'chase' or 'patrol'

#     def odom_callback(self, msg):
#         """Update robot's current position from odometry."""
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y
        
#         # Convert quaternion to yaw
#         orientation_q = msg.pose.pose.orientation
#         siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
#         cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
#         self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
#         self.odom_initialized = True

#     def normalize_angle(self, angle):
#         """Normalize angle to [-pi, pi]."""
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle

#     def patrol_behavior(self):
#         # Calculate distance and angle to goal
#         distance = math.sqrt(dx**2 + dy**2)
#         angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)
        
#         msg = Twist()
        
#         # NEW: Check if enough time has passed since reaching last waypoint
#         time_since_waypoint = time.time() - self.waypoint_reached_time
#         can_switch_waypoint = time_since_waypoint > self.waypoint_cooldown
        
#         # Check if we're close enough AND cooldown has passed
#         if distance < self.goal_tolerance and can_switch_waypoint:  # CHANGED
#             self.get_logger().info(f'✓ Reached waypoint {self.current_patrol_index + 1}/4 at ({goal["x"]:.1f}, {goal["y"]:.1f})')
#             self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
#             self.waypoint_reached_time = time.time()  # NEW: Record time
#             msg.linear.x = 0.0
#             msg.angular.z = 0.0
            
#             # NEW: Log next target
#             next_goal = self.patrol_points[self.current_patrol_index]
#             self.get_logger().info(f'→ Next target: waypoint {self.current_patrol_index + 1}/4 at ({next_goal["x"]:.1f}, {next_goal["y"]:.1f})')
#         else:
#             # Turn towards goal if needed
#             if abs(angle_error) > self.angle_tolerance:  # CHANGED: Use parameter instead of fixed 0.1
#                 # NEW: Proportional turn speed
#                 turn_speed = max(0.3, min(self.patrol_angular_speed, abs(angle_error) * 2.0))
#                 msg.angular.z = turn_speed if angle_error > 0 else -turn_speed
#                 msg.linear.x = 0.0
#                 # NEW: Better logging with symbols
#                 self.get_logger().info(f'↻ Turning to waypoint {self.current_patrol_index + 1}/4 | Angle error: {math.degrees(angle_error):.1f}° | Distance: {distance:.2f}m')
#             else:
#                 msg.linear.x = self.patrol_speed
#                 msg.angular.z = 1.5 * angle_error  # CHANGED: Increased from 0.3 to 1.5
#                 # NEW: Better logging
#                 self.get_logger().info(f'→ Moving to waypoint {self.current_patrol_index + 1}/4 | Distance: {distance:.2f}m | Angle: {math.degrees(angle_error):.1f}°')
        
#         return msg

#     def chase_behavior(self):
#         """Follow the detected ball."""
#         msg = Twist()
#         self.get_logger().info('TARGET FOUND!!!')
#         self.get_logger().info(f'Target X: {self.target_val:.3f}, Dist: {self.target_dist:.3f}')
        
#         # Apply deadzone - if ball is centered enough, don't rotate
#         if abs(self.target_val) < self.center_deadzone:
#             msg.angular.z = 0.0
#             self.get_logger().info('Ball centered - going straight!')
#         else:
#             msg.angular.z = -self.angular_chase_multiplier * self.target_val
#             self.get_logger().info(f'Adjusting angle: {msg.angular.z:.3f}')
        
#         msg.linear.x = self.forward_chase_speed
#         return msg

#     def timer_callback(self):
#         """Main control loop - switches between chase and patrol modes."""
#         # Check if we recently detected a ball
#         if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
#             self.mode = 'chase'
#             msg = self.chase_behavior()
#         else:
#             self.mode = 'patrol'
#             msg = self.patrol_behavior()
        
#         self.publisher_.publish(msg)

#     def listener_callback(self, msg):
#         """Update ball position when detected."""
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1 - f)
#         self.target_dist = self.target_dist * f + msg.z * (1 - f)
#         self.lastrcvtime = time.time()


# def main(args=None):
#     rclpy.init(args=args)
#     follow_ball = FollowBall()
#     rclpy.spin(follow_ball)
#     follow_ball.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#######################################################

# FROM SAV: NOV 5

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, Twist
# from nav_msgs.msg import Odometry
# import time
# import math


# class FollowBall(Node):
#     def __init__(self):
#         super().__init__('follow_ball')
       
#         # Subscriptions
#         self.create_subscription(Point, '/detected_ball', self.listener_callback, 10)
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
       
#         # Publisher
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
       
#         # Parameters
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('rcv_timeout_secs', 1.0),
#                 ('angular_chase_multiplier', 0.7),
#                 ('forward_chase_speed', 0.1),
#                 ('search_angular_speed', 0.4),
#                 ('max_size_thresh', 0.1),
#                 ('filter_value', 0.9),
#                 ('enable_grid_nav', True),
#                 ('grid_linear_speed', 0.15),
#                 ('grid_angular_speed', 0.4),
#                 ('position_tolerance', 0.20),
#                 ('angle_tolerance', 0.10),
#                 ('ball_lost_grid_timeout', 5.0),
#                 ('rotation_duration', 12.0),
#                 ('angle_kp', 1.0),
#                 ('grid_x_limit', 4.0),
#                 ('grid_y_limit', 4.0)
#             ]
#         )

#         # Get parameters
#         self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').value
#         self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').value
#         self.forward_chase_speed = self.get_parameter('forward_chase_speed').value
#         self.search_angular_speed = self.get_parameter('search_angular_speed').value
#         self.filter_value = self.get_parameter('filter_value').value
#         self.enable_grid_nav = self.get_parameter('enable_grid_nav').value
#         self.grid_linear_speed = self.get_parameter('grid_linear_speed').value
#         self.grid_angular_speed = self.get_parameter('grid_angular_speed').value
#         self.position_tolerance = self.get_parameter('position_tolerance').value
#         self.angle_tolerance = self.get_parameter('angle_tolerance').value
#         self.ball_lost_grid_timeout = self.get_parameter('ball_lost_grid_timeout').value
#         self.rotation_duration = self.get_parameter('rotation_duration').value
#         self.angle_kp = self.get_parameter('angle_kp').value
#         self.grid_x_limit = self.get_parameter('grid_x_limit').value
#         self.grid_y_limit = self.get_parameter('grid_y_limit').value

#         # Grid path (closed loop)
#         self.grid_points = [
#             (1.0, 1.0),
#             (1.0, 3.0),
#             (3.0, 3.0),
#             (3.0, 1.0),
#             (1.0, 1.0),
#             (0.0, 0.0)
#         ]
#         self.current_waypoint_index = 0

#         # Robot state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.odom_received = False

#         # Ball tracking
#         self.target_val = 0.0
#         self.target_dist = 0.0
#         self.lastrcvtime = time.time() - 10000

#         # Mode tracking
#         self.current_mode = 'SEARCH'
#         self.is_rotating = False
#         self.rotation_start_time = 0.0

#         # Timer
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info('✅ Follow Ball + Grid Navigation Node Started')

#     # --- Odometry ---
#     def odom_callback(self, msg):
#         """Extract pose (x, y, yaw) from odometry."""
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y

#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#         self.odom_received = True

#     # --- Timer loop ---
#     def timer_callback(self):
#         msg = Twist()
#         current_time = time.time()
#         time_since_ball = current_time - self.lastrcvtime

#         # Safety: stop if out of bounds
#         if abs(self.current_x) > self.grid_x_limit or abs(self.current_y) > self.grid_y_limit:
#             self.get_logger().warn(f'⚠️ Out of grid bounds! (x={self.current_x:.2f}, y={self.current_y:.2f}) — stopping.')
#             self.publisher_.publish(Twist())
#             return

#         # --- 360° rotation mode ---
#         if self.is_rotating:
#             elapsed = current_time - self.rotation_start_time
#             if time_since_ball < self.rcv_timeout_secs:
#                 self.get_logger().info('🎾 Ball detected during rotation → FOLLOW mode')
#                 self.is_rotating = False
#                 self.current_mode = 'FOLLOW'
#             elif elapsed >= self.rotation_duration:
#                 self.get_logger().info('✅ Rotation complete, moving to next waypoint')
#                 self.is_rotating = False
#                 self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.grid_points)
#             else:
#                 msg.angular.z = self.grid_angular_speed
#                 self.current_mode = 'ROTATING'

#         # --- Follow mode ---
#         elif time_since_ball < self.rcv_timeout_secs:
#             if self.current_mode != 'FOLLOW':
#                 self.get_logger().info('🎾 FOLLOW BALL MODE')
#             msg.linear.x = self.forward_chase_speed
#             msg.angular.z = -self.angular_chase_multiplier * self.target_val
#             self.current_mode = 'FOLLOW'

#         # --- Search mode ---
#         elif time_since_ball < self.ball_lost_grid_timeout:
#             if self.current_mode != 'SEARCH':
#                 self.get_logger().info('🔍 SEARCH MODE (spinning)')
#             msg.angular.z = self.search_angular_speed
#             self.current_mode = 'SEARCH'

#         # --- Grid navigation ---
#         elif self.enable_grid_nav and self.odom_received:
#             if self.current_mode != 'GRID':
#                 self.get_logger().info('🧭 GRID NAVIGATION MODE')
#             msg = self.navigate_grid()
#             self.current_mode = 'GRID'

#         self.publisher_.publish(msg)

#     # --- Grid Navigation Controller ---
#     def navigate_grid(self):
#         msg = Twist()
#         target_x, target_y = self.grid_points[self.current_waypoint_index]

#         dx = target_x - self.current_x
#         dy = target_y - self.current_y
#         distance = math.hypot(dx, dy)
#         target_angle = math.atan2(dy, dx)

#         angle_error = math.atan2(math.sin(target_angle - self.current_yaw), math.cos(target_angle - self.current_yaw))

#         if distance < self.position_tolerance:
#             self.get_logger().info(f'✅ Reached waypoint {self.current_waypoint_index}: ({target_x:.1f}, {target_y:.1f})')
#             self.is_rotating = True
#             self.rotation_start_time = time.time()
#             msg.angular.z = self.grid_angular_speed
#             return msg

#         if abs(angle_error) > self.angle_tolerance:
#             msg.angular.z = self.angle_kp * angle_error
#             # TEST self.get_logger().info(f'↻ Rotating to face waypoint (error={math.degrees(angle_error):.1f}°)')
#         else:
#             msg.linear.x = self.grid_linear_speed
#             msg.angular.z = 0.5 * angle_error
#             # TEST self.get_logger().info(f'➡ Moving to waypoint {self.current_waypoint_index} (dist={distance:.2f} m)')

#         return msg

#     # --- Ball Detection Callback ---
#     def listener_callback(self, msg):

#         # if listener_callback(self, msg):
#         if(msg.z == 0.0 and msg.x == 0.0):
#             return
                
#         f = self.filter_value
#         self.target_val = self.target_val * f + msg.x * (1 - f)
#         self.target_dist = self.target_dist * f + msg.z * (1 - f)
#         self.lastrcvtime = time.time()


# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowBall()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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


