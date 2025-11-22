#!/usr/bin/env python3

"""
Pedestrian Safety Bridge Node
Sits between perception and pure pursuit control
Modifies vehicle behavior based on pedestrian detection and behavior
"""

import rclpy
from rclpy.node import Node
import math

# Standard messages
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Point

# Placeholder for perception output, need to replace with actual perception message types
class PedestrianState:
    """Placeholder for pedestrian state from perception"""
    def __init__(self):
        self.detected = False
        self.distance = 0.0  # meters from vehicle
        self.lateral_distance = 0.0  # meters from road centerline
        self.velocity_x = 0.0  # m/s toward/away from road
        self.velocity_y = 0.0  # m/s parallel to road
        self.behavior = "NONE"  # PARALLEL, APPROACHING, STOPPED, CROSSING
      """
      /perception/pedestrian_detected  (Bool)
      - True if pedestrian detected

      /perception/pedestrian_distance  (Float64)
        - Distance in meters from vehicle
      
      /perception/pedestrian_behavior  (String)
        - Values: "PARALLEL", "APPROACHING", "STOPPED", "CROSSING", "NONE"
      
      /perception/pedestrian_position  (Point)
        - x: velocity toward road (m/s)
        - y: lateral distance from centerline (m)
        - z: (unused)
      
      /vehicle_speed  (Float64)
        - Current vehicle speed (from pacmod or pure pursuit)
      """


class PedestrianSafetyBridge(Node):
    def __init__(self):
        super().__init__('pedestrian_safety_bridge')
        
        # ============ PARAMETERS ============
        # Vehicle parameters
        self.declare_parameter('vehicle_mass', 600.0)  # kg
        self.declare_parameter('max_decel', 3.0)  # m/s^2 (braking)
        self.declare_parameter('emergency_decel', 5.0)  # m/s^2 (emergency)
        self.declare_parameter('min_detection_range', 2.0)  # meters (cam blind spot)
        self.declare_parameter('normal_speed', 2.0)  # m/s (default pure pursuit speed)
        
        # Safety thresholds
        self.declare_parameter('slow_zone_distance', 10.0)  # Start slowing at 10m
        self.declare_parameter('stop_zone_distance', 5.0)  # Stop at 5m
        self.declare_parameter('road_width', 3.0)  # meters
        self.declare_parameter('safety_margin', 1.0)  # extra meters for safety
        
        # Speed targets
        self.declare_parameter('slow_speed_factor', 0.5)  # 50% of normal speed when slowing
        
        # Get parameters
        self.vehicle_mass = self.get_parameter('vehicle_mass').value
        self.max_decel = self.get_parameter('max_decel').value
        self.emergency_decel = self.get_parameter('emergency_decel').value
        self.min_detection_range = self.get_parameter('min_detection_range').value
        self.normal_speed = self.get_parameter('normal_speed').value
        self.slow_zone_distance = self.get_parameter('slow_zone_distance').value
        self.stop_zone_distance = self.get_parameter('stop_zone_distance').value
        self.road_width = self.get_parameter('road_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.slow_speed_factor = self.get_parameter('slow_speed_factor').value
        
        # ============ STATE VARIABLES ============
        self.current_speed = 0.0
        self.pedestrian_detected = False
        self.pedestrian_distance = 100.0  
        self.pedestrian_behavior = "NONE"
        self.pedestrian_lateral_pos = 0.0
        self.pedestrian_vel_toward_road = 0.0
        
        self.current_action = "NORMAL"  # NORMAL, SLOW, STOP
        self.target_speed = self.normal_speed
        
        # Subscribe to perception outputs (PLACEHOLDER TOPICS - need to replace with actual)
        self.create_subscription(
            Bool, 
            '/perception/pedestrian_detected', 
            self.pedestrian_detected_callback, 
            10
        )
        self.create_subscription(
            Float64, 
            '/perception/pedestrian_distance', 
            self.pedestrian_distance_callback, 
            10
        )
        self.create_subscription(
            String, 
            '/perception/pedestrian_behavior', 
            self.pedestrian_behavior_callback, 
            10
        )
        self.create_subscription(
            Point, 
            '/perception/pedestrian_position', 
            self.pedestrian_position_callback, 
            10
        )
        
        # Subscribe to vehicle speed (from pure pursuit or pacmod)
        self.create_subscription(
            Float64,
            '/vehicle_speed',  # Or /pacmod/vehicle_speed_rpt
            self.vehicle_speed_callback,
            10
        )
        
        # Publish modified speed command to pure pursuit
        self.speed_override_pub = self.create_publisher(
            Float64, 
            '/pure_pursuit/speed_override', 
            10
        )
        
        # Publish brake override (emergency stop)
        self.brake_override_pub = self.create_publisher(
            Bool, 
            '/pure_pursuit/emergency_brake', 
            10
        )
        
        # Publish current action for logging/debugging
        self.action_pub = self.create_publisher(
            String,
            '/safety_bridge/current_action',
            10
        )
        
        # Publish safety status
        self.safety_status_pub = self.create_publisher(
            String,
            '/safety_bridge/status',
            10
        )
      
        # 20 Hz
        self.timer = self.create_timer(0.05, self.safety_logic_callback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Pedestrian Safety Bridge Node Started')
        self.get_logger().info(f'Normal speed: {self.normal_speed} m/s')
        self.get_logger().info(f'Slow zone: {self.slow_zone_distance} m')
        self.get_logger().info(f'Stop zone: {self.stop_zone_distance} m')
        self.get_logger().info(f'Blind spot (min detection): {self.min_detection_range} m')
        self.get_logger().info('=' * 60)
    
    def pedestrian_detected_callback(self, msg):
        self.pedestrian_detected = msg.data
    
    def pedestrian_distance_callback(self, msg):
        self.pedestrian_distance = msg.data
    
    def pedestrian_behavior_callback(self, msg):
        self.pedestrian_behavior = msg.data
    
    def pedestrian_position_callback(self, msg):
        # x: distance along road, y: lateral distance from centerline
        self.pedestrian_lateral_pos = msg.y
        self.pedestrian_vel_toward_road = msg.x  
    
    def vehicle_speed_callback(self, msg):
        self.current_speed = msg.data
    
    def safety_logic_callback(self):
        """
        action based on pedestrian state and calculates speed commands
        """
        
        # Default: no override (pure pursuit runs normally)
        action = "NORMAL"
        target_speed = self.normal_speed
        emergency_brake = False
        
        if not self.pedestrian_detected:
            # No pedestrian detected - normal operation
            action = "NORMAL"
            target_speed = self.normal_speed
        
        else:
            # Pedestrian detected - evaluate behavior
            
            # Calculate time to blind spot
            time_to_blind_spot = self.calculate_time_to_blind_spot()
            
            # Calculate stopping distance needed
            stopping_distance = self.calculate_stopping_distance(
                self.current_speed, 
                self.max_decel
            )
            
            # CASE 1: PARALLEL (walking parallel to road)
            if self.pedestrian_behavior == "PARALLEL":
                action = "NORMAL"
                target_speed = self.normal_speed
                self.get_logger().info(
                    f"Pedestrian PARALLEL - Continuing normally",
                    throttle_duration_sec=2.0
                )
            
            # CASE 2: APPROACHING (walking toward road, not yet on it) 
            elif self.pedestrian_behavior == "APPROACHING":
                if self.pedestrian_distance > self.slow_zone_distance:
                    # Far away - continue normally
                    action = "NORMAL"
                    target_speed = self.normal_speed
                
                elif self.pedestrian_distance > self.stop_zone_distance:
                    # In slow zone - reduce speed
                    action = "SLOW"
                    target_speed = self.normal_speed * self.slow_speed_factor
                    
                    # smooth deceleration
                    target_speed = self.calculate_smooth_deceleration(
                        self.current_speed,
                        target_speed,
                        self.pedestrian_distance,
                        self.max_decel
                    )
                    
                    self.get_logger().info(
                        f"Pedestrian APPROACHING at {self.pedestrian_distance:.2f}m - "
                        f"SLOWING to {target_speed:.2f} m/s",
                        throttle_duration_sec=1.0
                    )
                
                else:
                    # Too close - prepare to stop
                    action = "SLOW"
                    target_speed = 0.5  # Very slow speed
                    
                    self.get_logger().warn(
                        f"Pedestrian APPROACHING CLOSE at {self.pedestrian_distance:.2f}m - "
                        f"Preparing to STOP"
                    )
            
            # CASE 3: STOPPED (was approaching, now stopped before road) 
            elif self.pedestrian_behavior == "STOPPED":
                # Pedestrian stopped - gradually resume normal speed
                action = "RESUME"
                
                # Check if safe to resume
                if self.pedestrian_distance > self.slow_zone_distance:
                    target_speed = self.normal_speed
                else:
                    # Still close - maintain slow speed
                    target_speed = self.normal_speed * self.slow_speed_factor
                
                # Gradual acceleration 
                target_speed = self.calculate_smooth_acceleration(
                    self.current_speed,
                    target_speed
                )
                
                self.get_logger().info(
                    f"Pedestrian STOPPED at {self.pedestrian_distance:.2f}m - "
                    f"RESUMING to {target_speed:.2f} m/s",
                    throttle_duration_sec=1.0
                )
            
            # CASE 4: CROSSING (pedestrian on road, moving across) 
            elif self.pedestrian_behavior == "CROSSING":    
                # Check if we can stop in time
                distance_available = self.pedestrian_distance - self.min_detection_range - self.safety_margin
                
                if stopping_distance < distance_available:
                    # Can stop safely - apply smooth braking
                    action = "STOP"
                    target_speed = 0.0
                    
                    # braking profile
                    target_speed = self.calculate_smooth_braking(
                        self.current_speed,
                        distance_available,
                        self.max_decel
                    )
                    
                    self.get_logger().warn(
                        f"Pedestrian CROSSING at {self.pedestrian_distance:.2f}m - "
                        f"STOPPING (target: {target_speed:.2f} m/s, "
                        f"stopping dist: {stopping_distance:.2f}m)"
                    )
                
                else:
                    # Cannot stop in time - EMERGENCY BRAKE
                    action = "EMERGENCY_STOP"
                    target_speed = 0.0
                    emergency_brake = True
                    
                    self.get_logger().error(
                        f"Pedestrian CROSSING at {self.pedestrian_distance:.2f}m - "
                        f"EMERGENCY BRAKE! (stopping dist: {stopping_distance:.2f}m > "
                        f"available: {distance_available:.2f}m)"
                    )
            
            # WARNING: Entering blind spot
            if self.pedestrian_distance < (self.min_detection_range + 1.0):
                self.get_logger().warn(
                    f"Pedestrian entering blind spot! Distance: {self.pedestrian_distance:.2f}m"
                )
        
        # ============ PUBLISH COMMANDS ============
        # speed override
        speed_msg = Float64()
        speed_msg.data = target_speed
        self.speed_override_pub.publish(speed_msg)
        
        # emergency brake flag
        brake_msg = Bool()
        brake_msg.data = emergency_brake
        self.brake_override_pub.publish(brake_msg)
        
        # action for logging
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)
        
        # status
        status_msg = String()
        status_msg.data = (
            f"Action: {action}, "
            f"Pedestrian: {self.pedestrian_behavior}, "
            f"Distance: {self.pedestrian_distance:.2f}m, "
            f"Target Speed: {target_speed:.2f} m/s, "
            f"Current Speed: {self.current_speed:.2f} m/s"
        )
        self.safety_status_pub.publish(status_msg)
        
        # Store state
        self.current_action = action
        self.target_speed = target_speed
    
    
    def calculate_time_to_blind_spot(self):
        """Calculate time until pedestrian enters blind spot"""
        if self.current_speed <= 0:
            return float('inf')
        
        distance_to_blind_spot = self.pedestrian_distance - self.min_detection_range
        if distance_to_blind_spot <= 0:
            return 0.0
        
        return distance_to_blind_spot / self.current_speed
    
    def calculate_stopping_distance(self, speed, decel):
        """Calculate distance needed to stop from current speed"""
        if decel <= 0:
            return float('inf')
        
        # Using d = v^2 / (2 * a)
        return (speed ** 2) / (2 * decel)
    
    def calculate_smooth_deceleration(self, current_speed, target_speed, distance, max_decel):
        """
        speed command for smooth deceleration over distance
        """
        if current_speed <= target_speed:
            return current_speed
        
        if distance <= 0:
            return target_speed
        
        # required deceleration: v_f^2 = v_i^2 + 2*a*d
        # a = (v_f^2 - v_i^2) / (2*d)
        required_decel = (target_speed**2 - current_speed**2) / (2 * distance)
        required_decel = abs(required_decel)  # Make positive
        
        # Limit to max comfortable deceleration
        decel = min(required_decel, max_decel)
        
        # Calculate speed for this time step (0.05s)
        dt = 0.05
        new_speed = current_speed - decel * dt
        
        # Don't overshoot target....not sure
        new_speed = max(new_speed, target_speed)
        
        return new_speed
    
    def calculate_smooth_braking(self, current_speed, distance_available, max_decel):
        """
        Calculate braking profile to stop before pedestrian
        Returns target speed for current timestep
        """
        if current_speed <= 0:
            return 0.0
        
        if distance_available <= 0:
            return 0.0  # Should already be stopped!
        
        # Calculate deceleration needed to stop in available distance
        # v_f^2 = v_i^2 + 2*a*d, with v_f = 0
        # a = -v_i^2 / (2*d)
        required_decel = (current_speed ** 2) / (2 * distance_available)
        
        # Use emergency decel if required decel is higher than max comfortable
        decel = min(required_decel, max_decel)
        if required_decel > max_decel:
            decel = self.emergency_decel
        
        # Calculate new speed for this timestep
        dt = 0.05
        new_speed = current_speed - decel * dt
        
        # Don't go negative
        new_speed = max(0.0, new_speed)
        
        return new_speed
    
    def calculate_smooth_acceleration(self, current_speed, target_speed):
        """
        Gradual acceleration back to normal speed
        """
        if current_speed >= target_speed:
            return current_speed
        
        # Gentle acceleration: 1 m/s^2
        accel = 1.0
        dt = 0.05
        
        new_speed = current_speed + accel * dt
        
        # no overshoot
        new_speed = min(new_speed, target_speed)
        
        return new_speed


def main(args=None):
    rclpy.init(args=args)
    bridge = PedestrianSafetyBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
