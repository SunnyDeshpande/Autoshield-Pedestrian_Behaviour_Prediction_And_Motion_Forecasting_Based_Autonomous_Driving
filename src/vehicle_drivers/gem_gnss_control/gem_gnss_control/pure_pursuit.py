#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_ros2.py
# Description: GNSS waypoints tracker using PID and pure pursuit in ROS2
# Author: Jiaming Zhang, Hang Cui
# Date: 2025-06-03
# Fixed by: Ansh (debugging and corrections)
#================================================================

import os
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
import pymap3d as pm
import pygame

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from pacmod2_msgs.msg import PositionWithSpeed, VehicleSpeedRpt, GlobalCmd, SystemCmdFloat, SystemCmdInt
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
# Visualization imports
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Initialize pygame for joystick
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick connected")
joystick = pygame.joystick.Joystick(0)
joystick.init()


class PID:
    def __init__(self, kp, ki, kd, wg=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def reset(self):
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e):
        if self.last_t is None:
            dt = 0.0
            de = 0.0
        else:
            dt = t - self.last_t
            de = (e - self.last_e) / dt if dt > 0.0 else 0.0

        self.iterm += e * dt
        if self.wg is not None:
            self.iterm = max(min(self.iterm, self.wg), -self.wg)

        self.last_e = e
        self.last_t = t

        return self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter:
    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted[0]


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # Declare parameters with default values
        self.declare_parameter('rate_hz', 20)
        self.declare_parameter('look_ahead', 5.0)
        self.declare_parameter('wheelbase', 2.57)
        self.declare_parameter('offset', 1.26)
        self.declare_parameter('origin_lat', 40.0927422)
        self.declare_parameter('origin_lon', -88.2359639)
        self.declare_parameter('desired_speed', 2.0)
        self.declare_parameter('max_acceleration', 0.5)

        self.declare_parameter('pid/kp', 0.6)
        self.declare_parameter('pid/ki', 0.0)
        self.declare_parameter('pid/kd', 0.1)
        self.declare_parameter('pid/wg', 10)

        self.declare_parameter('filter/cutoff', 1.2)
        self.declare_parameter('filter/fs', 30)
        self.declare_parameter('filter/order', 4)
        self.declare_parameter('vehicle_name', "")
        
        vehicle_name=self.get_parameter('vehicle_name').value
        if (vehicle_name==""):
            self.get_logger().warn("No vehicle_name parameter found. No config file loaded, defaulting to 'e4' parameters.")
        else:
            self.get_logger().info(f"Using vehicle config: {vehicle_name}")


        self.rate_hz = self.get_parameter('rate_hz').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.offset = self.get_parameter('offset').value
        self.olat = self.get_parameter('origin_lat').value
        self.olon = self.get_parameter('origin_lon').value

        self.desired_speed = min(5.0,self.get_parameter('desired_speed').value) # desired speed capped at 5 m/s
        self.max_accel = min(2.0, self.get_parameter('max_acceleration').value) # max acceleration capped at 2 m/s^2
        self.pid_speed = PID(
            kp=self.get_parameter('pid/kp').value,
            ki=self.get_parameter('pid/ki').value,
            kd=self.get_parameter('pid/kd').value,
            wg=self.get_parameter('pid/wg').value
            )
        self.speed_filter = OnlineFilter(
            cutoff=self.get_parameter('filter/cutoff').value,
            fs=self.get_parameter('filter/fs').value,
            order=self.get_parameter('filter/order').value,)

        self.goal = 0

        # Subscriptions
        self.create_subscription(NavSatFix, '/navsatfix', self.gnss_callback, 10)
        self.create_subscription(INSNavGeod, '/insnavgeod', self.ins_callback, 10)
        self.create_subscription(Bool, '/pacmod/enabled', self.enable_callback, 10)
        self.create_subscription(VehicleSpeedRpt, '/pacmod/vehicle_speed_rpt', self.speed_callback, 10)

        # Publishers
        self.global_pub = self.create_publisher(GlobalCmd, '/pacmod/global_cmd', 10)
        self.gear_pub = self.create_publisher(SystemCmdInt, '/pacmod/shift_cmd', 10)
        self.brake_pub = self.create_publisher(SystemCmdFloat, '/pacmod/brake_cmd', 10)
        self.accel_pub = self.create_publisher(SystemCmdFloat, '/pacmod/accel_cmd', 10)
        self.turn_pub = self.create_publisher(SystemCmdInt, '/pacmod/turn_cmd', 10)
        self.steer_pub = self.create_publisher(PositionWithSpeed, '/pacmod/steering_cmd', 10)
        
        # Visualization publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/markers', 10)

        # Commands
        self.global_cmd = GlobalCmd(enable=False, clear_override = True)
        self.gear_cmd = SystemCmdInt(command=2)  # NEUTRAL
        self.brake_cmd = SystemCmdFloat(command=0.0)
        self.accel_cmd = SystemCmdFloat(command=0.0)
        self.turn_cmd = SystemCmdInt(command=1) # no signal
        self.steer_cmd = PositionWithSpeed(angular_position=0.0, angular_velocity_limit=4.0)

        self.read_waypoints()

        # Initialize
        self.lat = 0.0
        self.lon = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.gem_enable = False
        self.pacmod_enable = False

        self.dist_arr = np.zeros(len(self.path_points_lon_x))

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

    def gnss_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def ins_callback(self, msg):
        self.heading = msg.heading

    def speed_callback(self, msg):
        self.speed = self.speed_filter.get_data(msg.vehicle_speed)

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def read_waypoints(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/track.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        self.path_points_lon_x = [float(p[0]) for p in path_points]
        self.path_points_lat_y = [float(p[1]) for p in path_points]
        self.path_points_heading = [float(p[2]) for p in path_points]
        self.wp_size = len(self.path_points_lon_x)

    def heading_to_yaw(self, heading):
        return np.radians(90 - heading) if heading < 270 else np.radians(450 - heading)

    def wps_to_local_xy(self, lon, lat):
        x, y, _ = pm.geodetic2enu(lat, lon, 0, self.olat, self.olon, 0)
        return x, y

    def dist(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def front2steer(self, f_angle):
        f_angle = max(min(f_angle, 35), -35)
        angle = abs(f_angle)
        steer_angle = -0.1084 * angle ** 2 + 21.775 * angle
        return round(steer_angle if f_angle >= 0 else -steer_angle, 2)

    def check_joystick_enable(self):
        pygame.event.pump()
        try:
            lb = joystick.get_button(6)
            rb = joystick.get_button(7)
        except pygame.error:
            self.get_logger().warn("Joystick read failed")
            return 2
        if lb and rb:
            # enable
            return 1
        elif lb and not rb:
            # disable
            return 0
        # others
        return 2

    def get_gem_state(self):
        local_x, local_y = self.wps_to_local_xy(self.lon, self.lat)
        yaw = self.heading_to_yaw(self.heading)
        x = local_x - self.offset * math.cos(yaw)
        y = local_y - self.offset * math.sin(yaw)
        return x, y, yaw

    def control_loop(self):
        joy_enable = self.check_joystick_enable()

        if joy_enable == 1 and not self.pacmod_enable:
            # joystick enable when vehicle disbaled 
            self.global_cmd.enable = True
            self.global_cmd.clear_override = True
            self.global_pub.publish(self.global_cmd)
            
            self.gear_cmd.command = 3
            self.gear_pub.publish(self.gear_cmd)
            
            self.brake_cmd.command = 0.0
            self.brake_pub.publish(self.brake_cmd)

            self.accel_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)

            # FIX: Turn signal should be off (1) when enabling, not left (3)
            # Original: self.turn_cmd.command = 3
            self.turn_cmd.command = 1  # No turn signal when enabling
            self.turn_pub.publish(self.turn_cmd)
            
            self.get_logger().warn('Pacmod Disabled: Vehicle enabled and forward gear engaged')

        elif joy_enable == 0 and self.pacmod_enable:
            # joystick disable when vehicle enbaled
            self.global_cmd.enable = False
            self.global_pub.publish(self.global_cmd)

            self.turn_cmd.command = 1
            self.turn_pub.publish(self.turn_cmd)

            self.get_logger().warn('Joystick Disabled: Vehicle disabled')

        elif joy_enable != 0 and self.pacmod_enable:
            # execute controller
            self.path_points_x = np.array(self.path_points_lon_x)
            self.path_points_y = np.array(self.path_points_lat_y)

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            
            # FIX: Optimize waypoint search - only search locally instead of all waypoints
            # This prevents re-finding waypoints behind the vehicle and improves efficiency
            # Original: for i in range(self.wp_size):
            search_start = max(0, self.goal - 10)
            search_end = min(self.wp_size, self.goal + 50)
            
            for i in range(search_start, search_end):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

            # FIX: Find closest waypoint only in search window
            # Original: self.goal = np.argmin(self.dist_arr)
            closest_idx = search_start + np.argmin(self.dist_arr[search_start:search_end])
            
            # FIX: Adaptive look-ahead with better scaling and minimum bound
            # Original: ld = self.look_ahead + max(0.0, self.speed - 2.5) * 2
            # The original scaled too aggressively (2x) and started scaling too late (2.5 m/s)
            min_lookahead = 3.0  # Minimum to prevent curvature singularities
            speed_gain = 0.5     # Gentler scaling: +0.5m per 1 m/s
            speed_threshold = 1.0  # Start scaling earlier
            ld = max(min_lookahead, self.look_ahead + max(0.0, self.speed - speed_threshold) * speed_gain)
            
            # FIX: Improved goal waypoint search with fallback
            # Original logic would get stuck if no waypoint beyond look-ahead distance
            # Search forward from closest waypoint to find one beyond look-ahead
            found_goal = False
            search_range = min(30, self.wp_size - closest_idx)  # Don't search too far ahead
            
            for i in range(closest_idx, closest_idx + search_range):
                idx = i % self.wp_size  # Wrap around for circular tracks
                dist_to_wp = self.dist((self.path_points_x[idx], self.path_points_y[idx]), (curr_x, curr_y))
                if dist_to_wp >= ld:
                    self.goal = idx
                    found_goal = True
                    break
            
            # If no waypoint beyond look-ahead found, advance along path
            if not found_goal:
                # Use a waypoint ahead along the path (not necessarily beyond look-ahead distance)
                self.goal = (closest_idx + 10) % self.wp_size

            target_x = self.path_points_x[self.goal]
            target_y = self.path_points_y[self.goal]
            target_yaw = self.path_points_heading[self.goal]
            
            # Pure pursuit lateral control calculation
            alpha = math.atan2(target_y - curr_y, target_x - curr_x) - curr_yaw
            
            # Normalize alpha to [-pi, pi]
            while alpha > math.pi:
                alpha -= 2 * math.pi
            while alpha < -math.pi:
                alpha += 2 * math.pi
            
            # FIX: Remove low-speed steering cutoff - vehicle needs steering at all speeds
            # Original: curvature = 0.0 if self.speed < 0.2 else 2.0 * math.sin(alpha) / ld
            # The cutoff prevented the vehicle from steering when moving slowly or from standstill
            curvature = 2.0 * math.sin(alpha) / ld
            
            steering_angle = math.atan(self.wheelbase * curvature)
            steering_wheel_angle = self.front2steer(math.degrees(steering_angle))

            self.steer_cmd.angular_position = math.radians(steering_wheel_angle)
            self.steer_pub.publish(self.steer_cmd)

            # Speed control with PID
            now = self.get_clock().now().nanoseconds * 1e-9
            speed_error = self.desired_speed - self.speed
            
            # FIX: Increase deadband to reduce control jitter from sensor noise
            # Original: if abs(speed_error) < 0.05:
            # 0.05 m/s is too tight and causes constant small adjustments
            if abs(speed_error) < 0.2:  # ~0.7 km/h deadband
                speed_error = 0.0
            
            throttle_cmd = self.pid_speed.get_control(now, speed_error)
            
            # FIX: Add brake control when slowing down
            # Original: throttle_cmd = max(0.0, min(throttle_cmd, self.max_accel))
            # Original only used throttle, never brake - vehicle couldn't decelerate properly
            if throttle_cmd > 0.0:
                # Accelerating
                accel_command = min(throttle_cmd, self.max_accel)
                # Clamp to valid throttle range [0, 1]
                self.accel_cmd.command = min(1.0, max(0.0, accel_command))
                self.brake_cmd.command = 0.0
            else:
                # Braking needed
                self.accel_cmd.command = 0.0
                # Convert negative PID output to brake pressure [0, 1]
                brake_command = abs(throttle_cmd) * 0.3  # Scale factor for brake sensitivity
                self.brake_cmd.command = min(1.0, max(0.0, brake_command))

            # Original code (replaced above):
            # self.accel_cmd.command = throttle_cmd
            # self.brake_cmd.command = 0.0
            
            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)

            self.global_cmd.enable = True
            self.global_pub.publish(self.global_cmd)
            
            # Publish visualization markers
            self.publish_visualization(curr_x, curr_y, target_x, target_y, ld, closest_idx)

            # Enhanced logging for debugging
            cross_track_error = self.dist((target_x, target_y), (curr_x, curr_y)) * math.sin(alpha)
            
            self.get_logger().info(
                f"Pos: ({curr_x:.2f}, {curr_y:.2f}), Goal: {self.goal}, "
                f"Target: ({target_x:.2f}, {target_y:.2f}), "
                f"Speed: {self.speed:.2f}, Throttle: {self.accel_cmd.command:.2f}, "
                f"Brake: {self.brake_cmd.command:.2f}, Steering: {steering_wheel_angle:.2f}°, "
                f"Lookahead: {ld:.2f}m, Alpha: {math.degrees(alpha):.1f}°, "
                f"Cross-track: {cross_track_error:.2f}m"
            )
    
    def publish_visualization(self, curr_x, curr_y, target_x, target_y, lookahead_dist, closest_idx):
        """
        Publish visualization markers for RViz:
        - Current vehicle position (green sphere)
        - Target waypoint (red sphere)
        - Lookahead circle (blue circle)
        - Path ahead (yellow line)
        - All waypoints (small cyan spheres)
        """
        marker_array = MarkerArray()
        
        # Marker 1: Current vehicle position (GREEN)
        vehicle_marker = Marker()
        vehicle_marker.header.frame_id = "map"
        vehicle_marker.header.stamp = self.get_clock().now().to_msg()
        vehicle_marker.ns = "vehicle"
        vehicle_marker.id = 0
        vehicle_marker.type = Marker.SPHERE
        vehicle_marker.action = Marker.ADD
        vehicle_marker.pose.position.x = curr_x
        vehicle_marker.pose.position.y = curr_y
        vehicle_marker.pose.position.z = 0.5
        vehicle_marker.scale.x = 2.0
        vehicle_marker.scale.y = 2.0
        vehicle_marker.scale.z = 2.0
        vehicle_marker.color.r = 0.0
        vehicle_marker.color.g = 1.0
        vehicle_marker.color.b = 0.0
        vehicle_marker.color.a = 1.0
        marker_array.markers.append(vehicle_marker)
        
        # Marker 2: Target waypoint (RED)
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target_x
        target_marker.pose.position.y = target_y
        target_marker.pose.position.z = 0.5
        target_marker.scale.x = 1.5
        target_marker.scale.y = 1.5
        target_marker.scale.z = 1.5
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0
        marker_array.markers.append(target_marker)
        
        # Marker 3: Lookahead circle (BLUE)
        circle_marker = Marker()
        circle_marker.header.frame_id = "map"
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.ns = "lookahead_circle"
        circle_marker.id = 2
        circle_marker.type = Marker.LINE_STRIP
        circle_marker.action = Marker.ADD
        circle_marker.scale.x = 0.1  # Line width
        circle_marker.color.r = 0.0
        circle_marker.color.g = 0.5
        circle_marker.color.b = 1.0
        circle_marker.color.a = 0.7
        
        # Create circle points
        num_points = 50
        for i in range(num_points + 1):
            angle = 2.0 * math.pi * i / num_points
            p = Point()
            p.x = curr_x + lookahead_dist * math.cos(angle)
            p.y = curr_y + lookahead_dist * math.sin(angle)
            p.z = 0.0
            circle_marker.points.append(p)
        
        marker_array.markers.append(circle_marker)
        
        # Marker 4: Path ahead (YELLOW LINE)
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path_ahead"
        path_marker.id = 3
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.3  # Line width
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.9
        
        # Show next 30 waypoints
        look_ahead_points = 30
        for i in range(look_ahead_points):
            idx = (self.goal + i) % self.wp_size
            p = Point()
            p.x = self.path_points_x[idx]
            p.y = self.path_points_y[idx]
            p.z = 0.0
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # Marker 5: All waypoints (CYAN DOTS)
        waypoints_marker = Marker()
        waypoints_marker.header.frame_id = "map"
        waypoints_marker.header.stamp = self.get_clock().now().to_msg()
        waypoints_marker.ns = "all_waypoints"
        waypoints_marker.id = 4
        waypoints_marker.type = Marker.POINTS
        waypoints_marker.action = Marker.ADD
        waypoints_marker.scale.x = 0.3  # Point size
        waypoints_marker.scale.y = 0.3
        waypoints_marker.color.r = 0.0
        waypoints_marker.color.g = 1.0
        waypoints_marker.color.b = 1.0
        waypoints_marker.color.a = 0.3
        
        # Add all waypoints (skip every 5th for performance)
        for i in range(0, self.wp_size, 5):
            p = Point()
            p.x = self.path_points_x[i]
            p.y = self.path_points_y[i]
            p.z = 0.0
            waypoints_marker.points.append(p)
        
        marker_array.markers.append(waypoints_marker)
        
        # Marker 6: Closest waypoint indicator (MAGENTA)
        closest_marker = Marker()
        closest_marker.header.frame_id = "map"
        closest_marker.header.stamp = self.get_clock().now().to_msg()
        closest_marker.ns = "closest_waypoint"
        closest_marker.id = 5
        closest_marker.type = Marker.CYLINDER
        closest_marker.action = Marker.ADD
        closest_marker.pose.position.x = self.path_points_x[closest_idx]
        closest_marker.pose.position.y = self.path_points_y[closest_idx]
        closest_marker.pose.position.z = 0.0
        closest_marker.scale.x = 1.0
        closest_marker.scale.y = 1.0
        closest_marker.scale.z = 0.1
        closest_marker.color.r = 1.0
        closest_marker.color.g = 0.0
        closest_marker.color.b = 1.0
        closest_marker.color.a = 0.8
        marker_array.markers.append(closest_marker)
        
        # Publish all markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
