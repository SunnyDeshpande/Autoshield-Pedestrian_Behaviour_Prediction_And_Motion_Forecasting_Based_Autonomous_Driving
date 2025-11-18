#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_ros2.py
# Description: GNSS waypoints tracker using PID and pure pursuit in ROS2
# Author: Jiaming Zhang, Hang Cui
# Date: 2025-06-03
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

    def get_control(self, t, e, fwd=0):
        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # anti-windup
        if self.wg is not None:
            self.iterm = max(-self.wg, min(self.iterm, self.wg))

        p = self.kp * e
        i = self.ki * self.iterm
        d = self.kd * de
        ff = fwd

        self.last_e = e
        self.last_t = t

        return p + i + d + ff


class OnlineFilter:
    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        vehicle_name = self.declare_parameter('vehicle_name', 'e4').value

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('look_ahead', 5.0)
        self.declare_parameter('wheelbase', 1.75)
        self.declare_parameter('offset', 0.46)
        self.declare_parameter('origin_lat', 40.0927422)
        self.declare_parameter('origin_lon', -88.2359639)
        self.declare_parameter('desired_speed', 2.0)
        self.declare_parameter('max_acceleration', 0.48)
        self.declare_parameter('pid/kp', 0.5)
        self.declare_parameter('pid/ki', 0.0)
        self.declare_parameter('pid/kd', 0.1)
        self.declare_parameter('pid/wg', 20.0)
        self.declare_parameter('filter/cutoff', 1.2)
        self.declare_parameter('filter/fs', 20.0)
        self.declare_parameter('filter/order', 4)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.offset = self.get_parameter('offset').value
        self.olat = self.get_parameter('origin_lat').value
        self.olon = self.get_parameter('origin_lon').value

        self.desired_speed = min(5.0,self.get_parameter('desired_speed').value)
        self.max_accel = min(2.0, self.get_parameter('max_acceleration').value)
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
        
        # Visualization publisher
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
        self.pacmod_enable = False

        self.timer = self.create_timer(1.0/self.rate_hz, self.start_drive)

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def speed_callback(self, msg):
        self.speed = round(self.speed_filter.get_data(msg.vehicle_speed), 3)

    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 8)
        self.lon = round(msg.longitude, 8)

    def ins_callback(self, msg):
        self.heading = round(msg.heading, 5)

    def read_waypoints(self):
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to track.csv
        waypoint_file = os.path.join(script_dir, '..', 'waypoints', 'track.csv')

        with open(waypoint_file) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        self.path_points_lon_x   = [float(point[0]) for point in path_points]
        self.path_points_lat_y   = [float(point[1]) for point in path_points]
        self.path_points_heading = [float(point[2]) for point in path_points]
        self.wp_size             = len(self.path_points_lon_x)
        self.dist_arr            = np.zeros(self.wp_size)

        self.get_logger().info(f"Using vehicle config: e4")
        self.get_logger().info(f"Successfully loaded {self.wp_size} waypoints")

    def get_gem_state(self):
        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in local frame, in meters
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        curr_yaw = self.heading

        return round(local_x_curr, 3), round(local_y_curr, 3), round(curr_yaw, 4)

    # convert lat/lon to local x, y
    def wps_to_local_xy(self, lon_wp, lat_wp):
        x, y, _ = pm.geodetic2enu(lat_wp, lon_wp, 0, self.olat, self.olon, 0)
        return x, y

    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2), 3)

    def start_drive(self):
        # Get joystick input
        pygame.event.pump()
        joy_enable = joystick.get_button(7)

        if not self.pacmod_enable:
            return

        if joy_enable == 0:
            self.gear_cmd.command = 2  # NEUTRAL
            self.gear_pub.publish(self.gear_cmd)

            self.accel_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)

            self.brake_cmd.command = 1.0
            self.brake_pub.publish(self.brake_cmd)

            self.steer_cmd.angular_position = 0.0
            self.steer_pub.publish(self.steer_cmd)

            self.turn_cmd.command = 1
            self.turn_pub.publish(self.turn_cmd)

            self.get_logger().warn('Joystick Disabled: Vehicle disabled')

        elif joy_enable != 0 and self.pacmod_enable:
            # execute controller
            self.path_points_x = np.array(self.path_points_lon_x)
            self.path_points_y = np.array(self.path_points_lat_y)

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            
            # waypoint search 
            for i in range(self.wp_size):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

            self.goal = np.argmin(self.dist_arr)
            
            # MODIFIED: Adaptive look-ahead based on speed
            # Original: ld = self.look_ahead + max(0.0, self.speed - 2.5) * 2
            min_lookahead = 3.0
            speed_gain = 0.5
            speed_threshold = 1.0
            ld = max(min_lookahead, self.look_ahead + max(0.0, self.speed - speed_threshold) * speed_gain)
            
            # goal selection
            L = 0.0
            while L < ld and self.goal < self.wp_size:
                L = self.dist((self.path_points_x[self.goal], self.path_points_y[self.goal]), (curr_x, curr_y))
                self.goal += 1

            target_x = self.path_points_x[self.goal]
            target_y = self.path_points_y[self.goal]
            target_yaw = self.path_points_heading[self.goal]
            
            # Pure pursuit lateral control calculation
            alpha = math.atan2(target_y - curr_y, target_x - curr_x) - curr_yaw
            
            # ADDED: Normalize alpha to [-pi, pi]
            while alpha > math.pi:
                alpha -= 2.0 * math.pi
            while alpha < -math.pi:
                alpha += 2.0 * math.pi
            
            # Steering angle calculation
            steering_angle = math.atan((2 * self.wheelbase * math.sin(alpha)) / ld)
            steering_wheel_angle = math.degrees(steering_angle) * 15.5  # Steering ratio

            # MODIFIED: No steering command at very low speeds
            # Prevents steering instability when nearly stopped
            if self.speed < 0.2:  # Below 0.5 m/s
                self.steer_cmd.angular_position = 0.0  # Hold steering straight
            else:
                self.steer_cmd.angular_position = round(steering_wheel_angle, 1)
                
            self.steer_pub.publish(self.steer_cmd)

            self.gear_cmd.command = 3  # FORWARD
            self.gear_pub.publish(self.gear_cmd)

            # PID speed control
            e_v = self.desired_speed - self.speed
            throttle_cmd = self.pid_speed.get_control(self.get_clock().now().nanoseconds * 1e-9, e_v)
            throttle_cmd = round(np.clip(throttle_cmd, 0.0, self.max_accel), 5)

            self.accel_cmd.command = throttle_cmd
            self.brake_cmd.command = 0.0
            
            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)

            self.global_cmd.enable = True
            self.global_pub.publish(self.global_cmd)
            
            # ADDED: Publish visualization
            self.publish_visualization(curr_x, curr_y, target_x, target_y, ld, self.goal)

            # Logging
            self.get_logger().info(
                f"Pos: ({curr_x:.2f}, {curr_y:.2f}), Goal: {self.goal}, "
                f"Target: ({target_x:.2f}, {target_y:.2f}), "
                f"Speed: {self.speed:.2f}, Throttle: {self.accel_cmd.command:.2f}, "
                f"Steering: {steering_wheel_angle:.2f}°, Lookahead: {ld:.2f}m"
            )
    
    def publish_visualization(self, curr_x, curr_y, target_x, target_y, lookahead_dist, goal_idx):
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
        circle_marker.scale.x = 0.1
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
        path_marker.scale.x = 0.3
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.9
        
        # Show next 30 waypoints
        look_ahead_points = 30
        for i in range(look_ahead_points):
            idx = (goal_idx + i) % self.wp_size
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
        waypoints_marker.scale.x = 0.3
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
