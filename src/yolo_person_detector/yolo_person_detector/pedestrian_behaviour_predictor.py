#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray, Float64

from vehicle_msgs.msg import VehicleRpt  

from tf2_ros import TransformBroadcaster

import numpy as np


class PedestrianBehaviorPredictor(Node):
    def __init__(self):
        super().__init__('pedestrian_behavior_predictor')

        # ---------------- Parameters ----------------
        self.declare_parameter('prediction_time', 5.0)  # seconds
        self.declare_parameter('prediction_points', 20)
        self.declare_parameter('collision_distance_threshold', 1.0)  # meters
        self.declare_parameter('max_missing_frames', 10)
        self.declare_parameter('max_path_len', 100)
        self.declare_parameter('smooth_alpha', 0.6)

        self.prediction_time = float(self.get_parameter('prediction_time').value)
        self.prediction_points = int(self.get_parameter('prediction_points').value)
        self.collision_distance_threshold = float(self.get_parameter('collision_distance_threshold').value)
        self.max_missing_frames = int(self.get_parameter('max_missing_frames').value)
        self.max_path_len = int(self.get_parameter('max_path_len').value)
        self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)

        # Inputs:
        self.sub_ped = self.create_subscription(
            Int32MultiArray, 'fusion_pedestrian_position', self.pedestrian_cb, 10
        )
        self.sub_vehicle = self.create_subscription(
            VehicleRpt, 'vehicle_rpt', self.vehicle_cb, 10
        )

        # Outputs:
        self.pub_ped_motion = self.create_publisher(Twist, 'pedestrian_motion', 10)
        self.pub_ped_ttc = self.create_publisher(Float64, 'pedestrian_ttc', 10)

        # RViz markers
        self.pub_person_marker = self.create_publisher(Marker, 'person_marker', 10)
        self.pub_path_marker = self.create_publisher(Marker, 'person_path', 10)
        self.pub_prediction_marker = self.create_publisher(Marker, 'person_prediction', 10)
        self.pub_car_path_marker = self.create_publisher(Marker, 'car_path', 10)
        self.pub_camera_marker = self.create_publisher(Marker, 'camera_marker', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_camera_transform()

        # ---------------- State ----------------
        self.tracks = {}  # tid -> dict
        self.next_track_id = 0

        self.vehicle_speed = 0.0  # m/s
        self.vehicle_speed_valid = False

        self.get_logger().info("Pedestrian TTC Predictor ready (fusion distance+direction input + camera marker).")

    # ------------------------------------------------------------------
    # TF: base_link -> oak_rgb_camera_optical_frame
    # ------------------------------------------------------------------
    def publish_camera_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'oak_rgb_camera_optical_frame'
        t.transform.translation.x = 0.535
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.683
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.7071080798594738
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071054825112363
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Vehicle state callback
    # ------------------------------------------------------------------
    def vehicle_cb(self, msg: VehicleRpt):
        if msg.vehicle_speed_valid:
            self.vehicle_speed = float(msg.vehicle_speed)
            self.vehicle_speed_valid = True
        else:
            self.vehicle_speed_valid = False

    # ------------------------------------------------------------------
    # Tracking + prediction helpers
    # ------------------------------------------------------------------
    def _smooth_path_and_predict(self, path_3d, times):
        if len(path_3d) < 3 or len(times) < 3:
            return path_3d, []

        path_array = np.array(path_3d)

        # Spike removal
        filtered_path = [path_array[0]]
        spike_threshold = 0.5  # meters

        for i in range(1, len(path_array) - 1):
            prev_point = path_array[i - 1]
            curr_point = path_array[i]
            next_point = path_array[i + 1]
            expected = (prev_point + next_point) / 2.0
            deviation = np.linalg.norm(curr_point - expected)
            if deviation > spike_threshold:
                median_point = np.median([prev_point, curr_point, next_point], axis=0)
                filtered_path.append(median_point)
            else:
                filtered_path.append(curr_point)

        filtered_path.append(path_array[-1])
        filtered_array = np.array(filtered_path)

        # Moving average smoothing
        window = min(7, len(filtered_array))
        smoothed = []
        for i in range(len(filtered_array)):
            start_idx = max(0, i - window // 2)
            end_idx = min(len(filtered_array), i + window // 2 + 1)
            smoothed.append(np.mean(filtered_array[start_idx:end_idx], axis=0))

        # Velocity estimation using timestamps (non-constant dt)
        N = len(path_3d)
        start_index = max(1, N - 15)
        velocities = []
        for i in range(start_index, N):
            p_prev = np.array(path_3d[i - 1])
            p_curr = np.array(path_3d[i])
            t_prev = times[i - 1]
            t_curr = times[i]
            dt = t_curr - t_prev
            if dt <= 0.0:
                continue
            v = (p_curr - p_prev) / dt
            velocities.append(v)

        if len(velocities) == 0:
            return smoothed, []

        velocities_array = np.array(velocities)
        mean_vel = np.mean(velocities_array, axis=0)
        std_vel = np.std(velocities_array, axis=0) + 1e-6

        filtered_velocities = []
        for vel in velocities_array:
            if np.all(np.abs(vel - mean_vel) < 2 * std_vel):
                filtered_velocities.append(vel)

        if len(filtered_velocities) > 0:
            avg_velocity = np.mean(filtered_velocities, axis=0)
        else:
            avg_velocity = mean_vel

        # flatten vertical component
        avg_velocity[0] = 0.0

        speed = np.linalg.norm(avg_velocity)
        max_speed = 3.0
        if speed > max_speed:
            avg_velocity *= (max_speed / speed)

        avg_acceleration = np.zeros(3)

        last_pos = np.array(smoothed[-1])
        predicted = []

        ground_height = last_pos[0]
        ground_start_pos = last_pos.copy()
        ground_start_pos[0] = ground_height

        max_prediction_distance = 5.0
        for i in range(1, self.prediction_points + 1):
            t = (i / self.prediction_points) * self.prediction_time
            pred_pos = ground_start_pos + avg_velocity * t + 0.5 * avg_acceleration * t * t
            pred_pos[0] = ground_height

            distance = np.linalg.norm(pred_pos - ground_start_pos)
            if distance > max_prediction_distance:
                direction = (pred_pos - ground_start_pos) / (distance + 1e-9)
                pred_pos = ground_start_pos + direction * max_prediction_distance
                pred_pos[0] = ground_height
                predicted.append(tuple(pred_pos))
                break

            predicted.append(tuple(pred_pos))

        return smoothed, predicted

    def _update_tracks(self, detections, t_now):
        deleted_ids = []
        max_dist2 = 2.0 ** 2  # 2 m in YZ-plane
        used_tracks = set()

        for det in detections:
            X = det['x']
            Y = det['y']
            Z = det['z']

            best_id = None
            best_dist2 = max_dist2

            for tid, tr in self.tracks.items():
                dy = Y - tr['y']
