#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
<<<<<<< HEAD

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String

from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

=======
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
import numpy as np
import cv2
import time

<<<<<<< HEAD
=======
# Ultralytics YOLOv11
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

<<<<<<< HEAD
COCO_PERSON_CLASS_ID = 0
=======
COCO_PERSON_CLASS_ID = 0  # "person"
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)


class YoloV11PersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_v11_person_detector')

<<<<<<< HEAD
        # ---------------- Parameters ----------------
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
=======
        # Params
        self.declare_parameter('image_topic', '/camera/color/image_raw')
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('half', False)
        self.declare_parameter('max_detections', 100)
<<<<<<< HEAD

        self.declare_parameter('x_vel_buffer_length', 3)
        self.declare_parameter('y_vel_buffer_length', 10)
        self.declare_parameter('vehicle_speed', 0.5)
        self.declare_parameter('min_safe_distance', 1.0)
        self.declare_parameter('max_ped_speed_for_safety', 2.0)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

=======
        self.declare_parameter('x_vel_buffer_length', 3)
        self.declare_parameter('y_vel_buffer_length', 10)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.half = bool(self.get_parameter('half').value)
        self.max_det = int(self.get_parameter('max_detections').value)
<<<<<<< HEAD

        self.x_vel_buffer_length = int(self.get_parameter('x_vel_buffer_length').value)
        self.y_vel_buffer_length = int(self.get_parameter('y_vel_buffer_length').value)
        self.vehicle_speed = float(self.get_parameter('vehicle_speed').value)
        self.min_safe_distance = float(self.get_parameter('min_safe_distance').value)
        self.max_ped_speed_for_safety = float(self.get_parameter('max_ped_speed_for_safety').value)

        # ---------------- YOLO model ----------------
        if YOLO is None:
            raise RuntimeError("Ultralytics is not installed. `pip install ultralytics`")

        self.get_logger().info(f"Loading YOLOv11 model: {model_path}")
        self.model = YOLO(model_path)
=======
        self.x_vel_buffer_length = int(self.get_parameter('x_vel_buffer_length').value)
        self.y_vel_buffer_length = int(self.get_parameter('y_vel_buffer_length').value)

        if YOLO is None:
            raise RuntimeError("Ultralytics is not installed. pip install ultralytics")

        self.get_logger().info(f"Loading YOLOv11 model: {model_path}")
        self.model = YOLO(model_path)

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        self.model.overrides['conf'] = self.conf
        self.model.overrides['iou'] = self.iou
        self.model.overrides['device'] = self.device
        self.model.overrides['imgsz'] = self.imgsz
        self.model.overrides['half'] = self.half
        self.model.overrides['max_det'] = self.max_det
        self.model.overrides['classes'] = [COCO_PERSON_CLASS_ID]

<<<<<<< HEAD
        # ---------------- ROS I/O ----------------
=======
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.bridge = CvBridge()
<<<<<<< HEAD

        self.sub_rgb = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)
=======
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)

        self.pub_dets = self.create_publisher(Detection2DArray, 'detections', 10)
        self.pub_debug = self.create_publisher(Image, 'detections/image', 10) if self.publish_debug else None

<<<<<<< HEAD
<<<<<<< HEAD
        self.pub_marker = self.create_publisher(Marker, 'person_marker', 10)
        self.pub_path_marker = self.create_publisher(Marker, 'person_path', 10)
        self.pub_prediction_marker = self.create_publisher(Marker, 'person_prediction', 10)
        self.pub_car_marker = self.create_publisher(Marker, 'car_marker', 10)

        self.pub_vehicle_cmd = self.create_publisher(String, 'vehicle_control_command', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_camera_transform()

        # ---------------- State ----------------
=======
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
=======
        # FPS measurement (wall clock is fine here)
>>>>>>> b4cf006 (upgraded detect_node)
        self.last_fps_t = time.time()
        self.frame_count = 0

        # Simple buffers (not per-ID, just global)
        self.x_vel_buffer = []
        self.y_vel_buffer = []
<<<<<<< HEAD
        self.prev_boxes = []
        self.prev_time = None
        self.prev_corners_vel = None
        self.latest_depth = None

        self.tracks = {}
        self.next_track_id = 0
        self.max_path_len = 100
        self.max_missing_frames = 10
        self.smooth_alpha = 0.6
        self.prediction_time = 5.0
        self.prediction_points = 20
        
        # Kalman filter-like smoothing parameters for person location
        self.position_smooth_alpha = 0.3  # Lower = more smoothing

        self.get_logger().info("YOLOv11 person detector ready.")

    # ------------------------------------------------------------------
    # TF
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
    # Depth callback
    # ------------------------------------------------------------------
    def depth_cb(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_img.astype(np.float32) / 1000.0  # mm -> m
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _point_in_triangle(px, py, x1, y1, x2, y2, x3, y3):
        denom = ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3))
        if abs(denom) < 1e-6:
            return False
        a = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom
        b = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom
        c = 1.0 - a - b
        return (0.0 <= a <= 1.0) and (0.0 <= b <= 1.0) and (0.0 <= c <= 1.0)

    @staticmethod
    def _line_segment_intersection(p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-9:
            return False, None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return True, (ix, iy)
        return False, None

    @staticmethod
    def _path_intersects_line(path, line_start, line_end, threshold=0.2):
        # Segment intersection first
        for i in range(len(path) - 1):
            intersects, point = YoloV11PersonDetector._line_segment_intersection(
                path[i][:2], path[i + 1][:2], line_start, line_end
            )
            if intersects:
                return True, point
        # Distance of points to line
        x1, y1 = line_start
        x2, y2 = line_end
        denom = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2) + 1e-9
        for point in path:
            px, py = point[0], point[1]
            dist = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1) / denom
            if dist < threshold:
                return True, point
        return False, None

    # ------------------------------------------------------------------
    # Path smoothing + prediction
    # ------------------------------------------------------------------
    def _smooth_path_and_predict(self, path_3d):
        if len(path_3d) < 3:
            return path_3d, []

        path_array = np.array(path_3d)

        # AGGRESSIVE SPIKE REMOVAL - Multiple passes
        filtered_path = path_array.copy()
        
        # Pass 1: Remove outliers using median absolute deviation (MAD)
        for axis in range(3):  # X, Y, Z
            median = np.median(filtered_path[:, axis])
            mad = np.median(np.abs(filtered_path[:, axis] - median))
            if mad > 1e-6:
                # Points more than 3 MAD away are outliers
                outlier_mask = np.abs(filtered_path[:, axis] - median) > 3 * mad
                # Replace outliers with median
                filtered_path[outlier_mask, axis] = median
        
        # Pass 2: Median filter (removes isolated spikes)
        window_size = min(5, len(filtered_path))
        if window_size >= 3:
            median_filtered = []
            for i in range(len(filtered_path)):
                start_idx = max(0, i - window_size // 2)
                end_idx = min(len(filtered_path), i + window_size // 2 + 1)
                window = filtered_path[start_idx:end_idx]
                median_filtered.append(np.median(window, axis=0))
            filtered_path = np.array(median_filtered)
        
        # Pass 3: Remove points with sharp direction changes
        clean_path = [filtered_path[0]]
        spike_threshold = 0.3  # meters - much stricter threshold
        
        for i in range(1, len(filtered_path) - 1):
            prev_point = filtered_path[i - 1]
            curr_point = filtered_path[i]
            next_point = filtered_path[i + 1]
            
            # Expected position (linear interpolation)
            expected = (prev_point + next_point) / 2.0
            deviation = np.linalg.norm(curr_point - expected)
            
            # Check angle
            v1 = curr_point - prev_point
            v2 = next_point - curr_point
            
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = np.arccos(cos_angle)
                
                # Sharp angle (> 60 degrees) or large deviation = spike
                if deviation > spike_threshold or angle > np.pi / 3:
                    # Use linear interpolation instead of median
                    clean_path.append(expected)
                else:
                    clean_path.append(curr_point)
            elif deviation > spike_threshold:
                clean_path.append(expected)
            else:
                clean_path.append(curr_point)
        
        clean_path.append(filtered_path[-1])
        filtered_array = np.array(clean_path)

        # Heavy moving average smoothing
        window = min(15, len(filtered_array))  # Larger window for smoother result
        smoothed = []
        for i in range(len(filtered_array)):
            start_idx = max(0, i - window // 2)
            end_idx = min(len(filtered_array), i + window // 2 + 1)
            smoothed.append(np.mean(filtered_array[start_idx:end_idx], axis=0))

        # Strong exponential smoothing
        exponential_smoothed = [smoothed[0]]
        alpha_exp = 0.15  # Lower alpha = more smoothing
        for i in range(1, len(smoothed)):
            new_point = alpha_exp * np.array(smoothed[i]) + (1 - alpha_exp) * np.array(exponential_smoothed[-1])
            exponential_smoothed.append(new_point)

        smoothed_path = exponential_smoothed
        if len(smoothed_path) < 3:
            return smoothed_path, []

        # IMPROVED VELOCITY ESTIMATION - Use last 3x the number of prediction points
        dt_per_frame = 0.033
        num_points_for_velocity = min(len(smoothed_path), self.prediction_points * 3)
        recent_points = smoothed_path[-num_points_for_velocity:]
        
        if len(recent_points) < 2:
            return smoothed_path, []

        velocities = []
        for i in range(1, len(recent_points)):
            vel = (np.array(recent_points[i]) - np.array(recent_points[i - 1])) / dt_per_frame
            velocities.append(vel)

        if len(velocities) == 0:
            return smoothed_path, []

        velocities_array = np.array(velocities)
        mean_vel = np.mean(velocities_array, axis=0)
        std_vel = np.std(velocities_array, axis=0) + 1e-6

        # Filter outlier velocities
        filtered_velocities = []
        for vel in velocities_array:
            if np.all(np.abs(vel - mean_vel) < 2 * std_vel):
                filtered_velocities.append(vel)

        if len(filtered_velocities) > 0:
            avg_velocity = np.mean(filtered_velocities, axis=0)
        else:
            avg_velocity = mean_vel

        # Flatten vertical component (assume ground)
        avg_velocity[0] = 0.0

        # Clamp max speed
        speed = np.linalg.norm(avg_velocity)
        max_speed = 3.0
        if speed > max_speed:
            avg_velocity = avg_velocity * (max_speed / speed)

        avg_acceleration = np.zeros(3)

        last_pos = np.array(smoothed_path[-1])
        predicted = []

        ground_height = 0.0
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

        return smoothed_path, predicted

    # ------------------------------------------------------------------
    # Real-time position filtering (applied before adding to track)
    # ------------------------------------------------------------------
    def _filter_detection_position(self, new_X, new_Y, new_Z, track):
        """Apply real-time filtering to incoming detection positions"""
        if 'prev_filtered_X' not in track:
            # First detection - no filtering
            track['prev_filtered_X'] = new_X
            track['prev_filtered_Y'] = new_Y
            track['prev_filtered_Z'] = new_Z
            track['velocity_X'] = 0.0
            track['velocity_Y'] = 0.0
            track['velocity_Z'] = 0.0
            return new_X, new_Y, new_Z
        
        # Get previous filtered position and velocity
        prev_X = track['prev_filtered_X']
        prev_Y = track['prev_filtered_Y']
        prev_Z = track['prev_filtered_Z']
        prev_vx = track['velocity_X']
        prev_vy = track['velocity_Y']
        prev_vz = track['velocity_Z']
        
        dt = 0.033  # ~30 FPS
        
        # Predict where the person should be based on previous velocity
        predicted_X = prev_X + prev_vx * dt
        predicted_Y = prev_Y + prev_vy * dt
        predicted_Z = prev_Z + prev_vz * dt
        
        # Check if new detection is a spike (too far from prediction)
        deviation = np.sqrt(
            (new_X - predicted_X)**2 + 
            (new_Y - predicted_Y)**2 + 
            (new_Z - predicted_Z)**2
        )
        
        max_reasonable_speed = 5.0  # m/s (person can't move faster than this)
        max_reasonable_distance = max_reasonable_speed * dt
        
        if deviation > max_reasonable_distance:
            # This is likely a spike/outlier - use prediction instead
            filtered_X = predicted_X
            filtered_Y = predicted_Y
            filtered_Z = predicted_Z
        else:
            # Apply exponential smoothing
            alpha = self.position_smooth_alpha
            filtered_X = alpha * new_X + (1 - alpha) * predicted_X
            filtered_Y = alpha * new_Y + (1 - alpha) * predicted_Y
            filtered_Z = alpha * new_Z + (1 - alpha) * predicted_Z
        
        # Update velocity estimate
        new_vx = (filtered_X - prev_X) / dt
        new_vy = (filtered_Y - prev_Y) / dt
        new_vz = (filtered_Z - prev_Z) / dt
        
        # Smooth velocity too
        velocity_alpha = 0.4
        track['velocity_X'] = velocity_alpha * new_vx + (1 - velocity_alpha) * prev_vx
        track['velocity_Y'] = velocity_alpha * new_vy + (1 - velocity_alpha) * prev_vy
        track['velocity_Z'] = velocity_alpha * new_vz + (1 - velocity_alpha) * prev_vz
        
        # Store filtered position for next frame
        track['prev_filtered_X'] = filtered_X
        track['prev_filtered_Y'] = filtered_Y
        track['prev_filtered_Z'] = filtered_Z
        
        return filtered_X, filtered_Y, filtered_Z

    # ------------------------------------------------------------------
    # Tracking
    # ------------------------------------------------------------------
    def _update_tracks(self, detections, img_w, img_h):
        deleted_ids = []
        max_dist2 = 80.0 ** 2
        used_tracks = set()

        for det in detections:
            cx, cy, z = det['cx'], det['cy'], det['z']
            X, Y, Z = det['X'], det['Y'], det['Z']

            best_id = None
            best_dist2 = max_dist2

            for tid, tr in self.tracks.items():
                dx = cx - tr['cx']
                dy = cy - tr['cy']
                d2 = dx * dx + dy * dy
                if d2 < best_dist2:
                    best_dist2 = d2
                    best_id = tid

            if best_id is None:
                # New track
                tid = self.next_track_id
                self.next_track_id += 1
                self.tracks[tid] = {
                    'cx': cx, 'cy': cy, 'z': z,
                    'X': X, 'Y': Y, 'Z': Z,
                    'sx': X, 'sy': Y, 'sz': Z,
                    'path_img': [(int(cx), int(cy))],
                    'path_3d': [(X, Y, Z)],
                    'smoothed_path': [(X, Y, Z)],
                    'predicted_path': [],
                    'missed': 0,
                }
                used_tracks.add(tid)
            else:
                # Update existing track
                tr = self.tracks[best_id]
                tr['cx'] = cx
                tr['cy'] = cy
                tr['z'] = z
                
                # Apply real-time position filtering BEFORE adding to path
                filtered_X, filtered_Y, filtered_Z = self._filter_detection_position(X, Y, Z, tr)
                
                tr['X'] = filtered_X
                tr['Y'] = filtered_Y
                tr['Z'] = filtered_Z

                alpha = self.smooth_alpha
                tr['sx'] = alpha * filtered_X + (1.0 - alpha) * tr['sx']
                tr['sy'] = alpha * filtered_Y + (1.0 - alpha) * tr['sy']
                tr['sz'] = alpha * filtered_Z + (1.0 - alpha) * tr['sz']

                tr['path_img'].append((int(cx), int(cy)))
                tr['path_3d'].append((tr['sx'], tr['sy'], tr['sz']))

                if len(tr['path_img']) > self.max_path_len:
                    tr['path_img'].pop(0)
                if len(tr['path_3d']) > self.max_path_len:
                    tr['path_3d'].pop(0)

                smoothed, predicted = self._smooth_path_and_predict(tr['path_3d'])
                tr['smoothed_path'] = smoothed
                tr['predicted_path'] = predicted
                tr['missed'] = 0
                used_tracks.add(best_id)

        # Increment miss counters for unused tracks
        for tid in list(self.tracks.keys()):
            if tid not in used_tracks:
                self.tracks[tid]['missed'] += 1
                if self.tracks[tid]['missed'] > self.max_missing_frames:
                    deleted_ids.append(tid)
                    del self.tracks[tid]

        return deleted_ids

    # ------------------------------------------------------------------
    # Pedestrian kinematics helpers
    # ------------------------------------------------------------------
    def _is_pedestrian_slowing_down(self, tr):
        if len(tr['path_3d']) < 10:
            return False
        recent_path = tr['path_3d'][-10:]
        velocities = []
        dt = 0.033
        for i in range(1, len(recent_path)):
            vel = np.linalg.norm(np.array(recent_path[i]) - np.array(recent_path[i - 1])) / dt
            velocities.append(vel)
        if len(velocities) < 5:
            return False
        recent_vel = np.mean(velocities[-3:])
        older_vel = np.mean(velocities[:3])
        deceleration = older_vel - recent_vel
        return deceleration > 0.1

    def _get_pedestrian_velocity(self, tr):
        if len(tr['path_3d']) < 2:
            return 0.0
        recent_path = tr['path_3d'][-5:]
        velocities = []
        dt = 0.033
        for i in range(1, len(recent_path)):
            vel = np.linalg.norm(np.array(recent_path[i]) - np.array(recent_path[i - 1])) / dt
            velocities.append(vel)
        return np.mean(velocities) if velocities else 0.0

    def _compute_velocity_vector(self, tr):
        if len(tr['path_3d']) < 3:
            return np.array([0.0, 0.0, 0.0])
        recent_path = np.array(tr['path_3d'][-3:])
        dt = 0.033
        vx = (recent_path[-1, 0] - recent_path[0, 0]) / (2 * dt)
        vy = (recent_path[-1, 1] - recent_path[0, 1]) / (2 * dt)
        vz = (recent_path[-1, 2] - recent_path[0, 2]) / (2 * dt)
        return np.array([vx, vy, vz])

    def _time_to_collision(self, predicted_path, ped_velocity_vector, vehicle_width=1.0, vehicle_length=2.0):
        if len(predicted_path) < 2:
            return float('inf'), float('inf'), None

        vehicle_x_min = -vehicle_width / 2.0
        vehicle_x_max = vehicle_width / 2.0
        vehicle_y_min = 0.0
        vehicle_y_max = vehicle_length

        min_distance = float('inf')
        collision_time = float('inf')
        entry_point = None

        for i, point in enumerate(predicted_path):
            px, py, pz = point[0], point[1], point[2]

            distance_to_vehicle = np.sqrt(
                max(0, abs(px) - vehicle_width / 2.0) ** 2 +
                max(0, max(0, vehicle_y_min - py), py - vehicle_y_max) ** 2
            )

            if distance_to_vehicle < min_distance:
                min_distance = distance_to_vehicle
                entry_point = point

            in_x_zone = vehicle_x_min <= px <= vehicle_x_max
            in_y_zone = vehicle_y_min <= py <= vehicle_y_max

            if in_x_zone and in_y_zone:
                t = i * 0.033
                if t < collision_time:
                    collision_time = t
                    entry_point = point

        if collision_time == float('inf') and min_distance < 0.5:
            collision_time = min_distance / (np.linalg.norm(ped_velocity_vector) + 0.01)

        return collision_time, min_distance, entry_point

    def _get_closest_collision_pedestrian(self, collision_peds):
        if not collision_peds:
            return None, None
        min_ttc = float('inf')
        closest_ped = None
        for tid, ped_data in collision_peds.items():
            if ped_data['ttc'] < min_ttc:
                min_ttc = ped_data['ttc']
                closest_ped = tid
        
        if closest_ped is None:
            return None, None
        
        return closest_ped, collision_peds[closest_ped]

    def _safety_filter(self, primary_cmd, collision_detected, ttc, ped_speed, ped_slowing):
        if not collision_detected:
            return "CONTINUE"

        if ttc < 1.0:
            safe_cmd = "STOP"
            reason = f"TTC too short ({ttc:.2f}s)"
        elif ped_speed > self.max_ped_speed_for_safety and not ped_slowing:
            safe_cmd = "STOP"
            reason = f"Pedestrian speed high ({ped_speed:.2f} m/s) and not slowing"
        elif ped_slowing and ttc > 3.0:
            safe_cmd = "CONTINUE_SLOW"
            reason = f"Pedestrian slowing, TTC={ttc:.2f}s"
        elif not ped_slowing and ttc < 3.0:
            safe_cmd = "STOP"
            reason = f"Pedestrian not slowing, TTC={ttc:.2f}s"
        else:
            safe_cmd = "CONTINUE_SLOW"
            reason = "Default caution"

        self.get_logger().info(f"Safety Filter: {primary_cmd} -> {safe_cmd} ({reason})")
        return safe_cmd

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------
=======

        self.prev_boxes = []          # list of (x1, y1, x2, y2) from previous frame
        self.prev_time = None         # timestamp of previous frame (seconds)
        self.prev_corners_vel = None  # list of 4 (vx, vy) from previous frame

        self.get_logger().info("YOLOv11 person detector ready.")

    @staticmethod
    def _point_in_triangle(px, py, x1, y1, x2, y2, x3, y3):
        denom = ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3))
        if abs(denom) < 1e-6:
            return False
        a = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom
        b = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom
        c = 1.0 - a - b
        return (0.0 <= a <= 1.0) and (0.0 <= b <= 1.0) and (0.0 <= c <= 1.0)

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
    def image_cb(self, msg: Image):
        # Convert to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
<<<<<<< HEAD
            self.get_logger().error(f"cv_bridge RGB failed: {e}")
            return

        self.publish_camera_transform()

        h, w = cv_image.shape[:2]
        debug_img = cv_image.copy() if self.publish_debug else None

        fx = float(w)
        fy = float(h)
        cx0 = w / 2.0
        cy0 = h / 2.0

        # Region geometry in image space
=======
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, w = cv_image.shape[:2]
        debug_img = cv_image.copy() if self.publish_debug else None

        # fan geometry (for logic + drawing)
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        x_center = int(w / 2)
        x_left_third = int(w / 3)
        x_right_third = int(2 * w / 3)
        x_left_sixth = int(w / 6)
        x_right_sixth = int(5 * w / 6)
        delta = int(w / 6)

        top_center = (x_center, 0)
        top_left_yellow = (x_center - delta, 0)
        top_right_yellow = (x_center + delta, 0)
<<<<<<< HEAD
=======

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        bottom_left_red = (x_left_third, h - 1)
        bottom_right_red = (x_right_third, h - 1)
        bottom_left_yellow = (x_left_sixth, h - 1)
        bottom_right_yellow = (x_right_sixth, h - 1)

<<<<<<< HEAD
<<<<<<< HEAD
        stamp = msg.header.stamp
        t_now = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        dt = None
        if self.prev_time is not None:
            dt = t_now - self.prev_time
            if dt <= 0.0:
                dt = None

        # ---------------- YOLO inference ----------------
        results = self.model.predict(cv_image, verbose=False, stream=False)
=======
        now = self.get_clock().now()
        t_now = now.nanoseconds * 1e-9
=======
        # Use the camera's timestamp, not node clock
        stamp = msg.header.stamp
        t_now = float(stamp.sec) + float(stamp.nanosec) * 1e-9

>>>>>>> b4cf006 (upgraded detect_node)
        dt = None
        if self.prev_time is not None:
            dt = t_now - self.prev_time
            if dt <= 0.0:
                # If something weird, ignore dt for this step
                dt = None

        # Run YOLO
        results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False
        )

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
        det_msg = Detection2DArray()
        det_msg.header = msg.header

        r = results[0]
        curr_boxes = []
<<<<<<< HEAD
        detections_for_tracks = []
=======
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)

        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.detach().cpu().numpy()
            confs = r.boxes.conf.detach().cpu().numpy()
            clss = r.boxes.cls.detach().cpu().numpy().astype(int)

            for i in range(xyxy.shape[0]):
                if clss[i] != COCO_PERSON_CLASS_ID:
                    continue

                x1, y1, x2, y2 = xyxy[i]
                score = float(confs[i])

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                bw = (x2 - x1)
                bh = (y2 - y1)

<<<<<<< HEAD
<<<<<<< HEAD
                vx = vy = 0.0
                ax = ay = 0.0
                vel_candidates_scaled = None

                # Estimate velocity from box corner motion
                if dt is not None and len(self.prev_boxes) > 0:
                    prev_centers = [((px1 + px2) / 2.0, (py1 + py2) / 2.0) for (px1, py1, px2, py2) in self.prev_boxes]
                    dists = [(pcx - cx) ** 2 + (pcy - cy) ** 2 for (pcx, pcy) in prev_centers]
                    j = int(np.argmin(dists))
                    px1, py1, px2, py2 = self.prev_boxes[j]

                    prev_corners = [(px1, py1), (px2, py1), (px1, py2), (px2, py2)]
                    curr_corners = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]

                    vel_candidates_px = [
                        ((cx2 - cx1) / dt, (cy2 - cy1) / dt)
                        for (cx1, cy1), (cx2, cy2) in zip(prev_corners, curr_corners)
                    ]

                    # crude m/px scaling
                    meters_per_pixel = 1.7 / max(bh, 1.0)
                    vel_candidates_scaled = [
                        (vx_c * meters_per_pixel, vy_c * meters_per_pixel) for vx_c, vy_c in vel_candidates_px
                    ]
                    speeds = [vx_s ** 2 + vy_s ** 2 for (vx_s, vy_s) in vel_candidates_scaled]
                    min_v_idx = int(np.argmin(speeds))
                    vx, vy = vel_candidates_scaled[min_v_idx]

                    # acceleration from previous velocities
                    if self.prev_corners_vel is not None and len(self.prev_corners_vel) == 4:
                        accel_speeds = []
                        accel_candidates = []
                        for (vx_c, vy_c), (vx_prev, vy_prev) in zip(vel_candidates_scaled, self.prev_corners_vel):
                            ax_c = (vx_c - vx_prev) / dt
                            ay_c = (vy_c - vy_prev) / dt
                            accel_candidates.append((ax_c, ay_c))
                            accel_speeds.append(ax_c ** 2 + ay_c ** 2)
=======
=======
                # Velocity and acceleration in "meters per second" approx
>>>>>>> b4cf006 (upgraded detect_node)
                vx = 0.0
                vy = 0.0
                ax = 0.0
                ay = 0.0

                vel_candidates_scaled = None

                if dt is not None and len(self.prev_boxes) > 0:
                    # Compute centers of previous boxes
                    px_cx_cy = []
                    for (px1, py1, px2, py2) in self.prev_boxes:
                        pcx = (px1 + px2) / 2.0
                        pcy = (py1 + py2) / 2.0
                        px_cx_cy.append((pcx, pcy))

                    # Nearest previous box by center
                    dists = [
                        (pcx - cx) ** 2 + (pcy - cy) ** 2
                        for (pcx, pcy) in px_cx_cy
                    ]
                    j = int(np.argmin(dists))
                    px1, py1, px2, py2 = self.prev_boxes[j]

                    # 4 corners previous and current
                    prev_corners = [
                        (px1, py1),
                        (px2, py1),
                        (px1, py2),
                        (px2, py2),
                    ]
                    curr_corners = [
                        (x1, y1),
                        (x2, y1),
                        (x1, y2),
                        (x2, y2),
                    ]

                    # Pixel-space velocities (px/s)
                    vel_candidates_px = []
                    for (cx_prev, cy_prev), (cx_curr, cy_curr) in zip(prev_corners, curr_corners):
                        dx = cx_curr - cx_prev
                        dy = cy_curr - cy_prev
                        vx_c = dx / dt
                        vy_c = dy / dt
                        vel_candidates_px.append((vx_c, vy_c))

                    # Approximate pixel → meters scaling by assuming person height ~1.7 m
                    person_height_m = 1.7
                    bbox_height_px = max(bh, 1.0)
                    meters_per_pixel = person_height_m / bbox_height_px

                    vel_candidates_scaled = [
                        (vx_c * meters_per_pixel, vy_c * meters_per_pixel)
                        for (vx_c, vy_c) in vel_candidates_px
                    ]

                    # Pick the corner with minimum speed for stability
                    speeds = [
                        vx_s ** 2 + vy_s ** 2
                        for (vx_s, vy_s) in vel_candidates_scaled
                    ]
                    min_v_idx = int(np.argmin(speeds))
                    vx, vy = vel_candidates_scaled[min_v_idx]

                    # Acceleration estimate (difference of velocities / dt)
                    if (
                        self.prev_corners_vel is not None
                        and len(self.prev_corners_vel) == 4
                    ):
                        accel_speeds = []
                        accel_candidates = []
                        for (vx_curr, vy_curr), (vx_prev, vy_prev) in zip(
                            vel_candidates_scaled, self.prev_corners_vel
                        ):
                            ax_c = (vx_curr - vx_prev) / dt
                            ay_c = (vy_curr - vy_prev) / dt
                            a_speed = ax_c ** 2 + ay_c ** 2
                            accel_speeds.append(a_speed)
                            accel_candidates.append((ax_c, ay_c))

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
                        min_a_idx = int(np.argmin(accel_speeds))
                        ax, ay = accel_candidates[min_a_idx]

                # Store corner velocities for next frame
                if vel_candidates_scaled is not None:
                    self.prev_corners_vel = vel_candidates_scaled
                else:
                    self.prev_corners_vel = None

                curr_boxes.append((x1, y1, x2, y2))

<<<<<<< HEAD
<<<<<<< HEAD
                # Velocity buffers
=======
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
=======
                # Simple global buffers (you might later do per-ID buffers)
>>>>>>> b4cf006 (upgraded detect_node)
                self.x_vel_buffer.append(vx)
                self.y_vel_buffer.append(vy)
                if len(self.x_vel_buffer) > self.x_vel_buffer_length:
                    self.x_vel_buffer.pop(0)
                if len(self.y_vel_buffer) > self.y_vel_buffer_length:
                    self.y_vel_buffer.pop(0)

<<<<<<< HEAD
<<<<<<< HEAD
                # Detection2D for vision_msgs
                det = Detection2D()
                det.header = msg.header
=======
=======
                # Fill Detection2D message
>>>>>>> b4cf006 (upgraded detect_node)
                det = Detection2D()
                det.header = msg.header

>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = float(bw)
                det.bbox.size_y = float(bh)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)
<<<<<<< HEAD
                det_msg.detections.append(det)

                # Depth projection to 3D
                z = 0.0
                if self.latest_depth is not None:
                    xi = int(np.clip(cx, 0, self.latest_depth.shape[1] - 1))
                    yi = int(np.clip(cy, 0, self.latest_depth.shape[0] - 1))
                    z = float(self.latest_depth[yi, xi])
                if z <= 0.0:
                    z = 0.0

                X_optical = (cx - cx0) * z / fx
                Y_optical = (cy - cy0) * z / fy
                Z_optical = z

                # optical -> base (simple re-map; you can fix with real extrinsics)
                X = Y_optical
                Y = -X_optical
                Z = Z_optical

                # Force ground-projected X if desired
                X = 0.0

                detections_for_tracks.append({'cx': cx, 'cy': cy, 'z': z, 'X': X, 'Y': Y, 'Z': Z})

                # Draw debug
                if debug_img is not None:
                    p1 = (int(x1), int(y1))
                    p2 = (int(x2), int(y2))
                    color = (0, 255, 0)
                    crossing = False

                    # crude "crossing" heuristic based on region + velocity orientation
                    if abs(ax) < 30.0:
                        in_left = self._point_in_triangle(
=======

                det_msg.detections.append(det)

                # Debug drawing and crude "crossing" logic
                if debug_img is not None:
                    p1 = (int(x1), int(y1))
                    p2 = (int(x2), int(y2))

                    color = (0, 255, 0)
                    crossing = False

                    # crude gating on x-acc to ignore crazy jitter
                    if abs(ax) < 30.0:
                        in_left_fan = self._point_in_triangle(
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
                            cx, cy,
                            bottom_left_red[0], bottom_left_red[1],
                            top_center[0], top_center[1],
                            top_left_yellow[0], top_left_yellow[1]
                        )
<<<<<<< HEAD
                        in_right = self._point_in_triangle(
=======
                        in_right_fan = self._point_in_triangle(
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
                            cx, cy,
                            bottom_right_red[0], bottom_right_red[1],
                            top_center[0], top_center[1],
                            top_right_yellow[0], top_right_yellow[1]
                        )
<<<<<<< HEAD
                        if (in_left or in_right) and abs(vy / (vx + 1e-6)) < 0.5:
                            crossing = True
                            color = (0, 0, 255)
                        else:
                            color = (0, 165, 255)

                    cv2.rectangle(debug_img, p1, p2, color, 2)
                    label = f"person {score:.2f}, {'CROSSING' if crossing else 'CAUTION'}"
=======

                        # vy / vx ratio to see if motion is mostly vertical in image
                        if (in_left_fan or in_right_fan) and abs(vy / (vx + 0.001)) < 0.5:
                            crossing = True
                            color = (0, 0, 255)       # red
                        else:
                            color = (0, 165, 255)     # orange

                    cv2.rectangle(debug_img, p1, p2, color, 2)

                    label = (f"person {score:.2f}, "
                             f"{'CROSSING' if crossing else 'CAUTION'}")
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
                    cv2.putText(
                        debug_img,
                        label,
                        (p1[0], max(0, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
<<<<<<< HEAD
                    )

        self.prev_boxes = curr_boxes
        self.prev_time = t_now

        # Update tracks
        deleted_ids = self._update_tracks(detections_for_tracks, w, h)

        # Draw debug regions and paths
        if debug_img is not None:
            cv2.line(debug_img, bottom_left_red, top_center, (0, 0, 255), 2)
            cv2.line(debug_img, bottom_right_red, top_center, (0, 0, 255), 2)
            cv2.line(debug_img, bottom_left_yellow, top_left_yellow, (0, 255, 255), 2)
            cv2.line(debug_img, bottom_right_yellow, top_right_yellow, (0, 255, 255), 2)

            for tid, tr in self.tracks.items():
                if len(tr['path_img']) > 1:
                    pts = np.array(tr['path_img'], dtype=np.int32).reshape(-1, 1, 2)
                    color_table = [(255, 0, 255), (0, 255, 255), (0, 128, 255), (255, 255, 0)]
                    color = color_table[tid % 4]
                    cv2.polylines(debug_img, [pts], isClosed=False, color=color, thickness=2)

        # Publish detections & debug image
        self.pub_dets.publish(det_msg)
=======
                        cv2.LINE_AA,
                    )

                    info_text = (
                        f"vel_x/vel_y ratio={abs(vy/(vx+0.001)):.1f},"
                        f"img_x_vel={vx:.1f}, img_y_vel={vy:.1f}, "
                        f"img_x_accel={ax:.1f}, img_y_accel={ay:.1f}"
                    )
                    text_org = (p1[0], min(h - 5, p2[1] + 20))
                    cv2.putText(
                        debug_img,
                        info_text,
                        text_org,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                    if crossing:
                        print(f"Detection: vx={vx:.2f}, vy={vy:.2f}, ax={ax:.2f}, ay={ay:.2f}, CROSSING")
                    else:
                        print(f"Detection: vx={vx:.2f}, vy={vy:.2f}, ax={ax:.2f}, ay={ay:.2f}, CAUTION")

        # Save for next frame
        self.prev_boxes = curr_boxes
        self.prev_time = t_now

        # Draw fan lines on debug image
        if debug_img is not None:
            cv2.line(debug_img, bottom_left_red, top_center, (0, 0, 255), 2)
            cv2.line(debug_img, bottom_right_red, top_center, (0, 0, 255), 2)

            cv2.line(debug_img, bottom_left_yellow, top_left_yellow, (0, 255, 255), 2)
            cv2.line(debug_img, bottom_right_yellow, top_right_yellow, (0, 255, 255), 2)

        # Publish detections
        self.pub_dets.publish(det_msg)

<<<<<<< HEAD
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
=======
        # Publish debug image
>>>>>>> b4cf006 (upgraded detect_node)
        if debug_img is not None and self.pub_debug is not None:
            out_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_debug.publish(out_msg)

<<<<<<< HEAD
<<<<<<< HEAD
        # Delete markers for dead tracks
        for tid in deleted_ids:
            for ns, pub in [
                ("person", self.pub_marker),
                ("person_path", self.pub_path_marker),
                ("person_prediction", self.pub_prediction_marker),
                ("person_prediction_end", self.pub_prediction_marker),
            ]:
                m = Marker()
                m.header.frame_id = msg.header.frame_id
                m.header.stamp = msg.header.stamp
                m.ns = ns
                m.id = tid
                m.action = Marker.DELETE
                pub.publish(m)

        # ------------------------------------------------------------------
        # Collision checking + safety logic
        # ------------------------------------------------------------------
        vehicle_start = (0.0, 0.0)
        vehicle_end = (0.0, 5.0)

        collision_peds = {}
        for tid, tr in self.tracks.items():
            if 'predicted_path' in tr and len(tr['predicted_path']) > 0:
                intersects, point = self._path_intersects_line(
                    tr['predicted_path'],
                    vehicle_start,
                    vehicle_end,
                    threshold=0.3,
                )
                if intersects:
                    ped_speed = self._get_pedestrian_velocity(tr)
                    ped_vel_vector = self._compute_velocity_vector(tr)
                    ttc, min_dist, entry_pt = self._time_to_collision(
                        tr['predicted_path'],
                        ped_vel_vector,
                        vehicle_width=1.0,
                        vehicle_length=2.0,
                    )
                    pedestrian_slowing = self._is_pedestrian_slowing_down(tr)

                    collision_peds[tid] = {
                        'speed': ped_speed,
                        'ttc': ttc,
                        'slowing': pedestrian_slowing,
                        'track': tr,
                        'min_distance': min_dist,
                        'entry_point': entry_pt,
                    }

        if collision_peds:
            closest_tid, closest_ped = self._get_closest_collision_pedestrian(collision_peds)
            
            if closest_ped is not None:
                primary_cmd = "CONTINUE_SLOW" if closest_ped['slowing'] else "STOP"
                final_cmd = self._safety_filter(
                    primary_cmd,
                    True,
                    closest_ped['ttc'],
                    closest_ped['speed'],
                    closest_ped['slowing'],
                )
                self.get_logger().warn(
                    f"Collision risk: {len(collision_peds)} pedestrians, closest TTC={closest_ped['ttc']:.2f}s"
                )
            else:
                final_cmd = "CONTINUE"
        else:
            final_cmd = "CONTINUE"

        cmd_msg = String()
        cmd_msg.data = final_cmd
        self.pub_vehicle_cmd.publish(cmd_msg)

        # ------------------------------------------------------------------
        # Visualization markers
        # ------------------------------------------------------------------
        for tid, tr in self.tracks.items():
            sx, sy, sz = tr['sx'], tr['sy'], tr['sz']

            # Person position
            m = Marker()
            m.header.frame_id = msg.header.frame_id
            m.header.stamp = msg.header.stamp
            m.ns = "person"
            m.id = tid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = sx
            m.pose.position.y = sy
            m.pose.position.z = sz
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            self.pub_marker.publish(m)

            # Smoothed path
            path = Marker()
            path.header.frame_id = msg.header.frame_id
            path.header.stamp = msg.header.stamp
            path.ns = "person_path"
            path.id = tid
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.05
            path.color.a = 1.0
            path.color.r = 1.0
            path.color.g = 1.0
            path.color.b = 0.0

            if 'smoothed_path' in tr and len(tr['smoothed_path']) > 0:
                path.points = [
                    Point(x=float(px), y=float(py), z=float(pz))
                    for (px, py, pz) in tr['smoothed_path']
                ]
            else:
                path.points = [
                    Point(x=float(px), y=float(py), z=float(pz))
                    for (px, py, pz) in tr['path_3d']
                ]
            self.pub_path_marker.publish(path)

            # Predicted path
            if 'predicted_path' in tr and len(tr['predicted_path']) > 0:
                pred = Marker()
                pred.header.frame_id = msg.header.frame_id
                pred.header.stamp = msg.header.stamp
                pred.ns = "person_prediction"
                pred.id = tid
                pred.type = Marker.LINE_STRIP
                pred.action = Marker.ADD
                pred.pose.position.x = 0.0
                pred.pose.position.y = 0.0
                pred.pose.position.z = 0.0
                pred.pose.orientation.x = 0.0
                pred.pose.orientation.y = 0.0
                pred.pose.orientation.z = 0.0
                pred.pose.orientation.w = 1.0
                pred.scale.x = 0.15
                pred.color.a = 1.0
                pred.color.r = 1.0
                pred.color.g = 0.0
                pred.color.b = 0.0
                pred.lifetime.sec = 0
                pred.lifetime.nanosec = 0
                pred.points = [
                    Point(x=float(px), y=float(py), z=float(pz))
                    for (px, py, pz) in tr['predicted_path']
                ]
                self.pub_prediction_marker.publish(pred)

                # Prediction end marker
                end_marker = Marker()
                end_marker.header.frame_id = msg.header.frame_id
                end_marker.header.stamp = msg.header.stamp
                end_marker.ns = "person_prediction_end"
                end_marker.id = tid
                end_marker.type = Marker.SPHERE
                end_marker.action = Marker.ADD
                end_pos = tr['predicted_path'][-1]
                end_marker.pose.position.x = float(end_pos[0])
                end_marker.pose.position.y = float(end_pos[1])
                end_marker.pose.position.z = float(end_pos[2])
                end_marker.pose.orientation.x = 0.0
                end_marker.pose.orientation.y = 0.0
                end_marker.pose.orientation.z = 0.0
                end_marker.pose.orientation.w = 1.0
                end_marker.scale.x = 0.4
                end_marker.scale.y = 0.4
                end_marker.scale.z = 0.4
                end_marker.color.a = 1.0
                end_marker.color.r = 1.0
                end_marker.color.g = 0.3
                end_marker.color.b = 0.0
                end_marker.lifetime.sec = 0
                end_marker.lifetime.nanosec = 0
                self.pub_prediction_marker.publish(end_marker)

        # Car marker at origin
        car = Marker()
        car.header.frame_id = msg.header.frame_id
        car.header.stamp = msg.header.stamp
        car.ns = "car"
        car.id = 0
        car.type = Marker.CUBE
        car.action = Marker.ADD
        car.pose.position.x = 0.0
        car.pose.position.y = 0.0
        car.pose.position.z = 0.0
        car.pose.orientation.w = 1.0
        car.scale.x = 0.5
        car.scale.y = 0.5
        car.scale.z = 0.5
        car.color.a = 1.0
        car.color.r = 0.0
        car.color.g = 0.0
        car.color.b = 1.0
        self.pub_car_marker.publish(car)

        # FPS logging
=======
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
=======
        # FPS logging (still using wall clock)
>>>>>>> b4cf006 (upgraded detect_node)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now_t = time.time()
            fps = 30.0 / (now_t - self.last_fps_t + 1e-9)
            self.last_fps_t = now_t
            self.get_logger().info(f"~{fps:.1f} FPS")


<<<<<<< HEAD
def main(args=None):
    rclpy.init(args=args)
=======
def main():
    rclpy.init()
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
    node = YoloV11PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
<<<<<<< HEAD
        node.get_logger().info("Shutting down YOLOv11 detector")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
=======
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
>>>>>>> ef80029 (added pedestrian detection and behavior prediction)
