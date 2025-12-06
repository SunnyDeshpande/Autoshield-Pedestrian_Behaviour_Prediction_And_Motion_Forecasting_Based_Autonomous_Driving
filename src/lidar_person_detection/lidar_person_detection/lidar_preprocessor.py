
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from detected_object_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Vector3
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import colorsys
import math

class LidarObjectDetector(Node):
    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.declare_parameter('crop_min_x', 0.0)
        self.declare_parameter('crop_max_x', 0.0)
        self.declare_parameter('crop_min_y', 0.0)
        self.declare_parameter('crop_max_y', 0.0)
        self.declare_parameter('crop_min_z', 0.0)
        self.declare_parameter('crop_max_z', 0.0)
        self.declare_parameter('voxel_size', 0.0)
        self.declare_parameter('sor_nb_neighbors', 0)
        self.declare_parameter('sor_std_ratio', 0.0)
        self.declare_parameter('ground_z_threshold', 0.0)

        self.declare_parameter('dbscan_eps', 0.0)
        self.declare_parameter('dbscan_min_points', 0)
        self.declare_parameter('track_max_distance', 0.0)
        self.declare_parameter('track_max_age', 0)
        self.declare_parameter('track_min_hits', 0)
        self.declare_parameter('ema_alpha', 0.0)

        self.sub = self.create_subscription(PointCloud2, '/ouster/points', self.callback, 10)
        self.pub_processed = self.create_publisher(PointCloud2, '/processed_points', 10)
        self.pub_clustered = self.create_publisher(PointCloud2, '/clustered_points', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cluster_markers', 10)
        self.pub_objects = self.create_publisher(DetectedObjectArray, '/detected_objects', 10)

        self.tracker = SimpleClusterTracker(self)
        self.get_logger().info("Lidar Object Detector with Tracking STARTED")

    def callback(self, msg: PointCloud2):
        try:
            #  loading point cloud
            raw_pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if isinstance(raw_pts, np.ndarray):
                if raw_pts.size == 0:
                    return
                if raw_pts.dtype.names is not None:
                    points_np = np.vstack([raw_pts[name] for name in ('x', 'y', 'z')]).T.astype(np.float64)
                else:
                    points_np = raw_pts.astype(np.float64).reshape(-1, 3)
            else:
                pts_list = list(raw_pts)
                if len(pts_list) == 0:
                    return
                points_np = np.array(pts_list, dtype=np.float64)


            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)

            # Cropping
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                [self.get_parameter('crop_min_x').value, self.get_parameter('crop_min_y').value, self.get_parameter('crop_min_z').value],
                [self.get_parameter('crop_max_x').value, self.get_parameter('crop_max_y').value, self.get_parameter('crop_max_z').value]
            )
            pcd = pcd.crop(bbox)
            if len(pcd.points) < 10: return

            # Downsampling
            pcd = pcd.voxel_down_sample(self.get_parameter('voxel_size').value)

            # Statistical Outlier Removal
            _, ind = pcd.remove_statistical_outlier(
                nb_neighbors=self.get_parameter('sor_nb_neighbors').value,
                std_ratio=self.get_parameter('sor_std_ratio').value
            )
            pcd = pcd.select_by_index(ind)

            # Ground Removal
            points = np.asarray(pcd.points)
            points = points[points[:, 2] > self.get_parameter('ground_z_threshold').value]
            if len(points) < 10: return

            # Publish processed point cloud
            header = msg.header
            header.stamp = self.get_clock().now().to_msg()
            self.pub_processed.publish(pc2.create_cloud_xyz32(header, points.astype(np.float32)))

            # Clustering
            clusters = self.cluster_with_dbscan(pcd)
            # Tracking
            self.tracker.update(clusters, header)

        except Exception as e:
            self.get_logger().error(f"Error: {e}", throttle_duration_sec=5)

    def cluster_with_dbscan(self, pcd):
        eps = self.get_parameter('dbscan_eps').value
        min_pts = self.get_parameter('dbscan_min_points').value
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
            labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_pts, print_progress=False))

        clusters = []
        for label in np.unique(labels):
            if label == -1: continue
            pts = np.asarray(pcd.points)[labels == label]
            if len(pts) < min_pts: continue
            clusters.append({'points': pts, 'centroid': pts.mean(axis=0)})
        return clusters


class SimpleClusterTracker:
    def __init__(self, node):
        self.node = node
        self.tracks = {}
        self.next_id = 0
        self.colors = {}

    def get_color(self, tid):
        if tid not in self.colors:
            h = (tid * 37) % 360 / 360.0
            r, g, b = colorsys.hsv_to_rgb(h, 0.8, 0.9)
            self.colors[tid] = (r, g, b)
        return self.colors[tid]

    def update(self, clusters, header):
        centroids = [c['centroid'] for c in clusters]
        assignments = []
        used = set()

        # Track cluster
        for cent in centroids:
            best_id = None
            best_dist = self.node.get_parameter('track_max_distance').value
            for tid, tr in self.tracks.items():
                if tid in used: continue
                d = np.linalg.norm(tr['centroid'] - cent)
                if d < best_dist:
                    best_dist = d
                    best_id = tid
            if best_id is not None:
                assignments.append(best_id)
                used.add(best_id)
                # Exponential Moving Average smoothing
                a = self.node.get_parameter('ema_alpha').value
                self.tracks[best_id]['centroid'] = a * cent + (1 - a) * self.tracks[best_id]['centroid']
                self.tracks[best_id]['hits'] += 1
                self.tracks[best_id]['age'] = 0
            else:
                assignments.append(self.next_id)
                self.tracks[self.next_id] = {'centroid': cent.copy(), 'hits': 1, 'age': 0}
                self.next_id += 1

        # Age management
        for tid in list(self.tracks.keys()):
            if tid not in used:
                self.tracks[tid]['age'] += 1
                if self.tracks[tid]['age'] > self.node.get_parameter('track_max_age').value:
                    del self.tracks[tid]

        self.publish_visualization(assignments, clusters, header)
        self.publish_detected_objects(assignments, clusters, header)

    def publish_visualization(self, assignments, clusters, header):
        marker_array = MarkerArray()
        colored_points = []
        active_ids = set()

        for cluster, track_id in zip(clusters, assignments):
            track = self.tracks[track_id]
            if track['hits'] < self.node.get_parameter('track_min_hits').value:
                continue

            active_ids.add(track_id)
            centroid = track['centroid']
            r, g, b = self.get_color(track_id)

            # Colored point cloud
            pts = cluster['points'].copy()
            rgb_packed = (int(r*255) << 16) | (int(g*255) << 8) | int(b*255)
            pts_rgb = np.zeros((len(pts), 4))
            pts_rgb[:, :3] = pts
            pts_rgb[:, 3] = rgb_packed
            colored_points.append(pts_rgb)

            # Text marker
            text = Marker()
            text.header = header
            text.ns = "id_text"
            text.id = track_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.MODIFY
            text.pose.position.x = centroid[0]
            text.pose.position.y = centroid[1]
            text.pose.position.z = centroid[2] + 1.0
            text.text = str(track_id)
            text.scale.z = 0.8
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
            text.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # short lifetime
            marker_array.markers.append(text)

            # Center sphere
            sphere = Marker()
            sphere.header = header
            sphere.ns = "center"
            sphere.id = track_id + 10000
            sphere.type = Marker.SPHERE
            sphere.action = Marker.MODIFY
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = centroid
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0
            sphere.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(sphere)

        # Delete old markers
        for tid in self.tracks:
            if tid not in active_ids:
                for ns, base in [("id_text", 0), ("center", 10000)]:
                    del_marker = Marker()
                    del_marker.header = header
                    del_marker.ns = ns
                    del_marker.id = tid + base
                    del_marker.action = Marker.DELETE
                    marker_array.markers.append(del_marker)

        # Publish colored point cloud
        if colored_points:
            all_arr = np.vstack(colored_points)
            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
            structured = np.zeros(all_arr.shape[0], dtype=dtype)
            structured['x'] = all_arr[:,0]
            structured['y'] = all_arr[:,1]
            structured['z'] = all_arr[:,2]
            structured['rgb'] = all_arr[:,3].astype(np.uint32)

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
            ]
            cloud_msg = pc2.create_cloud(header, fields, structured)
            self.node.pub_clustered.publish(cloud_msg)

        self.node.pub_markers.publish(marker_array)

    def publish_detected_objects(self, assignments, clusters, header):
        arr = DetectedObjectArray()
        arr.header = header
        for cluster, tid in zip(clusters, assignments):
            track = self.tracks[tid]
            if track['hits'] < self.node.get_parameter('track_min_hits').value:
                continue
            obj = DetectedObject()
            obj.id = tid
            obj.center = Vector3(x=float(-track['centroid'][0]), # Negate X
                                 y=float(-track['centroid'][1]), # Negate Y
                                 z=float(track['centroid'][2] + 2.0)) # offset height
            obj.point_count = len(cluster['points'])
            dx, dy = -track['centroid'][0], -track['centroid'][1]
            obj.distance = math.hypot(dx, dy)
            obj.angle_deg = math.degrees(math.atan2(dy, dx))
            arr.objects.append(obj)
        self.node.pub_objects.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()