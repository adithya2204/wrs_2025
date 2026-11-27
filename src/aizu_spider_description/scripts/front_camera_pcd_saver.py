#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

class OptimizedFrontPCDSaver(Node):
    def __init__(self):
        super().__init__('optimized_front_pcd_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/HobbyDrone/BottomCamera/point_cloud',
            self.pc_callback,
            10
        )
        self.accumulated_pcd = o3d.geometry.PointCloud()
        self.voxel_size = 0.01  # 1 cm voxel
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_ready = False
        self.transform_matrix = None
        self.frame_count = 0
        self.downsample_interval = 5  # Downsample accumulated cloud every N frames
        self.get_logger().info("Optimized FrontCamera PCD Saver started!")

    def pc_callback(self, msg: PointCloud2):
        # Lazy initialization of static transform
        if not self.transform_ready:
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    'CHASSIS', msg.header.frame_id, rclpy.time.Time())
                self.transform_matrix = self.transform_to_matrix(trans)
                self.transform_ready = True
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().warn("Transform not available yet, skipping frame")
                return

        points, colors = self.extract_points_and_rgb(msg)
        if len(points) == 0:
            return

        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            frame_pcd.colors = o3d.utility.Vector3dVector(colors)

        # Transform to base frame
        frame_pcd.transform(self.transform_matrix)

        # Accumulate
        self.accumulated_pcd += frame_pcd
        self.frame_count += 1

        # KEY FIX: Downsample the accumulated cloud periodically to remove overlaps
        if self.frame_count % self.downsample_interval == 0:
            try:
                self.accumulated_pcd = self.accumulated_pcd.voxel_down_sample(self.voxel_size)
                self.get_logger().info(f"Downsampled accumulated cloud. Points: {len(self.accumulated_pcd.points)}")
            except Exception as e:
                self.get_logger().warn(f"Failed to downsample: {e}")

    def extract_points_and_rgb(self, msg: PointCloud2):
        points = []
        colors = []
        fmt = 'fff'  # x, y, z floats
        step = msg.point_step
        for i in range(msg.width * msg.height):
            offset = i * step
            try:
                x, y, z = struct.unpack_from(fmt, msg.data, offset)
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue
                points.append([x, y, z])
                # RGB extraction
                if 'rgb' in [f.name for f in msg.fields]:
                    rgb_offset = [f.offset for f in msg.fields if f.name=='rgb'][0]
                    rgb_bytes = struct.unpack_from('I', msg.data, offset + rgb_offset)[0]
                    r = ((rgb_bytes >> 16) & 0xFF) / 255.0
                    g = ((rgb_bytes >> 8) & 0xFF) / 255.0
                    b = (rgb_bytes & 0xFF) / 255.0
                    colors.append([r, g, b])
            except struct.error:
                continue
        return np.array(points), np.array(colors) if colors else None

    def transform_to_matrix(self, trans: TransformStamped):
        import math
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        r = trans.transform.rotation
        t = trans.transform.translation
        rot = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def save_accumulated_pcd(self):
        if len(self.accumulated_pcd.points) == 0:
            self.get_logger().warn("No points to save!")
            return
        
        # Final downsample before saving
        try:
            self.accumulated_pcd = self.accumulated_pcd.voxel_down_sample(self.voxel_size)
            self.get_logger().info(f"Final point count: {len(self.accumulated_pcd.points)}")
        except Exception as e:
            self.get_logger().warn(f"Final downsample failed: {e}")
        
        filename = 'FrontCamera_accumulated.pcd'
        o3d.io.write_point_cloud(filename, self.accumulated_pcd, write_ascii=True)
        self.get_logger().info(f"Saved accumulated PCD: {filename}")

def main():
    rclpy.init()
    node = OptimizedFrontPCDSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected, saving accumulated PCD...")
        node.save_accumulated_pcd()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()