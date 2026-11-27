#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
import psutil
import os
import math

class FrontCloudRecorder(Node):

    def __init__(self):
        super().__init__('front_cloud_recorder')

        # === CONFIG ===
        self.topic = '/HobbyDrone/FrontCamera/point_cloud'
        self.target_frame = 'CHASSIS'
        self.voxel = 0.02          # 2 cm resolution
        self.skip_translation = 0.05   # meters
        self.skip_rotation = 5.0       # degrees
        self.downsample_interval = 5
        self.log_interval = 3.0

        # === TF ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_pose = None

        # === Cloud ===
        self.cloud = o3d.geometry.PointCloud()
        self.accepted_frames = 0

        # === SUB ===
        self.create_subscription(PointCloud2, self.topic, self.callback, 10)

        # === TIMERS ===
        self.create_timer(self.log_interval, self.print_status)

        self.get_logger().info("Front Cloud Recorder started")
        self.get_logger().info("Press Ctrl+C anytime to save map")

    # -----------------------------

    def callback(self, msg: PointCloud2):

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        pose = self.tf_to_matrix(tf)

        # Skip if robot hasn't moved enough
        if self.last_pose is not None:
            t = np.linalg.norm(pose[:3,3] - self.last_pose[:3,3])
            r1 = R.from_matrix(self.last_pose[:3,:3])
            r2 = R.from_matrix(pose[:3,:3])
            rot = (r2 * r1.inv()).magnitude()*180/math.pi
            if t < self.skip_translation and rot < self.skip_rotation:
                return

        # Convert points
        pts = []
        for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            pts.append([p[0],p[1],p[2]])

        if not pts:
            return

        frame = o3d.geometry.PointCloud()
        frame.points = o3d.utility.Vector3dVector(np.asarray(pts))
        frame.transform(pose)

        self.cloud += frame
        self.last_pose = pose
        self.accepted_frames += 1

        # Periodic voxel filter
        if self.accepted_frames % self.downsample_interval == 0:
            self.cloud = self.cloud.voxel_down_sample(self.voxel)

    # -----------------------------

    def tf_to_matrix(self, trans: TransformStamped):
        r = trans.transform.rotation
        t = trans.transform.translation
        rot = R.from_quat([r.x,r.y,r.z,r.w]).as_matrix()
        m = np.eye(4)
        m[:3,:3] = rot
        m[:3,3] = [t.x,t.y,t.z]
        return m

    # -----------------------------

    def print_status(self):
        ram = psutil.Process(os.getpid()).memory_info().rss / (1024**3)
        pts = len(self.cloud.points)
        self.get_logger().info(
            f" Points: {pts:,} | Frames: {self.accepted_frames} | RAM: {ram:.2f} GB"
        )

    # -----------------------------

    def save(self):
        if len(self.cloud.points)==0:
            self.get_logger().warn("No cloud to save")
            return

        self.get_logger().info("Saving map...")
        self.cloud = self.cloud.voxel_down_sample(self.voxel)
        o3d.io.write_point_cloud("FrontCloud_Map.pcd", self.cloud)
        self.get_logger().info(" Saved FrontCloud_Map.pcd")

# -----------------------------

def main():
    rclpy.init()
    node = FrontCloudRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping capture, saving map...")
        node.save_cloud()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()