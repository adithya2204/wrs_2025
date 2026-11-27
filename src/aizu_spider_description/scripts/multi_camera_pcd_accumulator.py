import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import math
import time

class HobbyDrone360Mapper(Node):
    def __init__(self):
        super().__init__('hobbydrone_360_mapper')

        # Camera configuration with color coding
        # Add cameras here (only these will be subscribed)
        self.cameras = {
            'Front': {'color': [1.0, 0.0, 0.0]},     # Red
            'Back': {'color': [0.0, 1.0, 0.0]},      # Green
            # If you add Top/Bottom, add them to this dict
            # 'Top': {'color': [0.0, 0.0, 1.0]},
            # 'Bottom': {'color': [1.0, 1.0, 0.0]}
        }

        # Subscribe to camera pointclouds
        self.pc_subs = {}
        for cam_name in self.cameras.keys():
            topic = f'/HobbyDrone/{cam_name}Camera/point_cloud'
            self.pc_subs[cam_name] = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=cam_name: self.pc_callback(msg, name),
                10
            )
            self.get_logger().info(f"✓ Subscribed to {topic}")

        # Accumulated point cloud (Open3D)
        self.accumulated_pcd = o3d.geometry.PointCloud()
        self.voxel_size = 0.02  # 2cm voxel for downsampling

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Keep last accepted transform per camera (4x4 matrix)
        self.last_accepted_pose = {cam: None for cam in self.cameras.keys()}

        # Parameters to avoid overlap (tune these)
        self.min_translation = 0.05    # meters; require >= this to accept new frame
        self.min_rotation_deg = 5.0    # degrees; require >= this to accept new frame

        # Stats
        self.frame_count = 0
        self.downsample_interval = 10  # downsample every N accepted frames (not raw callbacks)
        self.accepted_frames = 0
        self.frames_per_camera = {cam: 0 for cam in self.cameras.keys()}

        self.get_logger().info("=" * 70)
        self.get_logger().info("🚁 HOBBYDRONE 360° MAPPER STARTED")
        self.get_logger().info("Color coding: " + ", ".join([f"{k}" for k in self.cameras.keys()]))
        self.get_logger().info("Press Ctrl+C to save the 360° map")
        self.get_logger().info("=" * 70)

    def pc_callback(self, msg: PointCloud2, camera_name: str):
        """Handle point cloud from any camera"""
        # Try to get transform from msg frame to CHASSIS (target_frame = 'CHASSIS')
        try:
            # Use latest available transform (time = msg.header.stamp)
            trans = self.tf_buffer.lookup_transform(
                'CHASSIS',
                msg.header.frame_id,
                msg.header.stamp  # pass the message stamp for exact transform if available
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF not yet available — skip gracefully
            # Optionally you can buffer the cloud or try again; for simplicity we skip frame
            self.get_logger().debug(f"TF not ready for {camera_name}: {e}")
            return

        # Convert transform to matrix
        pose_mat = self.transform_to_matrix(trans)

        # Pose-change filter: accept only if moved/rotated enough from last accepted pose
        last_pose = self.last_accepted_pose[camera_name]
        if last_pose is not None:
            trans_delta = np.linalg.norm(pose_mat[:3, 3] - last_pose[:3, 3])
            # compute relative rotation in degrees
            r1 = R.from_matrix(last_pose[:3, :3])
            r2 = R.from_matrix(pose_mat[:3, :3])
            rel_rot = (r2 * r1.inv()).magnitude()  # radians
            rot_deg = abs(rel_rot) * 180.0 / math.pi

            if trans_delta < self.min_translation and rot_deg < self.min_rotation_deg:
                # Too similar — reject to avoid heavy overlap
                self.get_logger().debug(f"Skipped {camera_name}: small motion t={trans_delta:.4f} m r={rot_deg:.2f}°")
                return

        # Extract points + rgb robustly using sensor_msgs_py.point_cloud2.read_points
        points = []
        colors = []
        # We'll request x,y,z and optionally rgb/rgba if present
        # read_points yields an iterator of tuples in the order of fields requested when available

        # Determine which fields exist
        field_names = [f.name for f in msg.fields]
        want_rgb = 'rgb' in field_names or 'rgba' in field_names
        # Choose returned fields
        if want_rgb:
            it = point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        else:
            it = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        for p in it:
            # p is tuple of (x,y,z) or (x,y,z,rgb)
            if len(p) >= 3:
                x, y, z = p[0], p[1], p[2]
                # skip invalid points again (just in case)
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                points.append([x, y, z])

                if want_rgb and len(p) >= 4 and p[3] is not None:
                    # rgb comes as float-packed uint32 (same convention as ROS)
                    rgb_packed = int(p[3])
                    r = ((rgb_packed >> 16) & 0xFF) / 255.0
                    g = ((rgb_packed >> 8) & 0xFF) / 255.0
                    b = (rgb_packed & 0xFF) / 255.0
                    colors.append([r, g, b])
                else:
                    # fallback camera color
                    cam_color = self.cameras[camera_name]['color']
                    colors.append(cam_color)
        if len(points) == 0:
            return

        # Build Open3D cloud for this frame
        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
        frame_pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

        # Transform frame to CHASSIS
        frame_pcd.transform(pose_mat)

        # Accept this frame: append to accumulated cloud and update last_accepted_pose
        self.accumulated_pcd += frame_pcd
        self.last_accepted_pose[camera_name] = pose_mat
        self.accepted_frames += 1
        self.frames_per_camera[camera_name] += 1

        # Periodic downsample to keep memory manageable
        if self.accepted_frames % self.downsample_interval == 0:
            self.downsample_and_log()

    def downsample_and_log(self):
        """Downsample accumulated point cloud and log statistics"""
        try:
            if len(self.accumulated_pcd.points) == 0:
                return
            self.accumulated_pcd = self.accumulated_pcd.voxel_down_sample(self.voxel_size)
            # optionally remove obvious outliers quickly
            if len(self.accumulated_pcd.points) > 1000:
                cl, ind = self.accumulated_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                self.accumulated_pcd = self.accumulated_pcd.select_by_index(ind)

            # Build status
            status = f"📊 Points: {len(self.accumulated_pcd.points):,} | Accepted frames: {self.accepted_frames} | Per-cam: "
            status += " ".join([f"{cam}:{self.frames_per_camera[cam]}" for cam in self.cameras.keys()])
            self.get_logger().info(status)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Downsample failed: {e}")

    def transform_to_matrix(self, trans: TransformStamped):
        """Convert ROS TransformStamped to 4x4 transformation matrix"""
        r = trans.transform.rotation
        t = trans.transform.translation
        rot = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        mat = np.eye(4, dtype=float)
        mat[:3, :3] = rot
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def save_accumulated_pcd(self):
        """Save the final 360° point cloud map"""
        if len(self.accumulated_pcd.points) == 0:
            self.get_logger().error("❌ No points to save!")
            return

        self.get_logger().info("🔄 Processing final 360° map...")

        # Final downsample
        try:
            self.accumulated_pcd = self.accumulated_pcd.voxel_down_sample(self.voxel_size)
            self.get_logger().info(f"   After downsample: {len(self.accumulated_pcd.points):,} points")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Final downsample failed: {e}")

        # Remove statistical outliers
        try:
            if len(self.accumulated_pcd.points) > 100:
                cl, ind = self.accumulated_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                self.accumulated_pcd = self.accumulated_pcd.select_by_index(ind)
                self.get_logger().info(f"   After outlier removal: {len(self.accumulated_pcd.points):,} points")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Outlier removal failed: {e}")

        # Optionally run a final mesh / surface reconstruction step here if needed

        # Save in multiple formats
        base_filename = 'HobbyDrone_360_Map'
        pcd_file = f'{base_filename}.pcd'
        o3d.io.write_point_cloud(pcd_file, self.accumulated_pcd, write_ascii=False)
        ply_file = f'{base_filename}.ply'
        o3d.io.write_point_cloud(ply_file, self.accumulated_pcd)
        self.get_logger().info(f"✅ Saved: {pcd_file}")
        self.get_logger().info(f"✅ Saved: {ply_file}")

        # Summary
        self.get_logger().info("=" * 70)
        self.get_logger().info("🎉 360° MAP SUCCESSFULLY SAVED!")
        self.get_logger().info("📷 Camera frames captured:")
        for cam in self.cameras.keys():
            self.get_logger().info(f"   {cam}: {self.frames_per_camera[cam]} frames")
        self.get_logger().info(f"📊 Total points in map: {len(self.accumulated_pcd.points):,}")
        self.get_logger().info("=" * 70)


def main():
    rclpy.init()
    node = HobbyDrone360Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Stopping capture...")
        node.save_accumulated_pcd()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()