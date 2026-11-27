#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int64
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import queue
import threading
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

# PyTorch + models
import torch
from torchvision import transforms, models
from PIL import Image as PILImage

# YOLO
from ultralytics import YOLO

class EnhancedImageSubscriber(Node):

    def __init__(self):
        super().__init__("perception_node")

        # ---------- PARAMETERS ----------
        self.declare_parameter("camera_topics", [
            "/HobbyDrone/FrontCamera",
            "/HobbyDrone/BackCamera",
            "/HobbyDrone/BottomCamera",
            "/HobbyDrone/TopCamera",
            "/AizuSpider/ARM2_CAMERA"
        ])
        self.declare_parameter("yolo_conf", 0.8)
        self.declare_parameter("enable_gui", True)

        self.available_topics = self.get_parameter("camera_topics").value
        self.yolo_conf = self.get_parameter("yolo_conf").value
        self.enable_gui = self.get_parameter("enable_gui").value

        self.current_topic_index = 0
        self.current_topic = self.available_topics[self.current_topic_index]

        # ---------- MODEL PATHS ----------
        pkg_share = get_package_share_directory("cv_wrs")
        self.yolo_path = os.path.join(pkg_share, "models", "valve_detector.pt")
        self.regression_model_path = os.path.join(pkg_share, "models", "gauge_regressor.pth")

        # ---------- PUBLISHERS ----------
        self.combined_pub = self.create_publisher(Image, "/enhanced/combined_detection", 10)
        self.qr_data_pub = self.create_publisher(String, "/detection/qr_data", 10)
        self.gauge_reading_pub = self.create_publisher(Int64, "/detection/gauge_reading", 10)

        self.br = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)

        if self.enable_gui:
            self.gui_thread = threading.Thread(target=self._gui_loop, daemon=True)
            self.gui_thread.start()

        # ---------- QR ----------
        self.qr_detector = cv2.QRCodeDetector()
        self.current_qr_data = ""
        self.qr_display_time = 0
        self.qr_display_duration = 90

        # ---------- GAUGE ----------
        self.current_frame = None
        self.last_gauge_reading = None
        self.gauge_display_time = 0
        self.gauge_display_duration = 150

        # ---------- YOLO ----------
        try:
            self.yolo_model = YOLO(self.yolo_path) if os.path.exists(self.yolo_path) else None
            self.get_logger().info(f"YOLO loaded: {self.yolo_path}")
        except Exception as e:
            self.get_logger().error(f"YOLO load error: {e}")
            self.yolo_model = None

        # ---------- REGRESSION ----------
        self.setup_regression_model()

        # ---------- CAMERA ----------
        self.create_camera_subscription()

        self.get_logger().info("Perception node ready")

    # ---------------------------------------------------
    def setup_regression_model(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.test_transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
        ])

        class RegressionModel(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.backbone = models.efficientnet_b0(weights=None)
                feature_dim = self.backbone.classifier[1].in_features
                self.backbone.classifier = torch.nn.Sequential(
                    torch.nn.Dropout(0.2),
                    torch.nn.Linear(feature_dim, 256),
                    torch.nn.ReLU(),
                    torch.nn.Dropout(0.1),
                    torch.nn.Linear(256, 64),
                    torch.nn.ReLU(),
                    torch.nn.Linear(64, 1),
                    torch.nn.Sigmoid()
                )

            def forward(self, x):
                return self.backbone(x).squeeze(-1)

        try:
            self.reg_model = RegressionModel().to(self.device)
            self.reg_model.load_state_dict(torch.load(self.regression_model_path, map_location=self.device))
            self.reg_model.eval()
            self.get_logger().info("Regression model loaded")
        except Exception as e:
            self.get_logger().error(f"Regression model error: {e}")
            self.reg_model = None

    # ---------------------------------------------------
    def create_camera_subscription(self):
        if hasattr(self, "subscription"):
            try:
                self.destroy_subscription(self.subscription)
            except Exception:
                pass
        self.subscription = self.create_subscription(Image, self.current_topic, self.listener_callback, 2)
        self.get_logger().info(f"Subscribed to: {self.current_topic}")

    # ---------------------------------------------------
    def switch_camera(self, direction=1):
        self.current_topic_index = (self.current_topic_index + direction) % len(self.available_topics)
        self.current_topic = self.available_topics[self.current_topic_index]
        self.create_camera_subscription()

    # ---------------------------------------------------
    def enhance_image(self, img):
        try:
            imgf = img.astype(np.float32) / 255.0
            illum = np.max(imgf, axis=2)
            illum_smooth = cv2.blur(illum, (15, 15))
            illum_smooth = np.clip(illum_smooth, 0.1, 1.0)
            enhanced = imgf / illum_smooth[:,:,None]
            enhanced = np.clip(enhanced * 255.0, 0, 255).astype(np.uint8)
            return cv2.fastNlMeansDenoisingColored(enhanced, None, 10, 10, 7, 21)
        except Exception:
            return img

    # ---------------------------------------------------
    def detect_qr(self, frame, display_frame):
        try:
            data, bbox, _ = self.qr_detector.detectAndDecode(frame)
            if data:
                self.current_qr_data = data
                self.qr_display_time = self.qr_display_duration

                qr_msg = String()
                qr_msg.data = json.dumps({"timestamp": str(datetime.now()), "qr_code": data})
                self.qr_data_pub.publish(qr_msg)

            return data
        except Exception:
            return None

    # ---------------------------------------------------
    def detect_objects(self, frame, display_frame):
        if not self.yolo_model:
            return []

        try:
            results = self.yolo_model(frame, conf=self.yolo_conf)
            detections = []
            for res in results:
                boxes = getattr(res, "boxes", None)
                if boxes is None:
                    continue
                for box in boxes:
                    x1,y1,x2,y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    name = self.yolo_model.names[cls_id]
                    detections.append({"class": name,"confidence": conf,"bbox":[x1,y1,x2,y2]})
            return detections
        except Exception:
            return []

    # ---------------------------------------------------
    def get_gauge_reading(self, frame):
        if self.reg_model is None:
            return None
        try:
            img_pil = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            tensor = self.test_transform(img_pil).unsqueeze(0).to(self.device)
            with torch.no_grad():
                prediction = self.reg_model(tensor).item()

            msg = Int64()
            msg.data = int(prediction * 1000000)
            self.gauge_reading_pub.publish(msg)
            return prediction
        except Exception:
            return None

    # ---------------------------------------------------
    def listener_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            enhanced = self.enhance_image(frame)
            display_frame = enhanced.copy()

            self.detect_qr(enhanced, display_frame)
            self.detect_objects(enhanced, display_frame)

            combined_msg = self.br.cv2_to_imgmsg(display_frame, "bgr8")
            combined_msg.header = msg.header
            self.combined_pub.publish(combined_msg)

            if self.enable_gui and not self.frame_queue.full():
                self.frame_queue.put_nowait(display_frame)

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

    # ---------------------------------------------------
    def _gui_loop(self):
        cv2.namedWindow("Perception", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                cv2.imshow("Perception", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.switch_camera(1)
                elif key == ord('b'):
                    self.switch_camera(-1)
                elif key == ord('p'):
                    if self.current_frame is not None:
                        reading = self.get_gauge_reading(self.current_frame)
                        if reading:
                            self.get_logger().info(f"Gauge reading: {reading:.4f}")

            except queue.Empty:
                pass
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
