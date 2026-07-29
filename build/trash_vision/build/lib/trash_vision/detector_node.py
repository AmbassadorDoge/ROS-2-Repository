import json
import queue
import threading
import time

import cv2
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


class TrashDetector(Node):
    def __init__(self) -> None:
        super().__init__("trash_detector")

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "model_path",
                    "/home/christopherg/best_ncnn_model",
                ),
                (
                    "camera_source",
                    "tcp://127.0.0.1:8888",
                ),
                ("confidence", 0.40),
                ("deadband", 0.12),
                (
                    "mapping_upload_url",
                    "http://192.168.0.102:5000/frame",
                ),
                ("mapping_enabled", True),
                ("mapping_interval_seconds", 2.0),
                ("mapping_jpeg_quality", 80),
            ],
        )

        model_path = str(
            self.get_parameter("model_path").value
        )
        camera_source = str(
            self.get_parameter("camera_source").value
        )

        self.confidence = float(
            self.get_parameter("confidence").value
        )
        self.deadband = float(
            self.get_parameter("deadband").value
        )

        self.mapping_upload_url = str(
            self.get_parameter("mapping_upload_url").value
        )
        self.mapping_enabled = bool(
            self.get_parameter("mapping_enabled").value
        )
        self.mapping_interval_seconds = float(
            self.get_parameter(
                "mapping_interval_seconds"
            ).value
        )
        self.mapping_jpeg_quality = int(
            self.get_parameter(
                "mapping_jpeg_quality"
            ).value
        )

        self.target_publisher = self.create_publisher(
            String,
            "/vision/target",
            10,
        )
        self.direction_publisher = self.create_publisher(
            String,
            "/vision/direction",
            10,
        )

        self.get_logger().info(
            f"Loading YOLO model: {model_path}"
        )
        self.model = YOLO(model_path)

        self.get_logger().info(
            f"Opening camera source: {camera_source}"
        )
        self.camera = cv2.VideoCapture(camera_source)

        if not self.camera.isOpened():
            raise RuntimeError(
                f"Could not open camera source: {camera_source}"
            )

        self.frame_number = 0
        self.fps_frame_count = 0
        self.fps_start_time = time.monotonic()

        self.last_mapping_upload_time = 0.0
        self.mapping_queue: queue.Queue = queue.Queue(
            maxsize=1
        )
        self.mapping_stop_event = threading.Event()

        self.mapping_thread = threading.Thread(
            target=self.mapping_upload_worker,
            name="lingbot-frame-uploader",
            daemon=True,
        )
        self.mapping_thread.start()

        self.timer = self.create_timer(
            0.001,
            self.process_frame,
        )

        self.get_logger().info(
            "Trash detector node started"
        )

        if self.mapping_enabled:
            self.get_logger().info(
                "LingBot frame uploading enabled: "
                f"{self.mapping_upload_url}"
            )
        else:
            self.get_logger().info(
                "LingBot frame uploading disabled"
            )

    def process_frame(self) -> None:
        success, frame = self.camera.read()

        if not success:
            self.get_logger().warning(
                "Camera frame unavailable",
                throttle_duration_sec=2.0,
            )
            return

        self.frame_number += 1
        self.fps_frame_count += 1

        self.queue_mapping_frame(frame)

        height, width = frame.shape[:2]

        results = self.model.predict(
            source=frame,
            imgsz=640,
            conf=self.confidence,
            verbose=False,
        )

        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            self.publish_no_target()
            self.report_processing_rate()
            return

        confidences = boxes.conf.cpu().numpy()
        best_index = int(confidences.argmax())

        x1, y1, x2, y2 = (
            boxes.xyxy[best_index]
            .cpu()
            .numpy()
            .tolist()
        )

        confidence = float(
            confidences[best_index]
        )

        center_x_pixels = (x1 + x2) / 2.0
        center_y_pixels = (y1 + y2) / 2.0

        center_x = center_x_pixels / width
        center_y = center_y_pixels / height

        floor_x = center_x_pixels / width
        floor_y = y2 / height

        horizontal_error = center_x - 0.5
        direction = self.get_direction(
            horizontal_error
        )

        target = {
            "detected": True,
            "frame_number": self.frame_number,
            "confidence": round(confidence, 4),
            "bbox_pixels": {
                "x1": round(x1, 1),
                "y1": round(y1, 1),
                "x2": round(x2, 1),
                "y2": round(y2, 1),
            },
            "center_normalized": {
                "x": round(center_x, 4),
                "y": round(center_y, 4),
            },
            "floor_point_normalized": {
                "x": round(floor_x, 4),
                "y": round(floor_y, 4),
            },
            "horizontal_error": round(
                horizontal_error,
                4,
            ),
            "direction": direction,
        }

        self.publish_target(target, direction)
        self.report_processing_rate(
            direction=direction,
            confidence=confidence,
        )

    def get_direction(
        self,
        horizontal_error: float,
    ) -> str:
        if horizontal_error < -self.deadband:
            return "LEFT"

        if horizontal_error > self.deadband:
            return "RIGHT"

        return "CENTER"

    def publish_target(
        self,
        target: dict,
        direction: str,
    ) -> None:
        target_message = String()
        target_message.data = json.dumps(target)
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = direction
        self.direction_publisher.publish(
            direction_message
        )

    def publish_no_target(self) -> None:
        target_message = String()
        target_message.data = json.dumps(
            {
                "detected": False,
                "frame_number": self.frame_number,
            }
        )
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = "NO_TARGET"
        self.direction_publisher.publish(
            direction_message
        )

    def report_processing_rate(
        self,
        direction: str | None = None,
        confidence: float | None = None,
    ) -> None:
        if self.fps_frame_count < 30:
            return

        current_time = time.monotonic()
        elapsed_time = (
            current_time - self.fps_start_time
        )

        if elapsed_time <= 0:
            return

        processing_rate = (
            self.fps_frame_count / elapsed_time
        )

        message = (
            f"processing_rate={processing_rate:.1f} FPS"
        )

        if direction is not None:
            message += f" target={direction}"

        if confidence is not None:
            message += (
                f" confidence={confidence:.2f}"
            )

        self.get_logger().info(message)

        self.fps_frame_count = 0
        self.fps_start_time = current_time

    def queue_mapping_frame(
        self,
        frame,
    ) -> None:
        if not self.mapping_enabled:
            return

        current_time = time.monotonic()

        elapsed_time = (
            current_time
            - self.last_mapping_upload_time
        )

        if (
            elapsed_time
            < self.mapping_interval_seconds
        ):
            return

        self.last_mapping_upload_time = current_time

        frame_copy = frame.copy()
        queued_item = (
            self.frame_number,
            time.time(),
            frame_copy,
        )

        try:
            self.mapping_queue.put_nowait(
                queued_item
            )
            return
        except queue.Full:
            pass

        try:
            self.mapping_queue.get_nowait()
            self.mapping_queue.task_done()
        except queue.Empty:
            pass

        try:
            self.mapping_queue.put_nowait(
                queued_item
            )
        except queue.Full:
            self.get_logger().warning(
                "Mapping queue remained full; "
                "dropping frame",
                throttle_duration_sec=5.0,
            )

    def mapping_upload_worker(self) -> None:
        session = requests.Session()

        while not self.mapping_stop_event.is_set():
            try:
                (
                    frame_number,
                    timestamp,
                    frame,
                ) = self.mapping_queue.get(
                    timeout=0.5
                )
            except queue.Empty:
                continue

            try:
                encode_success, encoded_frame = (
                    cv2.imencode(
                        ".jpg",
                        frame,
                        [
                            cv2.IMWRITE_JPEG_QUALITY,
                            self.mapping_jpeg_quality,
                        ],
                    )
                )

                if not encode_success:
                    self.get_logger().warning(
                        "Could not encode mapping frame"
                    )
                    continue

                timestamp_ms = int(
                    timestamp * 1000
                )

                filename = (
                    f"frame_{timestamp_ms}_"
                    f"{frame_number:08d}.jpg"
                )

                response = session.post(
                    self.mapping_upload_url,
                    files={
                        "frame": (
                            filename,
                            encoded_frame.tobytes(),
                            "image/jpeg",
                        )
                    },
                    data={
                        "filename": filename,
                        "frame_number": str(
                            frame_number
                        ),
                        "timestamp": str(timestamp),
                    },
                    timeout=(2.0, 10.0),
                )

                response.raise_for_status()

                self.get_logger().info(
                    f"Uploaded mapping frame: "
                    f"{filename}"
                )

            except requests.RequestException as error:
                self.get_logger().warning(
                    "Mapping upload failed: "
                    f"{error}",
                    throttle_duration_sec=5.0,
                )

            except Exception as error:
                self.get_logger().error(
                    "Unexpected mapping uploader "
                    f"error: {error}"
                )

            finally:
                self.mapping_queue.task_done()

        session.close()

    def destroy_node(self) -> None:
        self.get_logger().info(
            "Shutting down trash detector"
        )

        self.mapping_stop_event.set()

        if self.mapping_thread.is_alive():
            self.mapping_thread.join(
                timeout=2.0
            )

        if self.camera.isOpened():
            self.camera.release()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = TrashDetector()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().fatal(
                f"Fatal node error: {error}"
            )
        else:
            print(
                f"Failed to initialize node: {error}"
            )

        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
