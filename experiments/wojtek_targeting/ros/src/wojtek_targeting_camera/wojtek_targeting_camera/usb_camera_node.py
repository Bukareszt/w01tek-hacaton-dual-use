"""USB webcam -> image_raw + camera_info, for the targeting camera.

Deliberately thin. The only job is to get frames onto a topic with as little
latency and CPU as the RPi can manage, because everything interesting happens
downstream (YOLO, then gimbal_bridge).
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera")

        self._device = self.declare_parameter("device", "/dev/video0").value
        self._width = self.declare_parameter("image_width", 640).value
        self._height = self.declare_parameter("image_height", 480).value
        self._framerate = self.declare_parameter("framerate", 30.0).value
        self._fourcc = self.declare_parameter("fourcc", "MJPG").value
        self._frame_id = self.declare_parameter(
            "frame_id", "targeting_camera_optical_frame").value

        # Intrinsics, if this camera has ever been calibrated. gimbal_bridge
        # needs fx/fy to turn a bbox offset into a pan/tilt angle -- with them
        # left at zero it has nothing to work from, so say so loudly once
        # rather than letting it silently aim at the wrong place.
        self._fx = self.declare_parameter("fx", 0.0).value
        self._fy = self.declare_parameter("fy", 0.0).value
        self._cx = self.declare_parameter("cx", 0.0).value
        self._cy = self.declare_parameter("cy", 0.0).value
        # Five zeros rather than an empty default: rclpy cannot infer the type
        # of an empty list and refuses to declare the parameter at all.
        self._distortion = list(
            self.declare_parameter("distortion", [0.0] * 5).value)

        self._image_pub = self.create_publisher(
            Image, "~/image_raw", qos_profile_sensor_data)
        self._info_pub = self.create_publisher(
            CameraInfo, "~/camera_info", qos_profile_sensor_data)

        self._capture = self._open_capture()
        self._read_failures = 0
        self.create_timer(1.0 / self._framerate, self._tick)

    def _open_capture(self):
        # CAP_V4L2 explicitly: OpenCV will otherwise pick a backend (GStreamer)
        # that ignores the fourcc and buffer-size hints below.
        capture = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open {self._device}")

        # MJPG rather than the YUYV default: an uncompressed 640x480 stream at
        # 30 fps does not fit a USB2 link, and the camera silently drops to
        # 10 fps instead of saying so. Order matters -- fourcc before the
        # geometry, or the driver picks a mode for the old format.
        if self._fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        capture.set(cv2.CAP_PROP_FPS, self._framerate)

        # One buffer, so read() returns the newest frame instead of the oldest
        # queued one. Aiming at where the target was four frames ago is the
        # whole failure mode this package exists to avoid.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual = (
            int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            capture.get(cv2.CAP_PROP_FPS),
        )
        self.get_logger().info(
            f"{self._device} open at {actual[0]}x{actual[1]} @ {actual[2]:.0f} fps "
            f"({self._fourcc}); requested {self._width}x{self._height} @ "
            f"{self._framerate:.0f}")
        if (actual[0], actual[1]) != (self._width, self._height):
            self.get_logger().warn(
                "camera did not accept the requested geometry; camera_info and "
                "any pixel->angle math downstream must use the actual size")

        if self._fx <= 0.0 or self._fy <= 0.0:
            self.get_logger().warn(
                "fx/fy are unset: camera_info will carry no focal length, and "
                "gimbal_bridge cannot convert pixels to angles. Run a "
                "calibration and put the result in the params file.")

        return capture

    def _tick(self):
        ok, frame = self._capture.read()
        if not ok:
            self._read_failures += 1
            self.get_logger().error(
                f"frame read failed ({self._read_failures} in a row); "
                "is the camera still plugged in?",
                throttle_duration_sec=5.0)
            return
        self._read_failures = 0

        stamp = self.get_clock().now().to_msg()
        height, width = frame.shape[:2]

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = height
        msg.width = width
        # bgr8 is what OpenCV already holds and what a cv2-based detector wants
        # back, so nothing in this path pays for a colour conversion.
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = frame.tobytes()

        self._info_pub.publish(self._camera_info(stamp, width, height))
        self._image_pub.publish(msg)

    def _camera_info(self, stamp, width, height):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = self._distortion

        cx = self._cx if self._cx > 0.0 else width / 2.0
        cy = self._cy if self._cy > 0.0 else height / 2.0
        info.k = [self._fx, 0.0, cx, 0.0, self._fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self._fx, 0.0, cx, 0.0, 0.0, self._fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        if self._capture is not None:
            self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
