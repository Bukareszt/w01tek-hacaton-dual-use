"""The follow node: the Deck's lock becomes a pointed tower and a walking body.

Two loops live here and nothing else does.  The aim relay runs at 40 Hz and
keeps the tower team's controller supplied with a target that is never stale.
The body loop runs at 10 Hz and turns the bearing, the range and the obstacle
bands into a velocity.  All of the arithmetic is in `core/`, which imports no
ROS at all and is tested without one.

What this node is allowed to touch is a short list.  It publishes the gimbal's
target on `/targeting/target` and calls `/targeting/enable_tracking`.  It
publishes its own velocity on `/wojtek/follow/cmd_vel`, which the Deck
gateway's DriveGate picks up as one source among several.  It never publishes
`/cmd_vel`, never talks to ros2_control, and never arms anything.  The single
publisher of `/cmd_vel` in the Deck setup is the gateway, and the dead-man that
stops the robot lives there.

Nothing is published on `/wojtek/follow/cmd_vel` while the state is IDLE.  A
stream of zeros would be indistinguishable from a follow node that is working,
and the gateway's dead-man already stops the robot when a source goes quiet.
"""

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from wojtek_targeting_msgs.msg import LaserTarget, TargetingStatus
except ImportError:  # the tower team's package is not built here
    LaserTarget = TargetingStatus = None

from .core import controller
from .core.aim import AimParams, AimRelay
from .core.gate import GateParams, obstacle_bands
from .core.geometry import Intrinsics
from .core.rangefinder import RangeParams, estimate_range

DEG = 57.29577951308232


class FollowNode(Node):
    def __init__(self):
        super().__init__("wojtek_follow")

        self._aim_params = AimParams(
            pan_sign=float(self._param("pan_sign", -1.0)),
            tilt_sign=float(self._param("tilt_sign", 1.0)),
            coast_s=float(self._param("coast_s", 0.7)),
            track_latency_s=float(self._param("track_latency_s", 0.15)),
            gimbal_max_age_s=float(self._param("gimbal_max_age_s", 0.5)),
        )
        # A depth frame older than this is not the scene in front of the
        # robot.  Three 15 Hz periods.
        self._depth_max_age_s = float(self._param("depth_max_age_s", 0.5))
        # How long after asking the gimbal to track its reported mode is
        # believed over the request.  The service answers asynchronously and
        # its status can say DISABLED for a tick or two after a request.
        self._tracking_reconcile_s = float(self._param("tracking_reconcile_s", 1.0))
        self._range_params = RangeParams(
            depth_camera_pitch_deg=float(self._param("depth_camera_pitch_deg", 0.0)),
            window_px=int(self._param("range_window_px", 24)),
            percentile=float(self._param("range_percentile", 20.0)),
            min_range_m=float(self._param("range_min_m", 0.3)),
            max_range_m=float(self._param("range_max_m", 4.0)),
            max_azimuth_deg=float(self._param("range_max_azimuth_deg", 35.0)),
        )
        self._gate_params = GateParams(
            decimate=int(self._param("gate_decimate", 4)),
            depth_camera_pitch_deg=self._range_params.depth_camera_pitch_deg,
            depth_camera_height_m=float(self._param("depth_camera_height_m", 0.20)),
            min_height_m=float(self._param("gate_min_height_m", 0.06)),
            max_height_m=float(self._param("gate_max_height_m", 0.35)),
            min_range_m=float(self._param("gate_min_range_m", 0.3)),
            max_range_m=float(self._param("gate_max_range_m", 3.0)),
            side_azimuth_deg=float(self._param("gate_side_azimuth_deg", 15.0)),
            cone_azimuth_deg=float(self._param("gate_cone_azimuth_deg", 10.0)),
            cone_margin_m=float(self._param("gate_cone_margin_m", 0.3)),
        )
        self._follow_params = controller.FollowParams(
            yaw_gain=float(self._param("yaw_gain", 1.5)),
            yaw_deadband_deg=float(self._param("yaw_deadband_deg", 2.0)),
            yaw_max=float(self._param("yaw_max", 0.7)),
            align_threshold_deg=float(self._param("align_threshold_deg", 30.0)),
            hold_distance_m=float(self._param("hold_distance_m", 1.0)),
            hold_hysteresis_m=float(self._param("hold_hysteresis_m", 0.15)),
            forward_gain=float(self._param("forward_gain", 0.8)),
            vx_max=float(self._param("vx_max", 0.4)),
            vx_min_reverse=float(self._param("vx_min_reverse", -0.2)),
            min_speed=float(self._param("min_speed", 0.12)),
            stop_range_m=float(self._param("stop_range_m", 0.5)),
            slow_range_m=float(self._param("slow_range_m", 1.0)),
            sidestep=float(self._param("sidestep", 0.25)),
            hold_after_loss_s=float(self._param("hold_after_loss_s", 1.0)),
            drop_after_loss_s=float(self._param("drop_after_loss_s", 3.0)),
        )

        track_topic = self._param("track_topic", "/wojtek/track/target")
        gimbal_state_topic = self._param("gimbal_state_topic", "/targeting/gimbal_state")
        targeting_status_topic = self._param("targeting_status_topic", "/targeting/status")
        tower_camera_info_topic = self._param(
            "tower_camera_info_topic",
            "/targeting_camera/targeting_camera/camera_info")
        depth_topic = self._param(
            "depth_topic", "/camera/camera/depth/image_rect_raw")
        depth_camera_info_topic = self._param(
            "depth_camera_info_topic", "/camera/camera/depth/camera_info")
        imu_topic = self._param("imu_topic", "/imu_sensor_broadcaster/imu")
        target_topic = self._param("target_topic", "/targeting/target")
        cmd_vel_topic = self._param("cmd_vel_topic", "/wojtek/follow/cmd_vel")
        status_topic = self._param("status_topic", "/wojtek/follow/status")
        enable_service = self._param("enable_tracking_service", "/targeting/enable_tracking")
        self._pan_joint = self._param("pan_joint_name", "targeting_pan_joint")
        self._tilt_joint = self._param("tilt_joint_name", "targeting_tilt_joint")

        aim_rate = float(self._param("aim_rate_hz", 40.0))
        body_rate = float(self._param("body_rate_hz", 10.0))
        # A fixed gimbal: pan and tilt are zero for ever, nothing is sent to
        # the targeting side and nothing is expected from it.  This is the
        # simulation and any rig without the tower, where the "tower camera"
        # is a body-fixed camera and following is follow v1 exactly.
        self._gimbal_fixed = bool(self._param("gimbal_fixed", False))
        if not self._gimbal_fixed and LaserTarget is None:
            raise RuntimeError(
                "wojtek_targeting_msgs is not built. Build the targeting "
                "experiment beside this one, or run with gimbal_fixed:=true.")

        self._relay = AimRelay(self._aim_params)
        self._state = controller.FollowState()
        self._depth = None
        self._depth_stamp = None
        self._depth_stale = False
        self._depth_intrinsics = None
        self._tower_camera_info_topic = tower_camera_info_topic
        self._gimbal_mode = None
        self._tracking_requested = None
        self._tracking_requested_at = None
        self._pending_calls = []
        self._last_command = (0.0, 0.0, 0.0)
        self._last_track_cam = None

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, track_topic, self._on_track, 10)
        if self._gimbal_fixed:
            self._target_pub = None
            self._enable_client = None
        else:
            self._target_pub = self.create_publisher(LaserTarget, target_topic, 10)
            self.create_subscription(
                JointState, gimbal_state_topic, self._on_gimbal, qos_profile_sensor_data)
            self.create_subscription(
                TargetingStatus, targeting_status_topic, self._on_targeting_status, 10)
        self.create_subscription(
            CameraInfo, tower_camera_info_topic, self._on_tower_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, depth_camera_info_topic, self._on_depth_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, imu_topic, self._on_imu, qos_profile_sensor_data)

        if not self._gimbal_fixed:
            self._enable_client = self.create_client(SetBool, enable_service)

        self._body_dt = 1.0 / body_rate
        self.create_timer(1.0 / aim_rate, self._aim_tick)
        self.create_timer(self._body_dt, self._body_tick)

        self.get_logger().info(
            f"wojtek_follow up. Waiting for a track on {track_topic}. "
            f"pan_sign={self._aim_params.pan_sign:+.0f} "
            f"tilt_sign={self._aim_params.tilt_sign:+.0f} (the negated "
            "pan_direction and tilt_direction of the targeting controller): "
            "verify these on the bench before the robot walks, by turning the "
            "body by hand and watching the tower counter-rotate.")

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # --- inputs ------------------------------------------------------------

    def _on_track(self, msg):
        """The Deck's lock, as the gateway republished it.

        Anything malformed is dropped with a warning rather than allowed to
        become a bearing.  This message comes off a websocket from a phone.
        """
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn("track message is not JSON", throttle_duration_sec=5.0)
            return
        if not isinstance(data, dict):
            self.get_logger().warn("track message is not an object", throttle_duration_sec=5.0)
            return
        if data.get("unlock"):
            self.get_logger().info("the Deck let go of the target")
            self._drop_lock()
            return

        cam = data.get("cam", "tower")
        self._last_track_cam = cam
        if cam != "tower":
            # A front-camera pixel carries no tower bearing.  Following from it
            # needs the front camera's intrinsics and its mount transform,
            # neither of which this node has.  See the README's known gaps.
            self.get_logger().warn(
                f"ignoring a track from the {cam} camera: this node follows the "
                "tower picture only", throttle_duration_sec=5.0)
            return

        try:
            track = {
                "cx": float(data["cx"]),
                "cy": float(data["cy"]),
                "w": float(data["w"]),
                "h": float(data["h"]),
                "fw": int(data["fw"]),
                "fh": int(data["fh"]),
                "label": data.get("label") or "",
                "age": float(data.get("age", 0.0)),
                "stamp": float(data.get("stamp", self._now())),
            }
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn("track message is missing fields", throttle_duration_sec=5.0)
            return
        if track["fw"] <= 0 or track["fh"] <= 0:
            self.get_logger().warn("track message has no frame size", throttle_duration_sec=5.0)
            return

        self._relay.on_track(track)
        self._request_tracking(True)

    def _on_gimbal(self, msg):
        names = list(msg.name)
        if self._pan_joint not in names or self._tilt_joint not in names:
            self.get_logger().warn(
                "gimbal_state carries no pan/tilt joint by the configured names",
                throttle_duration_sec=10.0)
            return
        position = list(msg.position)
        pan = position[names.index(self._pan_joint)]
        tilt = position[names.index(self._tilt_joint)]
        stamp = self._stamp_seconds(msg.header.stamp)
        # An unstamped joint state is treated as having just arrived, which is
        # true to within one 20 Hz period.
        self._relay.on_gimbal(stamp if stamp > 0.0 else self._now(), pan, tilt)

    def _on_targeting_status(self, msg):
        """The gimbal's own word on what it is doing.

        A targeting controller that faults or is restarted mid-lock comes
        back DISABLED, and nothing else would ever tell this node.  While a
        lock is held and tracking was asked for, a DISABLED or FAULT report
        that outlives the request by `tracking_reconcile_s` forgets the
        request, so the next track asks again.  Asking again also clears a
        latched FAULT on the controller's side, which retries the bus.
        """
        self._gimbal_mode = int(msg.mode)
        if self._tracking_requested is not True or not self._relay.has_track:
            return
        if self._gimbal_mode not in (
                TargetingStatus.MODE_DISABLED, TargetingStatus.MODE_FAULT):
            return
        asked = self._tracking_requested_at
        if asked is not None and self._now() - asked < self._tracking_reconcile_s:
            return
        word = "FAULT" if self._gimbal_mode == TargetingStatus.MODE_FAULT else "DISABLED"
        self.get_logger().warn(
            f"the targeting controller reports {word} while a lock is held "
            f"({msg.fault_message or 'no fault message'}); the next track asks "
            "it to track again", throttle_duration_sec=5.0)
        self._tracking_requested = None

    def _on_tower_info(self, msg):
        intrinsics = Intrinsics.from_camera_info(msg)
        if not intrinsics.valid:
            self.get_logger().warn(
                "the tower camera publishes no focal length, so there is no "
                "bearing to be had. Calibrate it.", throttle_duration_sec=30.0)
            self._relay.set_intrinsics(None)
            return
        self._relay.set_intrinsics(intrinsics)

    def _on_depth_info(self, msg):
        self._depth_intrinsics = Intrinsics.from_camera_info(msg)

    def _on_depth(self, msg):
        """16UC1 millimetres, decoded without cv_bridge.

        `step` is the row stride in bytes and it is not always twice the width,
        so the rows are cut to the width after the reshape.
        """
        if msg.encoding not in ("16UC1", "mono16"):
            self.get_logger().warn(
                f"depth image is {msg.encoding}, expected 16UC1",
                throttle_duration_sec=30.0)
            return
        stride = int(msg.step) // 2
        height = int(msg.height)
        width = int(msg.width)
        if stride < width or height <= 0 or width <= 0:
            return
        dtype = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
        try:
            # The message's own buffer, with no copy. rclpy hands over an
            # array of bytes, which is 200 KB a frame at 15 Hz and not worth
            # duplicating on a Raspberry Pi.
            flat = np.frombuffer(msg.data, dtype=dtype)
        except (TypeError, ValueError):
            self.get_logger().warn("undecodable depth image", throttle_duration_sec=10.0)
            return
        if flat.size < stride * height:
            self.get_logger().warn("short depth image", throttle_duration_sec=10.0)
            return
        self._depth = flat[:stride * height].reshape(height, stride)[:, :width]
        stamp = self._stamp_seconds(msg.header.stamp)
        # An unstamped frame is treated as having just arrived.
        self._depth_stamp = stamp if stamp > 0.0 else self._now()

    def _fresh_depth(self, now):
        """The last depth frame, or None once it is too old to walk on.

        A RealSense that drops off the bus leaves its last frame behind.  The
        range and the obstacle bands taken from it would be a scene that no
        longer exists, so past `depth_max_age_s` the frame is withheld: the
        range falls back to the box width, which never drives the walk, and
        the bands empty.
        """
        if self._depth is None or self._depth_stamp is None:
            return None
        age = now - self._depth_stamp
        if age > self._depth_max_age_s:
            if not self._depth_stale:
                self.get_logger().warn(
                    f"the depth image is {age:.1f} s old; no walking and no "
                    "obstacle bands until it is fresh again")
                self._depth_stale = True
            return None
        if self._depth_stale:
            self.get_logger().info("the depth image is fresh again")
            self._depth_stale = False
        return self._depth

    def _depth_age(self, now):
        if self._depth_stamp is None:
            return None
        return max(0.0, now - self._depth_stamp)

    def _on_imu(self, msg):
        self._relay.on_gyro(
            self._now(), msg.angular_velocity.y, msg.angular_velocity.z)

    # --- the gimbal's tracking switch --------------------------------------

    def _request_tracking(self, enable):
        """Ask the tower controller to start or stop tracking.

        Asynchronous on purpose.  The body loop and the aim relay share this
        node's executor, and blocking on a service future here would stop both
        of them until the gimbal answers.
        """
        if self._gimbal_fixed or self._tracking_requested == enable:
            return
        if not self._enable_client.service_is_ready():
            # Forget what was asked for rather than remember a request that
            # never went out.  Otherwise a failed "stop" would make the next
            # "start" look like a repeat and be skipped.
            self._tracking_requested = None
            self.get_logger().warn(
                "the targeting controller is not up, so the gimbal will not "
                "track. The follow node keeps aiming anyway.",
                throttle_duration_sec=10.0)
            return
        self._tracking_requested = enable
        self._tracking_requested_at = self._now()
        request = SetBool.Request()
        request.data = bool(enable)
        self._pending_calls.append(self._enable_client.call_async(request))
        self._prune_calls()
        self.get_logger().info(f"enable_tracking({enable})")

    def _prune_calls(self):
        # The futures are kept only so nothing collects them mid-flight.
        self._pending_calls = [
            call for call in self._pending_calls
            if not (hasattr(call, "done") and call.done())
        ]

    def _drop_lock(self):
        self._relay.clear()
        self._state = controller.FollowState()
        self._request_tracking(False)

    # --- loops -------------------------------------------------------------

    def _aim_tick(self):
        if self._gimbal_fixed:
            # The gimbal never moves, so its state is a fact this node can
            # supply itself, and there is nobody to send a target to.
            self._relay.on_gimbal(self._now(), 0.0, 0.0)
            return
        target = self._relay.target(self._now())
        if target is None:
            return
        msg = LaserTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.detected = bool(target["detected"])
        msg.class_name = str(target["class_name"])
        msg.target_x = float(target["target_x"])
        msg.target_y = float(target["target_y"])
        msg.distance_m = float(target["distance_m"])
        msg.confidence = float(target["confidence"])
        self._target_pub.publish(msg)

    def _waiting_for(self):
        """What a held lock is missing before it has a bearing, or None."""
        if self._relay.has_track and self._relay.intrinsics is None:
            return f"tower camera_info on {self._tower_camera_info_topic}"
        return None

    def _body_tick(self):
        now = self._now()
        bearing = self._relay.bearing(now)
        track_age = self._relay.track_age(now)
        depth = self._fresh_depth(now)
        range_m = None
        source = "none"
        bands = {"left": None, "centre": None, "right": None}

        waiting_for = self._waiting_for()
        if waiting_for is not None:
            self.get_logger().warn(
                f"a lock is held but there is no bearing: waiting for the "
                f"{waiting_for}", throttle_duration_sec=5.0)
        if bearing is not None and self._relay.gimbal_stale(now):
            self.get_logger().warn(
                f"no gimbal_state for {self._relay.gimbal_age(now):.1f} s; the "
                "bearing uses the last pan the gimbal reported",
                throttle_duration_sec=5.0)

        if bearing is not None:
            intrinsics = self._relay.intrinsics
            range_m, source = estimate_range(
                bearing,
                depth,
                self._depth_intrinsics,
                self._relay.box_width_px(),
                (self._relay.track or {}).get("label", ""),
                intrinsics.fx if intrinsics is not None else 0.0,
                self._range_params,
            )
            # Only a measured range goes into the gimbal's message.  A range
            # guessed from a box width is not a distance anyone should aim by.
            self._relay.set_range(range_m if source == "depth" else None)
            bands = obstacle_bands(
                depth, self._depth_intrinsics, self._gate_params,
                target_azimuth=bearing.azimuth,
                target_range=range_m if source == "depth" else None,
            )

        vx, vy, wz, state = controller.step(
            self._state, bearing, range_m, source, bands, track_age,
            self._body_dt, self._follow_params)
        dropped = state.unlock
        self._state = state
        self._last_command = (vx, vy, wz)

        # Nothing goes out while there is nothing to say.  WAITING is a lock
        # with no bearing, and a stream of zeros for it would look like a
        # follow node that is working.
        if state.mode not in (controller.IDLE, controller.WAITING):
            command = Twist()
            command.linear.x = float(vx)
            command.linear.y = float(vy)
            command.angular.z = float(wz)
            self._cmd_pub.publish(command)

        self._publish_status(bearing, range_m, source, bands, track_age,
                             waiting_for, self._depth_age(now))

        if dropped:
            self.get_logger().info(
                "the target has been out of sight too long, dropping the lock")
            self._drop_lock()

    def _publish_status(self, bearing, range_m, source, bands, track_age,
                        waiting_for=None, depth_age=None):
        status = {
            "state": self._state.mode,
            "waiting_for": waiting_for,
            "azimuth_deg": round(bearing.azimuth * DEG, 2) if bearing else None,
            "elevation_deg": round(bearing.elevation * DEG, 2) if bearing else None,
            "range_m": round(range_m, 3) if range_m is not None else None,
            "range_source": source,
            "nearest_obstacle_m": {
                name: (round(value, 3) if value is not None else None)
                for name, value in bands.items()
            },
            "track_age_s": round(track_age, 3) if track_age != float("inf") else None,
            "depth_age_s": round(depth_age, 3) if depth_age is not None else None,
            "cam": self._last_track_cam,
            "gimbal_mode": self._gimbal_mode,
            "vx": round(self._last_command[0], 3),
            "vy": round(self._last_command[1], 3),
            "wz": round(self._last_command[2], 3),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FollowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
