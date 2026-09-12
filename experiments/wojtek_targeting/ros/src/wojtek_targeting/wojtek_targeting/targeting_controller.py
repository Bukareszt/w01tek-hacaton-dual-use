"""The gimbal's state machine, and the only process that commands the servos.

Everything that can move the gimbal goes through this node: the detector only
publishes detections and never touches a motor, and the manual-angle service is
refused while tracking is on so a calibration session cannot fight the
detector for the bus.

There is no laser code here, and there should never be. The laser is armed by
hardware outside this graph; ROS is not allowed to be the only thing standing
between a detection and an energised beam.

Mode transitions:

    DISABLED --enable--> SEARCHING --fresh target--> TRACKING --dwell--> LOCKED
        ^                    ^          |                 |                |
        |                    +--timeout-+                 +----timeout-----+
        +--------disable, or any bus failure (-> FAULT)---------------------+

FAULT latches. It means a write to a servo failed, so the controller no longer
knows what the gimbal is doing and stops commanding it -- while leaving torque
on, because a head-heavy assembly dropping limp is worse than one holding a
stale pose. Toggling `enable_tracking` off and on clears it and retries the
bus; if the cable is genuinely out, it faults straight back.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_srvs.srv import SetBool, Trigger
from wojtek_targeting_msgs.msg import LaserTarget, TargetingStatus
from wojtek_targeting_msgs.srv import SetGimbalAngles

from .dynamixel_driver import DynamixelDriver, DynamixelError, MotorConfig
from .kinematics import Intrinsics, clamp, rate_limit, target_to_optical_angles


class TargetingController(Node):
    def __init__(self, driver_factory=DynamixelDriver):
        super().__init__("targeting_controller")

        self._update_rate = self.declare_parameter("update_rate_hz", 40.0).value
        state_rate = self.declare_parameter("state_rate_hz", 20.0).value
        self._confidence_threshold = self.declare_parameter("confidence_threshold", 0.65).value
        self._target_timeout = self.declare_parameter("target_timeout_s", 0.20).value
        self._lock_tolerance = self.declare_parameter("lock_tolerance_deg", 1.0).value
        self._lock_dwell = self.declare_parameter("lock_dwell_s", 0.30).value
        self._tracking_gain = self.declare_parameter("tracking_gain", 0.6).value
        self._home = (
            self.declare_parameter("pan_home_deg", 0.0).value,
            self.declare_parameter("tilt_home_deg", 0.0).value,
        )
        self._offsets = (
            self.declare_parameter("pan_offset_deg", 0.0).value,
            self.declare_parameter("tilt_offset_deg", 0.0).value,
        )
        self._max_rates = (
            self.declare_parameter("max_pan_velocity_deg_s", 45.0).value,
            self.declare_parameter("max_tilt_velocity_deg_s", 35.0).value,
        )
        self._joint_names = (
            self.declare_parameter("pan_joint_name", "targeting_pan_joint").value,
            self.declare_parameter("tilt_joint_name", "targeting_tilt_joint").value,
        )
        self._home_on_disable = self.declare_parameter("home_on_disable", False).value

        self._pan = self._motor_config("pan", 1, self._max_rates[0])
        self._tilt = self._motor_config("tilt", 2, self._max_rates[1])

        self._driver = driver_factory(
            port=self.declare_parameter("device", "/dev/wojtek_gimbal").value,
            baud_rate=self.declare_parameter("baud_rate", 57600).value,
            motors=[self._pan, self._tilt],
            logger=self.get_logger(),
        )

        self._mode = TargetingStatus.MODE_DISABLED
        self._fault_message = ""
        self._tracking_enabled = False
        self._target = None
        self._target_age = float("inf")
        self._intrinsics = None
        self._locked_since = None
        self._last_error = (0.0, 0.0)
        # What the controller last asked for, as opposed to where the servos
        # are. Tracking integrates onto this rather than onto the measured
        # angle: feeding a measurement that lags the command back into the
        # command is how a servo loop starts hunting.
        self._command = self._home

        # Namespace-relative, not node-relative: these belong to the targeting
        # subsystem rather than to this particular node, so launching under
        # namespace `targeting` puts them at /targeting/target and friends
        # whatever the node ends up being called.
        self._status_pub = self.create_publisher(TargetingStatus, "status", 10)
        self._joint_pub = self.create_publisher(JointState, "gimbal_state", 10)

        self.create_subscription(
            LaserTarget, "target", self._on_target, qos_profile_sensor_data)
        # Remapped by the launch onto the targeting camera's camera_info. The
        # intrinsics are read from the camera rather than duplicated into this
        # package's parameters, so a recalibration lands in one place.
        self.create_subscription(
            CameraInfo, "camera_info", self._on_camera_info, qos_profile_sensor_data)

        self.create_service(SetBool, "enable_tracking", self._on_enable_tracking)
        self.create_service(Trigger, "home", self._on_home)
        self.create_service(SetGimbalAngles, "set_manual_angles", self._on_set_manual_angles)

        self._driver.connect()
        self._driver.enable_torque()
        # Start from where the gimbal actually is, not from home: the servos
        # were just seeded with their present position and a home command here
        # would move the robot before anyone asked it to.
        states = self._driver.read_states()
        self._command = (
            states[self._pan.motor_id].position_deg,
            states[self._tilt.motor_id].position_deg,
        )

        self._state_decimation = max(1, round(self._update_rate / max(state_rate, 1e-6)))
        self._tick_count = 0
        self._dt = 1.0 / self._update_rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"targeting_controller up, holding "
            f"pan {self._command[0]:.1f} / tilt {self._command[1]:.1f} deg. "
            "Tracking is DISABLED until /targeting/enable_tracking says otherwise.")

    def _motor_config(self, axis, default_id, max_rate):
        return MotorConfig(
            name=axis,
            motor_id=self.declare_parameter(f"{axis}_motor_id", default_id).value,
            center_raw=self.declare_parameter(f"{axis}_center_raw", 2048).value,
            direction=self.declare_parameter(f"{axis}_direction", 1 if axis == "pan" else -1).value,
            min_deg=self.declare_parameter(f"{axis}_min_deg", -80.0 if axis == "pan" else -35.0).value,
            max_deg=self.declare_parameter(f"{axis}_max_deg", 80.0 if axis == "pan" else 45.0).value,
            profile_velocity_deg_s=max_rate,
            profile_acceleration_deg_s2=self.declare_parameter(
                f"{axis}_profile_acceleration_deg_s2", 180.0).value,
            p_gain=self.declare_parameter(f"{axis}_p_gain", 800).value,
            i_gain=self.declare_parameter(f"{axis}_i_gain", 0).value,
            d_gain=self.declare_parameter(f"{axis}_d_gain", 0).value,
        )

    # --- inputs ------------------------------------------------------------

    def _on_target(self, msg):
        self._target = msg

    def _on_camera_info(self, msg):
        intrinsics = Intrinsics(
            width=msg.width, height=msg.height,
            fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])
        if not intrinsics.valid:
            # The camera node already warns about this at startup; saying it
            # again every frame would bury everything else.
            self._intrinsics = None
            return
        if self._intrinsics != intrinsics:
            self.get_logger().info(
                f"camera intrinsics: {intrinsics.width}x{intrinsics.height}, "
                f"fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f}")
        self._intrinsics = intrinsics

    def _age_of(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            # An unstamped detection cannot be aged, so it is treated as
            # infinitely old rather than assumed current.
            return float("inf")
        now = self.get_clock().now().nanoseconds
        return (now - (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9

    # --- services ----------------------------------------------------------

    def _on_enable_tracking(self, request, response):
        self._tracking_enabled = request.data
        # Either direction leaves FAULT, so either direction clears it. Letting
        # the message outlive the mode means the next fault report is whatever
        # went wrong the time before.
        if self._mode == TargetingStatus.MODE_FAULT:
            self.get_logger().info("clearing fault; the next tick retries the bus")
            self._fault_message = ""

        if request.data:
            self._mode = TargetingStatus.MODE_SEARCHING
            response.message = "tracking enabled"
            if self._intrinsics is None:
                response.message = (
                    "tracking enabled, but the camera has published no usable intrinsics yet "
                    "-- the gimbal will hold until it does. Calibrate the targeting camera.")
        else:
            self._mode = TargetingStatus.MODE_DISABLED
            self._locked_since = None
            if self._home_on_disable:
                self._command = self._home
                response.message = "tracking disabled, returning to home"
            else:
                response.message = "tracking disabled, holding current pose"
        response.success = True
        return response

    def _on_home(self, request, response):
        del request
        if self._tracking_enabled:
            response.success = False
            response.message = "refused: disable tracking first, or it will immediately re-aim"
            return response
        self._command = self._home
        response.success = True
        response.message = f"moving to home ({self._home[0]:.1f}, {self._home[1]:.1f}) deg"
        return response

    def _on_set_manual_angles(self, request, response):
        if self._tracking_enabled:
            response.success = False
            response.message = "refused: disable tracking first, or it will immediately re-aim"
            return response
        if self._mode == TargetingStatus.MODE_FAULT:
            response.success = False
            response.message = f"refused: controller is in FAULT ({self._fault_message})"
            return response

        pan = clamp(request.pan_deg, self._pan.min_deg, self._pan.max_deg)
        tilt = clamp(request.tilt_deg, self._tilt.min_deg, self._tilt.max_deg)
        self._command = (pan, tilt)
        response.success = True
        if (pan, tilt) != (request.pan_deg, request.tilt_deg):
            response.message = (
                f"clamped to limits: pan {pan:.1f}, tilt {tilt:.1f} deg")
        else:
            response.message = f"moving to pan {pan:.1f}, tilt {tilt:.1f} deg"
        return response

    # --- control loop ------------------------------------------------------

    def _tick(self):
        self._tick_count += 1
        if self._tick_count % self._state_decimation == 0:
            try:
                states = self._driver.read_states()
            except DynamixelError as exc:
                self._enter_fault(str(exc))
                states = None
            if states is not None:
                self._publish_joint_state(states)

        if self._mode != TargetingStatus.MODE_FAULT:
            self._update_command()
            try:
                self._driver.write_goals({
                    self._pan.motor_id: self._command[0] + self._offsets[0],
                    self._tilt.motor_id: self._command[1] + self._offsets[1],
                })
            except DynamixelError as exc:
                self._enter_fault(str(exc))

        self._publish_status()

    def _update_command(self):
        if not self._tracking_enabled:
            return

        self._target_age = self._age_of(self._target) if self._target is not None else float("inf")
        usable = (
            self._target is not None
            and self._target.detected
            and self._target.confidence >= self._confidence_threshold
            and self._target_age <= self._target_timeout
            and self._intrinsics is not None
        )
        if not usable:
            # Hold the last commanded pose. Not homing here is deliberate: a
            # detector that blinks for three frames should not throw the aim
            # away and start again.
            self._mode = TargetingStatus.MODE_SEARCHING
            self._locked_since = None
            if self._intrinsics is None:
                self._fault_message = "waiting for usable camera intrinsics"
            return

        self._fault_message = ""
        yaw_deg, pitch_deg = target_to_optical_angles(
            self._target.target_x, self._target.target_y, self._intrinsics)

        # The optical angles say where the target is relative to the axis; the
        # servo directions say which way each axis has to turn to get there.
        pan_error = yaw_deg * self._pan.direction
        tilt_error = pitch_deg * self._tilt.direction
        self._last_error = (pan_error, tilt_error)

        # Take a fraction of the correction per tick. The detector's answer is
        # already a frame or two old, so closing the whole error every tick
        # commands a gimbal that is chasing where the target used to be, and
        # overshoots past it.
        desired = (
            self._command[0] + pan_error * self._tracking_gain,
            self._command[1] + tilt_error * self._tracking_gain,
        )
        self._command = (
            clamp(rate_limit(desired[0], self._command[0], self._max_rates[0], self._dt),
                  self._pan.min_deg, self._pan.max_deg),
            clamp(rate_limit(desired[1], self._command[1], self._max_rates[1], self._dt),
                  self._tilt.min_deg, self._tilt.max_deg),
        )

        centred = (
            abs(pan_error) <= self._lock_tolerance
            and abs(tilt_error) <= self._lock_tolerance
        )
        now = self.get_clock().now().nanoseconds / 1e9
        if not centred:
            self._locked_since = None
            self._mode = TargetingStatus.MODE_TRACKING
        else:
            if self._locked_since is None:
                self._locked_since = now
            self._mode = (
                TargetingStatus.MODE_LOCKED
                if now - self._locked_since >= self._lock_dwell
                else TargetingStatus.MODE_TRACKING
            )

    def _enter_fault(self, message):
        if self._mode != TargetingStatus.MODE_FAULT:
            self.get_logger().error(
                f"FAULT: {message}. Holding position and no longer commanding the servos. "
                "Toggle /targeting/enable_tracking off then on to retry.")
        self._mode = TargetingStatus.MODE_FAULT
        self._fault_message = message
        self._locked_since = None

    # --- outputs -----------------------------------------------------------

    def _publish_joint_state(self, states):
        pan = states[self._pan.motor_id]
        tilt = states[self._tilt.motor_id]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [math.radians(pan.position_deg), math.radians(tilt.position_deg)]
        msg.velocity = [pan.velocity_rad_s, tilt.velocity_rad_s]
        # Amperes, not newton-metres. There is no torque constant for these
        # servos in the control table, and current is what the hardware
        # actually measures -- calling it effort is the ROS convention for
        # Dynamixels and beats publishing raw encoder counts.
        msg.effort = [pan.current_a, tilt.current_a]
        self._joint_pub.publish(msg)

        for name, state in (("pan", pan), ("tilt", tilt)):
            if state.temperature_c >= 70.0:
                self.get_logger().warn(
                    f"{name} servo at {state.temperature_c:.0f} C -- it will shut down on "
                    "its own at 80. Let it cool before the next run.",
                    throttle_duration_sec=10.0)

    def _publish_status(self):
        states = self._driver.last_states
        msg = TargetingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self._mode
        msg.target_fresh = self._target_age <= self._target_timeout
        msg.locked = self._mode == TargetingStatus.MODE_LOCKED
        msg.requested_pan_deg = float(self._command[0])
        msg.requested_tilt_deg = float(self._command[1])
        msg.actual_pan_deg = float(states[self._pan.motor_id].position_deg)
        msg.actual_tilt_deg = float(states[self._tilt.motor_id].position_deg)
        msg.detection_age_s = float(min(self._target_age, 1e6))
        msg.fault_message = self._fault_message
        self._status_pub.publish(msg)

    def destroy_node(self):
        self._driver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetingController()
        rclpy.spin(node)
    except (DynamixelError, KeyboardInterrupt) as exc:
        if isinstance(exc, DynamixelError):
            rclpy.logging.get_logger("targeting_controller").fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
