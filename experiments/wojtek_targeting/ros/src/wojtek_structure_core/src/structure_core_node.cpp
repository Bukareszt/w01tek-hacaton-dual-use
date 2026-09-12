// ROS 2 driver for the Occipital Structure Core (ST02D-C), built against the
// closed-source Structure SDK (Cross-Platform). See the package README for how
// the SDK is obtained and pointed at.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <ST/CaptureSession.h>
#include <ST/CaptureSessionSettings.h>
#include <ST/CaptureSessionTypes.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace wojtek_structure_core
{

class StructureCoreNode : public rclcpp::Node, public ST::CaptureSessionDelegate
{
public:
  StructureCoreNode()
  : rclcpp::Node("structure_core")
  {
    serial_ = declare_parameter<std::string>("sensor_serial", "");
    enable_color_ = declare_parameter<bool>("enable_color", true);
    enable_depth_ = declare_parameter<bool>("enable_depth", true);
    color_framerate_ = declare_parameter<double>("color_framerate", 30.0);
    depth_framerate_ = declare_parameter<double>("depth_framerate", 30.0);
    depth_resolution_ = declare_parameter<std::string>("depth_resolution", "VGA");
    const auto prefix = declare_parameter<std::string>("frame_id_prefix", "targeting_camera");

    color_frame_id_ = prefix + "_color_optical_frame";
    depth_frame_id_ = prefix + "_depth_optical_frame";

    const auto qos = rclcpp::SensorDataQoS();
    if (enable_color_) {
      color_pub_ = create_publisher<sensor_msgs::msg::Image>("~/color/image_raw", qos);
      color_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("~/color/camera_info", qos);
    }
    if (enable_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_rect_raw", qos);
      depth_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", qos);
    }
  }

  ~StructureCoreNode() override
  {
    session_.stopStreaming();
  }

  bool start()
  {
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = enable_depth_;
    settings.structureCore.visibleEnabled = enable_color_;
    // The aiming path never uses these, and each one costs USB bandwidth on a
    // link shared with the terrain camera. IMU in particular is cut by design:
    // the fallback is stop-then-lock-then-track, not aim-while-walking.
    settings.structureCore.infraredEnabled = false;
    settings.structureCore.accelerometerEnabled = false;
    settings.structureCore.gyroscopeEnabled = false;

    settings.structureCore.depthResolution = depthResolutionFromParam();
    settings.structureCore.visibleResolution = ST::StructureCoreVisibleResolution::_640x480;
    settings.structureCore.depthFramerate = static_cast<float>(depth_framerate_);
    settings.structureCore.visibleFramerate = static_cast<float>(color_framerate_);
    settings.structureCore.latencyReducerEnabled = true;
    settings.structureCore.dynamicCalibrationMode = ST::StructureCoreDynamicCalibrationMode::Off;

    // Borrowed pointer: the SDK does not copy the string, so serial_ has to
    // outlive the session. Null selects the first sensor on the bus.
    settings.structureCore.sensorSerial = serial_.empty() ? nullptr : serial_.c_str();

    session_.setDelegate(this);
    if (!session_.startMonitoring(settings)) {
      RCLCPP_ERROR(get_logger(), "startMonitoring failed; is the sensor plugged into USB3?");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "monitoring for Structure Core (serial: %s)",
      serial_.empty() ? "first available" : serial_.c_str());
    return true;
  }

  void captureSessionEventDidOccur(
    ST::CaptureSession * session, ST::CaptureSessionEventId event) override
  {
    switch (event) {
      case ST::CaptureSessionEventId::Booting:
        RCLCPP_INFO(get_logger(), "sensor booting");
        break;
      case ST::CaptureSessionEventId::Connected:
        RCLCPP_INFO(get_logger(), "sensor connected");
        break;
      case ST::CaptureSessionEventId::Ready:
        // Streaming can only be started from inside this callback -- the SDK
        // has no other point at which the sensor is known to be ready.
        RCLCPP_INFO(get_logger(), "sensor ready, starting stream");
        if (!session->startStreaming()) {
          RCLCPP_ERROR(get_logger(), "startStreaming failed");
        }
        break;
      case ST::CaptureSessionEventId::Disconnected:
        RCLCPP_WARN(get_logger(), "sensor disconnected");
        break;
      case ST::CaptureSessionEventId::Error:
        RCLCPP_ERROR(get_logger(), "sensor reported an error");
        break;
      case ST::CaptureSessionEventId::UsbError:
        RCLCPP_ERROR(get_logger(), "USB error; check the cable is USB3 and the port is not a hub");
        break;
      case ST::CaptureSessionEventId::LowPowerMode:
        RCLCPP_ERROR(get_logger(), "sensor in low power mode; the port cannot supply enough current");
        break;
      case ST::CaptureSessionEventId::USBDriverNotInstalled:
        RCLCPP_ERROR(get_logger(), "USB permissions missing; install the SDK's udev rule");
        break;
      default:
        break;
    }
  }

  void captureSessionDidOutputSample(
    ST::CaptureSession *, const ST::CaptureSessionSample & sample) override
  {
    switch (sample.type) {
      case ST::CaptureSessionSample::Type::SynchronizedFrames:
        publishColor(sample.visibleFrame);
        publishDepth(sample.depthFrame);
        break;
      case ST::CaptureSessionSample::Type::VisibleFrame:
        publishColor(sample.visibleFrame);
        break;
      case ST::CaptureSessionSample::Type::DepthFrame:
        publishDepth(sample.depthFrame);
        break;
      default:
        break;
    }
  }

private:
  ST::StructureCoreDepthResolution depthResolutionFromParam()
  {
    if (depth_resolution_ == "QVGA") {
      return ST::StructureCoreDepthResolution::_320x240;
    }
    if (depth_resolution_ == "SXGA") {
      return ST::StructureCoreDepthResolution::_1280x960;
    }
    if (depth_resolution_ != "VGA") {
      RCLCPP_WARN(
        get_logger(), "unknown depth_resolution '%s', using VGA", depth_resolution_.c_str());
    }
    return ST::StructureCoreDepthResolution::_640x480;
  }

  static sensor_msgs::msg::CameraInfo makeCameraInfo(
    const ST::Intrinsics & in, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = static_cast<uint32_t>(in.width);
    info.height = static_cast<uint32_t>(in.height);
    info.distortion_model = "plumb_bob";
    info.d = {in.k1, in.k2, in.p1, in.p2, in.k3};
    info.k = {in.fx, 0.0, in.cx, 0.0, in.fy, in.cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {in.fx, 0.0, in.cx, 0.0, 0.0, in.fy, in.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void publishColor(const ST::ColorFrame & frame)
  {
    if (!color_pub_ || !frame.isValid() || frame.rgbData() == nullptr) {
      return;
    }

    std_msgs::msg::Header header;
    // The SDK's own frame timestamp is on an unrelated epoch, so the arrival
    // time on the node clock is what downstream can actually compare against.
    header.stamp = now();
    header.frame_id = color_frame_id_;

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = static_cast<uint32_t>(frame.height());
    msg->width = static_cast<uint32_t>(frame.width());
    msg->encoding = "rgb8";
    msg->is_bigendian = 0;
    msg->step = msg->width * 3;
    msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
    std::memcpy(msg->data.data(), frame.rgbData(), msg->data.size());

    color_info_pub_->publish(makeCameraInfo(frame.intrinsics(), header));
    color_pub_->publish(std::move(msg));
  }

  void publishDepth(const ST::DepthFrame & frame)
  {
    if (!depth_pub_ || !frame.isValid() || frame.depthInMillimeters() == nullptr) {
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = depth_frame_id_;

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = static_cast<uint32_t>(frame.height());
    msg->width = static_cast<uint32_t>(frame.width());
    msg->encoding = "16UC1";
    msg->is_bigendian = 0;
    msg->step = msg->width * sizeof(uint16_t);
    msg->data.resize(static_cast<size_t>(msg->step) * msg->height);

    // The SDK hands out float millimetres with NaN for "no return"; ROS depth
    // images are uint16 millimetres with 0 for the same thing.
    const float * src = frame.depthInMillimeters();
    auto * dst = reinterpret_cast<uint16_t *>(msg->data.data());
    const size_t count = static_cast<size_t>(frame.width()) * frame.height();
    for (size_t i = 0; i < count; ++i) {
      const float mm = src[i];
      dst[i] = (std::isfinite(mm) && mm > 0.0f && mm < 65535.0f)
        ? static_cast<uint16_t>(mm)
        : 0;
    }

    depth_info_pub_->publish(makeCameraInfo(frame.intrinsics(), header));
    depth_pub_->publish(std::move(msg));
  }

  std::string serial_;
  bool enable_color_{true};
  bool enable_depth_{true};
  double color_framerate_{30.0};
  double depth_framerate_{30.0};
  std::string depth_resolution_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

  ST::CaptureSession session_;
};

}  // namespace wojtek_structure_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Constructed first, started second: the SDK calls back on its own thread as
  // soon as monitoring begins, and those callbacks touch the publishers.
  auto node = std::make_shared<wojtek_structure_core::StructureCoreNode>();
  if (!node->start()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
