// A depth camera's depth image -> a small point cloud for Nav2's costmaps:
// pixels between min_range and max_range from the camera, back-projected
// with the camera's intrinsics, one point per voxel (their centroid), in
// the camera's optical frame (the costmap raytraces from a cloud's frame
// origin to clear).
//
// Why not the camera's own point cloud: a D435i cloud at 424x240 is ~100k
// points, 2-3 MB at 15 Hz. With Nav2 subscribed, Nav2's TF input stalled
// now and then in sim (the controller lost the robot's pose and aborted
// while the base kept going). The depth image is 0.2-0.4 MB, and 5 cm
// voxels out to 3 m are a few thousand points; a costmap cell is 5 cm
// anyway.
//
// Each cloud is held until TF can place it in wait_for_frame (the costmap's
// odom) wait_margin past its stamp. Nav2's costmaps pass observations
// through a tf2_ros::MessageFilter; a cloud newer than the latest TF is
// handed to the costmap from inside the TF buffer's callback when the
// transform arrives, and with the depth camera's clouds (always a few ms
// ahead of the base's TF) that froze the local costmap's TF within a few
// runs in sim: the controller lost the robot's pose and aborted. A cloud
// that is already transformable goes straight through.
//
// Topics: depth (sensor_msgs/Image, 32FC1 in m or 16UC1 in mm, 0/NaN/inf =
// no depth), camera_info, points (x/y/z float32). Parameters: voxel_size,
// min_range, max_range [m]; frame_id: overrides the image's (the sim's
// depth images are labelled with the camera's body frame, but are in the
// optical frame's axes like any image); wait_for_frame ('' = don't wait),
// wait_margin, wait_timeout [s].
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class DepthToCloud : public rclcpp::Node
{
public:
  DepthToCloud()
  : Node("depth_to_cloud")
  {
    voxel_ = declare_parameter("voxel_size", 0.05);
    min_range_ = declare_parameter("min_range", 0.2);
    max_range_ = declare_parameter("max_range", 3.0);
    frame_id_ = declare_parameter("frame_id", std::string());
    wait_frame_ = declare_parameter("wait_for_frame", std::string());
    wait_margin_ = rclcpp::Duration::from_seconds(declare_parameter("wait_margin", 0.05));
    wait_timeout_ = rclcpp::Duration::from_seconds(declare_parameter("wait_timeout", 0.5));
    pub_ = create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
    if (!wait_frame_.empty()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {release();});
    }
    info_sub_ = create_subscription<CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(), [this](CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    depth_sub_ = create_subscription<Image>(
      "depth", rclcpp::SensorDataQoS(), [this](Image::ConstSharedPtr msg) {convert(*msg);});
  }

private:
  struct Sum
  {
    double x = 0, y = 0, z = 0;
    int n = 0;
  };

  void convert(const Image & img)
  {
    namespace enc = sensor_msgs::image_encodings;
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no camera_info yet");
      return;
    }
    const bool mm = img.encoding == enc::TYPE_16UC1 || img.encoding == enc::MONO16;
    if (!mm && img.encoding != enc::TYPE_32FC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported depth encoding %s", img.encoding.c_str());
      return;
    }
    // camera_info may be for the full-size image (decimation): scale.
    const double sx = info_->width ? double(img.width) / info_->width : 1.0;
    const double sy = info_->height ? double(img.height) / info_->height : 1.0;
    const double fx = info_->k[0] * sx, cx = info_->k[2] * sx;
    const double fy = info_->k[4] * sy, cy = info_->k[5] * sy;
    if (fx <= 0 || fy <= 0) {
      return;
    }

    voxels_.clear();
    for (uint32_t v = 0; v < img.height; ++v) {
      const uint8_t * row = img.data.data() + v * img.step;
      for (uint32_t u = 0; u < img.width; ++u) {
        double z;
        if (mm) {
          uint16_t d;
          std::memcpy(&d, row + 2 * u, 2);
          z = d * 1e-3;
        } else {
          float d;
          std::memcpy(&d, row + 4 * u, 4);
          z = d;
        }
        if (!std::isfinite(z) || z <= 0) {
          continue;
        }
        const double x = (u - cx) * z / fx, y = (v - cy) * z / fy;
        const double r = std::sqrt(x * x + y * y + z * z);
        if (r < min_range_ || r > max_range_) {
          continue;
        }
        // 21 bits per axis: +-52 km at 5 cm.
        int64_t key = 0;
        for (double c : {x, y, z}) {
          key = (key << 21) | ((static_cast<int64_t>(std::floor(c / voxel_)) + (1 << 20)) & 0x1FFFFF);
        }
        Sum & s = voxels_[key];
        s.x += x;
        s.y += y;
        s.z += z;
        ++s.n;
      }
    }

    PointCloud2 out;
    out.header = img.header;
    if (!frame_id_.empty()) {
      out.header.frame_id = frame_id_;
    }
    out.height = 1;
    out.width = voxels_.size();
    out.is_dense = true;
    out.is_bigendian = false;
    out.point_step = 12;
    out.row_step = out.point_step * out.width;
    for (uint32_t i = 0; i < 3; ++i) {
      PointField f;
      f.name = std::string(1, char('x' + i));
      f.offset = 4 * i;
      f.datatype = PointField::FLOAT32;
      f.count = 1;
      out.fields.push_back(f);
    }
    out.data.resize(out.row_step);
    float * d = reinterpret_cast<float *>(out.data.data());
    for (const auto & kv : voxels_) {
      const Sum & s = kv.second;
      *d++ = s.x / s.n;
      *d++ = s.y / s.n;
      *d++ = s.z / s.n;
    }
    if (wait_frame_.empty()) {
      pub_->publish(out);
      return;
    }
    pending_.push_back(std::move(out));
    while (pending_.size() > 3) {
      pending_.pop_front();
    }
    release();
  }

  // Publish the pending clouds TF can already place (see the top).
  void release()
  {
    while (!pending_.empty()) {
      const PointCloud2 & cloud = pending_.front();
      const rclcpp::Time stamp(cloud.header.stamp);
      if (tf_buffer_->canTransform(wait_frame_, cloud.header.frame_id, stamp + wait_margin_)) {
        pub_->publish(cloud);
      } else if (now() - stamp < wait_timeout_) {
        return;
      }
      pending_.pop_front();
    }
  }

  double voxel_, min_range_, max_range_;
  std::string frame_id_, wait_frame_;
  rclcpp::Duration wait_margin_{0, 0}, wait_timeout_{0, 0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<PointCloud2> pending_;
  CameraInfo::ConstSharedPtr info_;
  std::unordered_map<int64_t, Sum> voxels_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToCloud>());
  rclcpp::shutdown();
  return 0;
}
