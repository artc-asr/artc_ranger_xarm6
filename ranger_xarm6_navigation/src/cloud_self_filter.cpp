// A point cloud without the robot itself: points inside a box around the
// robot (x/y in target_frame, any height) are dropped, the rest published
// in target_frame, one per voxel (their centroid).
//
// For collision_monitor, which has no self-filter: the Mid-360 sees the
// arm (0.27-0.67 m from the lidar at home/stow), inside every zone around
// the robot, so the raw cloud would stop the base for good. The sensor
// must be fixed to target_frame (its transform is looked up once).
//
// Parameters: target_frame; min_x, max_x, min_y, max_y [m]: the box;
// voxel_size [m] (0 = keep every point). Topics: in, out (PointCloud2,
// x/y/z float32 in and out).
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class CloudSelfFilter : public rclcpp::Node
{
public:
  CloudSelfFilter()
  : Node("cloud_self_filter")
  {
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));
    min_x_ = declare_parameter("min_x", -0.57);
    max_x_ = declare_parameter("max_x", 0.42);
    min_y_ = declare_parameter("min_y", -0.31);
    max_y_ = declare_parameter("max_y", 0.31);
    voxel_ = declare_parameter("voxel_size", 0.05);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<PointCloud2>("out", rclcpp::SensorDataQoS());
    sub_ = create_subscription<PointCloud2>(
      "in", rclcpp::SensorDataQoS(), [this](PointCloud2::ConstSharedPtr msg) {filter(*msg);});
  }

private:
  struct Sum
  {
    double x = 0, y = 0, z = 0;
    int n = 0;
  };

  void filter(const PointCloud2 & in)
  {
    if (!have_tf_) {
      try {
        const auto t = tf_buffer_->lookupTransform(target_frame_, in.header.frame_id, tf2::TimePointZero);
        const auto & q = t.transform.rotation;
        const auto & p = t.transform.translation;
        tf_ = Eigen::Translation3d(p.x, p.y, p.z) * Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        have_tf_ = true;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        return;
      }
    }
    int off[3] = {-1, -1, -1};
    for (const auto & f : in.fields) {
      int i = f.name == "x" ? 0 : f.name == "y" ? 1 : f.name == "z" ? 2 : -1;
      if (i >= 0 && f.datatype == PointField::FLOAT32) {
        off[i] = f.offset;
      }
    }
    if (off[0] < 0 || off[1] < 0 || off[2] < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cloud has no float32 x/y/z");
      return;
    }

    voxels_.clear();
    std::vector<float> kept;
    for (size_t row = 0; row < in.height; ++row) {
      const uint8_t * p = in.data.data() + row * in.row_step;
      for (size_t col = 0; col < in.width; ++col, p += in.point_step) {
        float v[3];
        for (int i = 0; i < 3; ++i) {
          std::memcpy(&v[i], p + off[i], sizeof(float));
        }
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {
          continue;
        }
        const Eigen::Vector3d b = tf_ * Eigen::Vector3d(v[0], v[1], v[2]);
        if (b.x() >= min_x_ && b.x() <= max_x_ && b.y() >= min_y_ && b.y() <= max_y_) {
          continue;
        }
        if (voxel_ <= 0) {
          kept.insert(kept.end(), {float(b.x()), float(b.y()), float(b.z())});
          continue;
        }
        // 21 bits per axis: +-52 km at 5 cm.
        int64_t key = 0;
        for (int i = 0; i < 3; ++i) {
          key = (key << 21) | ((static_cast<int64_t>(std::floor(b[i] / voxel_)) + (1 << 20)) & 0x1FFFFF);
        }
        Sum & s = voxels_[key];
        s.x += b.x();
        s.y += b.y();
        s.z += b.z();
        ++s.n;
      }
    }
    for (const auto & kv : voxels_) {
      const Sum & s = kv.second;
      kept.insert(kept.end(), {float(s.x / s.n), float(s.y / s.n), float(s.z / s.n)});
    }

    PointCloud2 out;
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.width = kept.size() / 3;
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
    std::memcpy(out.data.data(), kept.data(), out.data.size());
    pub_->publish(out);
  }

  std::string target_frame_;
  double min_x_, max_x_, min_y_, max_y_, voxel_;
  bool have_tf_ = false;
  Eigen::Affine3d tf_;
  std::unordered_map<int64_t, Sum> voxels_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
