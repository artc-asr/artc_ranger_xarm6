// What a depth camera has seen, kept after it's out of view: for
// collision_monitor, which only looks at the latest data.
//
// The front D435i (0.69 m up, level) sees a 0.15 m-tall obstacle only from
// ~1 m ahead of the lens, and the lidar not at all; collision_monitor's
// zones are all closer than that, so without memory it never stops for
// one. This node keeps the camera's obstacle points (voxels, in odom) and
// publishes them all with each depth frame.
//
// A remembered voxel is forgotten when:
//  - the camera sees past it: it projects into the current image, in
//    range, and the depth there is more than clear_margin beyond it (the
//    obstacle has gone). No depth there (too close, too far) clears
//    nothing;
//  - it's older than max_age, or farther than keep_radius from the robot;
//  - it's inside the robot's footprint box (base_frame): it can't be an
//    obstacle there, and collision_monitor would stop for good.
// The strip right in front of the bumper is below the camera's view, so a
// voxel there is only forgotten by age (or when the base backs off and the
// camera sees the spot again).
//
// Heights: points z_min..z_max in odom (odom's z = 0 is the floor, see
// odometry.launch.py). Topics: depth, camera_info (as depth_to_cloud),
// points (PointCloud2 in odom_frame, x/y/z float32). Each frame is used
// once TF can place it in odom (the same wait as depth_to_cloud).
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{
Eigen::Isometry3d toEigen(const geometry_msgs::msg::TransformStamped & t)
{
  const auto & q = t.transform.rotation;
  const auto & p = t.transform.translation;
  Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
  m.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  m.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return m;
}
}  // namespace

class ObstacleMemory : public rclcpp::Node
{
public:
  ObstacleMemory()
  : Node("obstacle_memory")
  {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    frame_id_ = declare_parameter("frame_id", std::string());
    voxel_ = declare_parameter("voxel_size", 0.05);
    min_range_ = declare_parameter("min_range", 0.2);
    max_range_ = declare_parameter("max_range", 3.0);
    z_min_ = declare_parameter("z_min", 0.05);
    z_max_ = declare_parameter("z_max", 1.5);
    clear_margin_ = declare_parameter("clear_margin", 0.1);
    max_age_ = declare_parameter("max_age", 30.0);
    keep_radius_ = declare_parameter("keep_radius", 3.0);
    fp_ = {declare_parameter("footprint_min_x", -0.57), declare_parameter("footprint_max_x", 0.42),
      declare_parameter("footprint_min_y", -0.31), declare_parameter("footprint_max_y", 0.31)};
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
    info_sub_ = create_subscription<CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(), [this](CameraInfo::ConstSharedPtr m) {info_ = m;});
    depth_sub_ = create_subscription<Image>(
      "depth", rclcpp::SensorDataQoS(), [this](Image::ConstSharedPtr m) {
        pending_.push_back(m);
        while (pending_.size() > 3) {
          pending_.pop_front();
        }
        release();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {release();});
  }

private:
  struct Voxel
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int n = 0;
    double last_seen = 0;
    Eigen::Vector3d centre() const {return sum / n;}
  };

  int64_t key(const Eigen::Vector3d & p) const
  {
    int64_t k = 0;
    for (int i = 0; i < 3; ++i) {
      k = (k << 21) | ((static_cast<int64_t>(std::floor(p[i] / voxel_)) + (1 << 20)) & 0x1FFFFF);
    }
    return k;
  }

  // Use the pending frames TF can place in odom (see depth_to_cloud).
  void release()
  {
    while (!pending_.empty()) {
      const auto img = pending_.front();
      const std::string frame = frame_id_.empty() ? img->header.frame_id : frame_id_;
      const rclcpp::Time stamp(img->header.stamp);
      if (tf_buffer_->canTransform(odom_frame_, frame, stamp + rclcpp::Duration::from_seconds(0.05)) &&
        tf_buffer_->canTransform(odom_frame_, base_frame_, stamp))
      {
        pending_.pop_front();
        try {
          update(*img, frame, stamp);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        }
      } else if (now() - stamp > rclcpp::Duration::from_seconds(0.5)) {
        pending_.pop_front();
      } else {
        return;
      }
    }
  }

  void update(const Image & img, const std::string & frame, const rclcpp::Time & stamp)
  {
    namespace enc = sensor_msgs::image_encodings;
    if (!info_) {
      return;
    }
    const bool mm = img.encoding == enc::TYPE_16UC1 || img.encoding == enc::MONO16;
    if (!mm && img.encoding != enc::TYPE_32FC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported depth encoding %s", img.encoding.c_str());
      return;
    }
    const double sx = info_->width ? double(img.width) / info_->width : 1.0;
    const double sy = info_->height ? double(img.height) / info_->height : 1.0;
    const double fx = info_->k[0] * sx, cx = info_->k[2] * sx, fy = info_->k[4] * sy, cy = info_->k[5] * sy;
    if (fx <= 0 || fy <= 0) {
      return;
    }
    const Eigen::Isometry3d odom_cam = toEigen(tf_buffer_->lookupTransform(odom_frame_, frame, stamp));
    const Eigen::Isometry3d odom_base = toEigen(tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp));
    const Eigen::Isometry3d cam_odom = odom_cam.inverse(), base_odom = odom_base.inverse();
    const double t = stamp.seconds();

    auto depth_at = [&](int u, int v) {
        const uint8_t * p = img.data.data() + v * img.step;
        if (mm) {
          uint16_t d;
          std::memcpy(&d, p + 2 * u, 2);
          return d * 1e-3;
        }
        float d;
        std::memcpy(&d, p + 4 * u, 4);
        return double(d);
      };

    // Forget what the camera now sees past, what's too old or far, and
    // what's inside the footprint.
    for (auto it = voxels_.begin(); it != voxels_.end(); ) {
      const Eigen::Vector3d c = it->second.centre();
      const Eigen::Vector3d b = base_odom * c;
      bool forget = t - it->second.last_seen > max_age_ || b.head<2>().norm() > keep_radius_ ||
        (b.x() >= fp_[0] && b.x() <= fp_[1] && b.y() >= fp_[2] && b.y() <= fp_[3]);
      if (!forget) {
        const Eigen::Vector3d p = cam_odom * c;
        if (p.z() >= min_range_ && p.z() <= max_range_) {
          const int u = std::lround(fx * p.x() / p.z() + cx), v = std::lround(fy * p.y() / p.z() + cy);
          if (u >= 0 && v >= 0 && u < int(img.width) && v < int(img.height)) {
            const double d = depth_at(u, v);
            forget = std::isfinite(d) && d > 0 && d > p.z() + clear_margin_;
          }
        }
      }
      it = forget ? voxels_.erase(it) : std::next(it);
    }

    // Remember what it sees now.
    for (uint32_t v = 0; v < img.height; ++v) {
      for (uint32_t u = 0; u < img.width; ++u) {
        const double z = depth_at(u, v);
        if (!std::isfinite(z) || z <= 0) {
          continue;
        }
        const Eigen::Vector3d p((u - cx) * z / fx, (v - cy) * z / fy, z);
        const double r = p.norm();
        if (r < min_range_ || r > max_range_) {
          continue;
        }
        const Eigen::Vector3d o = odom_cam * p;
        if (o.z() < z_min_ || o.z() > z_max_) {
          continue;
        }
        Voxel & vx = voxels_[key(o)];
        if (vx.n > 50) {  // plenty for a centroid; keep it bounded
          vx.sum *= 50.0 / vx.n;
          vx.n = 50;
        }
        vx.sum += o;
        ++vx.n;
        vx.last_seen = t;
      }
    }
    publish(img.header.stamp);
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = odom_frame_;
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
      const Eigen::Vector3d c = kv.second.centre();
      *d++ = c.x();
      *d++ = c.y();
      *d++ = c.z();
    }
    pub_->publish(out);
  }

  std::string odom_frame_, base_frame_, frame_id_;
  double voxel_, min_range_, max_range_, z_min_, z_max_, clear_margin_, max_age_, keep_radius_;
  std::vector<double> fp_;
  CameraInfo::ConstSharedPtr info_;
  std::deque<Image::ConstSharedPtr> pending_;
  std::unordered_map<int64_t, Voxel> voxels_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMemory>());
  rclcpp::shutdown();
  return 0;
}
