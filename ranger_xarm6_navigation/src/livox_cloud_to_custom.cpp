// Livox PointCloud2 (livox_ros_driver2 xfer_format 0) -> livox_ros_driver2
// CustomMsg, for FAST-LIO.
//
// FAST-LIO reads a Livox lidar's per-point times only from CustomMsg
// (offset_time), but the driver can publish only one format, and RViz,
// Nav2 and everything else want PointCloud2. So the driver (and the sim,
// gz_lidar_to_pointcloud.py, which publishes the same layout) stays on
// xfer_format 0, whose points carry their absolute time in ns
// ('timestamp', float64), and this converts. timebase and header.stamp
// are the earliest point's time; offset_time is each point's time after it.
//
// A scan whose points all share one time (gz-sim renders a scan at one
// instant) gets header.stamp moved back by scan_period and every
// offset_time set to scan_period instead: FAST-LIO takes a scan's end
// time from its last point's offset, and one of ~0 makes it assume the
// scan ended a mean scan period after header.stamp, i.e. 0.1 s late.
// Stamped this way, the scan ends at its true capture time, and every
// point is at that end, so FAST-LIO's undistortion leaves it as it is.
//
// Parameters: scan_period [s] (0.1, the Mid-360 at 10 Hz).
// Topics: in 'livox/lidar' (PointCloud2), out 'livox/lidar_custom'.

#include <cstring>
#include <limits>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

struct FieldOffsets
{
  int x = -1, y = -1, z = -1, intensity = -1, tag = -1, line = -1, timestamp = -1;
};

FieldOffsets find_fields(const sensor_msgs::msg::PointCloud2 & cloud)
{
  FieldOffsets f;
  for (const auto & field : cloud.fields) {
    const auto & n = field.name;
    const int o = static_cast<int>(field.offset);
    if (n == "x") {f.x = o;} else if (n == "y") {f.y = o;} else if (n == "z") {f.z = o;} else if (
      n == "intensity") {f.intensity = o;} else if (n == "tag") {f.tag = o;} else if (n == "line") {
      f.line = o;
    } else if (n == "timestamp") {f.timestamp = o;}
  }
  return f;
}

template<typename T>
T read(const uint8_t * p, int offset)
{
  T v;
  std::memcpy(&v, p + offset, sizeof(T));
  return v;
}

}  // namespace

class LivoxCloudToCustom : public rclcpp::Node
{
public:
  LivoxCloudToCustom()
  : Node("livox_cloud_to_custom")
  {
    scan_period_ns_ = static_cast<uint64_t>(declare_parameter("scan_period", 0.1) * 1e9);
    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar_custom", 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {convert(*msg);});
  }

private:
  void convert(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const FieldOffsets f = find_fields(cloud);
    if (f.x < 0 || f.y < 0 || f.z < 0 || f.timestamp < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cloud has no x/y/z/timestamp fields; expected livox_ros_driver2 xfer_format 0.");
      return;
    }
    const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
    const uint8_t * data = cloud.data.data();

    double t_min = std::numeric_limits<double>::max(), t_max = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double t = read<double>(data + i * cloud.point_step, f.timestamp);
      t_min = std::min(t_min, t);
      t_max = std::max(t_max, t);
    }
    if (n == 0) {
      t_min = t_max = static_cast<double>(rclcpp::Time(cloud.header.stamp).nanoseconds());
    }
    // Instantaneous scan (see top): move the start back by one period.
    const bool instant = (t_max - t_min) < 1e6;  // < 1 ms
    const uint64_t base = static_cast<uint64_t>(t_min) - (instant ? scan_period_ns_ : 0);

    livox_ros_driver2::msg::CustomMsg out;
    out.header.frame_id = cloud.header.frame_id;
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(base), RCL_ROS_TIME);
    out.timebase = base;
    out.point_num = static_cast<uint32_t>(n);
    out.points.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = data + i * cloud.point_step;
      auto & q = out.points[i];
      q.x = read<float>(p, f.x);
      q.y = read<float>(p, f.y);
      q.z = read<float>(p, f.z);
      q.reflectivity = f.intensity >= 0 ?
        static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, read<float>(p, f.intensity)))) : 0;
      q.tag = f.tag >= 0 ? read<uint8_t>(p, f.tag) : 0;
      q.line = f.line >= 0 ? read<uint8_t>(p, f.line) : 0;
      q.offset_time = static_cast<uint32_t>(read<double>(p, f.timestamp) - static_cast<double>(base));
    }
    pub_->publish(out);
  }

  uint64_t scan_period_ns_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCloudToCustom>());
  rclcpp::shutdown();
  return 0;
}
