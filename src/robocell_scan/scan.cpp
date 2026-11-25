#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>

#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace
{
using Scan = sensor_msgs::msg::LaserScan;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

bool valid(const Scan & scan)
{
  return !scan.header.frame_id.empty() &&
    (scan.header.stamp.sec > 0 ||
    (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec > 0)) &&
    !scan.ranges.empty() &&
    (scan.intensities.empty() || scan.intensities.size() == scan.ranges.size()) &&
    std::isfinite(scan.angle_min) && std::isfinite(scan.angle_max) &&
    std::isfinite(scan.angle_increment) && scan.angle_increment != 0 &&
    std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
    scan.range_min > 0 && scan.range_max > scan.range_min &&
    std::isfinite(scan.time_increment) && scan.time_increment >= 0 &&
    std::isfinite(scan.scan_time) && scan.scan_time >= 0;
}

Scan rectified(
  const Scan & scan, tf2_ros::Buffer & buffer, laser_geometry::LaserProjection & projection,
  const std::string & fixed, std::size_t count)
{
  const auto duration = (scan.ranges.size() - 1) * static_cast<double>(scan.time_increment);
  const auto stamp = rclcpp::Time(scan.header.stamp) +
    rclcpp::Duration::from_nanoseconds(std::llround(duration * 1e9));
  const auto transform = buffer.lookupTransform(scan.header.frame_id, fixed, stamp).transform;
  const tf2::Transform reference(
    {transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w},
    {transform.translation.x, transform.translation.y, transform.translation.z});
  sensor_msgs::msg::PointCloud2 cloud;
  const auto cutoff = std::nextafter(
    static_cast<double>(scan.range_max), std::numeric_limits<double>::infinity());
  if (scan.ranges.size() == 1) {
    projection.projectLaser(scan, cloud, cutoff, laser_geometry::channel_option::Index);
  } else {
    projection.transformLaserScanToPointCloud(
      fixed, scan, cloud, buffer, cutoff, laser_geometry::channel_option::Index);
  }
  Scan result;
  result.header = scan.header;
  result.header.stamp = stamp;
  result.angle_min = -pi;
  result.angle_increment = 2 * pi / count;
  result.angle_max = -pi + (count - 1) * (2 * pi / count);
  result.scan_time = scan.scan_time;
  result.range_min = scan.range_min;
  result.range_max = scan.range_max;
  result.ranges.assign(count, std::numeric_limits<float>::infinity());
  result.intensities.assign(count, 0.0f);
  if (cloud.width == 0) {
    return result;
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<std::int32_t> index(cloud, "index");
  for (; x != x.end(); ++x, ++y, ++z, ++index) {
    const tf2::Vector3 sample(*x, *y, *z);
    const auto point = scan.ranges.size() == 1 ? sample : reference * sample;
    const float range = std::hypot(point.x(), point.y());
    if (!(scan.range_min <= range && range <= scan.range_max)) {
      continue;
    }
    const auto slot = std::lround((std::atan2(point.y(), point.x()) + pi) * count / (2 * pi)) %
      count;
    if (range < result.ranges[slot]) {
      result.ranges[slot] = range;
      if (!scan.intensities.empty()) {
        result.intensities[slot] = scan.intensities[*index];
      }
    }
  }
  return result;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("slam_scan");
  const auto input = node->declare_parameter<std::string>("input", "/scan");
  const auto output = node->declare_parameter<std::string>("output", "/scan/slam");
  const auto fixed = node->declare_parameter<std::string>("fixed_frame", "odom");
  const auto preserve_sector = node->declare_parameter<bool>("preserve_sector", false);
  const auto bins = node->declare_parameter<int>("bins", 360);
  const auto capacity = node->declare_parameter<int>("capacity", 8);
  const auto timeout = node->declare_parameter<double>("transform_timeout", 0.5);
  const auto topics = node->get_node_topics_interface();
  if (topics->resolve_topic_name(input) == topics->resolve_topic_name(output) ||
    fixed.empty() || bins <= 1 || capacity < 1 || !std::isfinite(timeout) || timeout <= 0)
  {
    throw std::invalid_argument("SLAM scan requires distinct topics and positive limits");
  }
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node, false);
  laser_geometry::LaserProjection projection;
  const auto topic = node->create_publisher<Scan>(output, 10);
  std::deque<std::pair<Scan::ConstSharedPtr, Clock::time_point>> pending;
  const auto source = node->create_subscription<Scan>(
    input, rclcpp::SensorDataQoS(), [&](Scan::ConstSharedPtr scan) {
      if (!valid(*scan)) {
        RCLCPP_WARN(node->get_logger(), "SLAM scan rejected invalid scan metadata");
        return;
      }
      if (pending.size() >= static_cast<std::size_t>(capacity)) {
        pending.pop_front();
        RCLCPP_WARN(node->get_logger(), "SLAM scan queue dropped its oldest rotation");
      }
      pending.emplace_back(std::move(scan), Clock::now());
    });
  const auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
      while (!pending.empty()) {
        const auto & [scan, received] = pending.front();
        if (Clock::now() - received >= std::chrono::duration<double>(timeout)) {
          pending.pop_front();
          RCLCPP_WARN(node->get_logger(), "SLAM scan expired while waiting for odometry TF");
          continue;
        }
        try {
          if (preserve_sector && scan->time_increment == 0) {
            buffer.lookupTransform(fixed, scan->header.frame_id, rclcpp::Time(scan->header.stamp));
            topic->publish(*scan);
          } else {
            topic->publish(rectified(*scan, buffer, projection, fixed, bins));
          }
        } catch (const tf2::TransformException &) {
          break;
        }
        pending.pop_front();
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
