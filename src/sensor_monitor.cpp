// sensor_monitor: subscribes to every sensor, measures rate / staleness, checks GPS fix,
// estimates nearest obstacle from lidar + depth image, and publishes /diagnostics.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Channel
{
  std::string topic;
  double min_rate_hz{0.0};
  std::size_t count{0};
  bool seen{false};
  rclcpp::Time last_rx;
};

KeyValue kv(const std::string & key, double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", value);
  KeyValue out;
  out.key = key;
  out.value = buf;
  return out;
}
}  // namespace

class SensorMonitor : public rclcpp::Node
{
public:
  SensorMonitor()
  : Node("sensor_monitor")
  {
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 1.0);
    channels_["lidar"] = Channel{"scan", declare_parameter<double>("lidar_min_hz", 5.0)};
    channels_["depth"] = Channel{"camera/depth/image_raw", declare_parameter<double>("depth_min_hz", 5.0)};
    channels_["gps"] = Channel{"gps/fix", declare_parameter<double>("gps_min_hz", 5.0)};
    channels_["imu"] = Channel{"imu/data", declare_parameter<double>("imu_min_hz", 20.0)};

    const auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      channels_["lidar"].topic, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      channels_["depth"].topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onDepth(*m);});
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      channels_["gps"].topic, qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {onGps(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      channels_["imu"].topic, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {touch("imu");});

    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    dist_pub_ = create_publisher<std_msgs::msg::Float32>("min_obstacle_distance", 10);

    last_report_ = now();
    timer_ = create_wall_timer(1s, [this] {report();});
  }

private:
  void touch(const std::string & name)
  {
    auto & c = channels_.at(name);
    ++c.count;
    c.seen = true;
    c.last_rx = now();
  }

  void onScan(const sensor_msgs::msg::LaserScan & m)
  {
    touch("lidar");
    float best = kInf;
    for (const float r : m.ranges) {
      if (std::isfinite(r) && r >= m.range_min && r <= m.range_max) {
        best = std::min(best, r);
      }
    }
    scan_min_ = best;
  }

  // Nearest return in the central window of the depth image (upper-middle rows to skip floor)
  void onDepth(const sensor_msgs::msg::Image & m)
  {
    touch("depth");
    if (m.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
      RCLCPP_WARN_ONCE(get_logger(), "Unexpected depth encoding '%s'", m.encoding.c_str());
      return;
    }
    const uint32_t r0 = m.height / 4, r1 = m.height / 2;
    const uint32_t c0 = m.width / 3, c1 = 2 * m.width / 3;
    float best = kInf;
    for (uint32_t r = r0; r < r1; ++r) {
      const uint8_t * row = &m.data[static_cast<std::size_t>(r) * m.step];
      for (uint32_t c = c0; c < c1; ++c) {
        float d;
        std::memcpy(&d, row + c * sizeof(float), sizeof(float));
        if (std::isfinite(d) && d > 0.0f) {
          best = std::min(best, d);
        }
      }
    }
    depth_min_ = best;
  }

  void onGps(const sensor_msgs::msg::NavSatFix & m)
  {
    touch("gps");
    gps_status_ = m.status.status;
  }

  void report()
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_report_).seconds();
    if (dt <= 0.0) {
      return;  // simulation paused
    }
    last_report_ = t;

    DiagnosticArray arr;
    arr.header.stamp = t;
    std::map<std::string, double> rates;

    for (auto & [name, c] : channels_) {
      const double rate = static_cast<double>(c.count) / dt;
      c.count = 0;
      rates[name] = rate;

      DiagnosticStatus st;
      st.name = "sensor_monitor: " + name;
      st.hardware_id = name;
      const double age = c.seen ? (t - c.last_rx).seconds() : -1.0;

      if (!c.seen) {
        st.level = DiagnosticStatus::ERROR;
        st.message = "no data on " + c.topic;
      } else if (age > stale_timeout_s_) {
        st.level = DiagnosticStatus::ERROR;
        st.message = "stale";
      } else if (name == "gps" && gps_status_ < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        st.level = DiagnosticStatus::ERROR;
        st.message = "no GPS fix";
      } else if (rate < c.min_rate_hz) {
        st.level = DiagnosticStatus::WARN;
        st.message = "low rate";
      } else {
        st.level = DiagnosticStatus::OK;
        st.message = "ok";
      }
      KeyValue topic;
      topic.key = "topic";
      topic.value = c.topic;
      st.values = {topic, kv("rate_hz", rate), kv("age_s", age)};
      arr.status.push_back(st);
    }
    diag_pub_->publish(arr);

    const float nearest = std::min(scan_min_, depth_min_);
    if (std::isfinite(nearest)) {
      std_msgs::msg::Float32 msg;
      msg.data = nearest;
      dist_pub_->publish(msg);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "lidar %.1f Hz | depth %.1f Hz | gps %.1f Hz (status %d) | imu %.1f Hz | nearest %.2f m",
      rates["lidar"], rates["depth"], rates["gps"], gps_status_, rates["imu"],
      std::isfinite(nearest) ? nearest : -1.0f);
  }

  double stale_timeout_s_{1.0};
  std::map<std::string, Channel> channels_;
  float scan_min_{kInf};
  float depth_min_{kInf};
  int gps_status_{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  rclcpp::Time last_report_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorMonitor>());
  rclcpp::shutdown();
  return 0;
}
