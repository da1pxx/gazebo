// gps_waypoint_follower: converts GPS waypoints (lat/lon) into map-frame poses via
// navsat_transform's fromLL service and executes them with Nav2 NavigateThroughPoses.
//
// NavigateThroughPoses (not FollowWaypoints) plans ONE path through every pose, so the
// robot keeps its speed through the intermediate waypoints instead of decelerating,
// rotating in place and pausing at each one.
//
// After the mission it verifies that the robot actually stopped inside finish zone B.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class GpsWaypointFollower : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using FromLL = robot_localization::srv::FromLL;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Marker = visualization_msgs::msg::Marker;

  GpsWaypointFollower()
  : Node("gps_waypoint_follower")
  {
    lat_ = declare_parameter<std::vector<double>>("latitudes", std::vector<double>{});
    lon_ = declare_parameter<std::vector<double>>("longitudes", std::vector<double>{});
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    finish_radius_ = declare_parameter<double>("finish_radius", 1.0);

    if (lat_.empty() || lat_.size() != lon_.size()) {
      throw std::invalid_argument("'latitudes' and 'longitudes' must be non-empty and equal length");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    from_ll_ = create_client<FromLL>("fromLL");
    nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");
    marker_pub_ = create_publisher<MarkerArray>(
      "gps_waypoints/markers", rclcpp::QoS(1).transient_local());

    // navsat_transform publishes odometry/gps only once its transform is valid,
    // i.e. fromLL returns meaningful results only after the first message here.
    gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr) {gps_ready_ = true;});

    timer_ = create_wall_timer(1s, [this] {tick();});
    RCLCPP_INFO(get_logger(), "Loaded %zu GPS waypoints", lat_.size());
  }

private:
  enum class State { kWaiting, kConverting, kNavigating, kDone };

  void tick()
  {
    if (state_ != State::kWaiting) {
      return;
    }
    if (!gps_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for navsat_transform...");
      return;
    }
    if (!from_ll_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for fromLL service...");
      return;
    }
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Nav2 navigate_through_poses...");
      return;
    }
    convertWaypoints();
  }

  void convertWaypoints()
  {
    state_ = State::kConverting;
    map_points_.assign(lat_.size(), geometry_msgs::msg::Point{});
    pending_ = lat_.size();

    for (std::size_t i = 0; i < lat_.size(); ++i) {
      auto req = std::make_shared<FromLL::Request>();
      req->ll_point.latitude = lat_[i];
      req->ll_point.longitude = lon_[i];
      req->ll_point.altitude = 0.0;

      from_ll_->async_send_request(
        req, [this, i](rclcpp::Client<FromLL>::SharedFuture future) {
          map_points_[i] = future.get()->map_point;
          map_points_[i].z = 0.0;
          RCLCPP_INFO(
            get_logger(), "WP%zu  (%.8f, %.8f) -> %s (%.2f, %.2f)", i + 1, lat_[i], lon_[i],
            global_frame_.c_str(), map_points_[i].x, map_points_[i].y);
          if (--pending_ == 0) {
            sendMission();
          }
        });
    }
  }

  static double headingTo(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  std::vector<geometry_msgs::msg::PoseStamped> buildPoses() const
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    const auto stamp = now();
    const std::size_t n = map_points_.size();
    for (std::size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = global_frame_;
      ps.header.stamp = stamp;
      ps.pose.position = map_points_[i];
      // face the next waypoint (or keep the last leg's heading at the finish)
      double yaw = 0.0;
      if (i + 1 < n) {
        yaw = headingTo(map_points_[i], map_points_[i + 1]);
      } else if (i > 0) {
        yaw = headingTo(map_points_[i - 1], map_points_[i]);
      }
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      ps.pose.orientation = tf2::toMsg(q);
      poses.push_back(ps);
    }
    return poses;
  }

  void publishMarkers(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    MarkerArray arr;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const bool last = (i + 1 == poses.size());
      Marker sphere;
      sphere.header = poses[i].header;
      sphere.ns = "gps_waypoints";
      sphere.id = static_cast<int>(2 * i);
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose = poses[i].pose;
      sphere.pose.position.z = 0.2;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
      sphere.color.a = 0.9f;
      sphere.color.r = 1.0f;
      sphere.color.g = last ? 0.1f : 0.8f;
      sphere.color.b = 0.1f;
      arr.markers.push_back(sphere);

      Marker label = sphere;
      label.id = static_cast<int>(2 * i + 1);
      label.type = Marker::TEXT_VIEW_FACING;
      label.pose.position.z = 0.7;
      label.scale.x = label.scale.y = 0.0;
      label.scale.z = 0.4;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.text = last ? "B (finish)" : "WP" + std::to_string(i + 1);
      arr.markers.push_back(label);
    }
    marker_pub_->publish(arr);
  }

  void sendMission()
  {
    NavigateThroughPoses::Goal goal;
    goal.poses = buildPoses();
    publishMarkers(goal.poses);

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;
    opts.goal_response_callback = [this](const GoalHandle::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Mission rejected by Nav2");
          state_ = State::kDone;
          return;
        }
        RCLCPP_INFO(get_logger(), "Mission accepted, driving A -> B");
      };
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> fb) {
        const int remaining = static_cast<int>(fb->number_of_poses_remaining);
        if (remaining > 0 && remaining != remaining_) {
          remaining_ = remaining;
          const std::size_t done = lat_.size() - static_cast<std::size_t>(remaining);
          if (done > 0) {   // skip the initial feedback, when nothing has been passed yet
            RCLCPP_INFO(
              get_logger(), "Passed waypoint %zu/%zu, %.1f m to go", done, lat_.size(),
              fb->distance_remaining);
          }
        }
      };
    opts.result_callback = [this](const GoalHandle::WrappedResult & result) {onResult(result);};

    start_time_ = now();
    state_ = State::kNavigating;
    nav_client_->async_send_goal(goal, opts);
  }

  void onResult(const GoalHandle::WrappedResult & result)
  {
    state_ = State::kDone;
    const double elapsed = (now() - start_time_).seconds();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Mission FAILED (action code %d) after %.1f s",
        static_cast<int>(result.code), elapsed);
      return;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      const auto & b = map_points_.back();
      const double dist = std::hypot(
        tf.transform.translation.x - b.x, tf.transform.translation.y - b.y);
      if (dist <= finish_radius_) {
        RCLCPP_INFO(get_logger(),
          "MISSION COMPLETE: reached B in %.1f s, %.2f m from finish centre", elapsed, dist);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Mission ended %.2f m from B (limit %.2f m)", dist, finish_radius_);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Cannot verify finish position: %s", ex.what());
    }
  }

  // parameters
  std::vector<double> lat_, lon_;
  std::string global_frame_, base_frame_;
  double finish_radius_{1.0};

  // state
  State state_{State::kWaiting};
  bool gps_ready_{false};
  std::size_t pending_{0};
  int remaining_{-1};
  std::vector<geometry_msgs::msg::Point> map_points_;
  rclcpp::Time start_time_;

  // ROS interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<FromLL>::SharedPtr from_ll_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsWaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
