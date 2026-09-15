// ROS2 node for the Enpose 6-DoF optical marker tracking system.
//
// On startup the node connects to an Enpose sensor: either the one whose IPv4
// address is given by the `ip` parameter, or — when that is empty (the default)
// — the first compatible device found by network discovery. It then streams
// marker poses on a background thread and, for every detected marker, publishes
//
//   * a geometry_msgs/PoseStamped on  markers/marker_<id>/pose
//   * a TF transform  frame_id -> marker_<id>
//
// Discovery is the only step that is retried: until a sensor is found, the node
// keeps searching instead of exiting, and the periodic status log reports a
// warning rather than a pose rate. Connecting itself needs no retry or
// reconnection logic — the pose stream is a self-maintaining UDP subscription
// (the library re-subscribes once a second), so a sensor that is briefly
// unreachable or simply sends no poses for a long time recovers on its own. Once
// connected, the node logs the average number of poses received per second every
// two seconds (0.0/s when the sensor is idle).
//
// Timestamps: the device reports a free-running microsecond clock with an
// arbitrary epoch, so it cannot be used as a ROS time directly. Instead the node
// estimates the offset between the device clock and ROS time with a one-sided
// minimum filter and stamps each pose by its own device timestamp plus that
// offset. This removes receive-side jitter and keeps the distinct capture times
// of frames that the library batched into a single receive() call. Set the
// `use_device_timestamps` parameter to false to fall back to the ROS receive
// time instead.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "enpose_api.hpp"

using namespace std::chrono_literals;

class EnposeNode : public rclcpp::Node
{
public:
  EnposeNode()
  : rclcpp::Node("enpose_node")
  {
    // Parameters.
    ip_ = declare_parameter<std::string>("ip", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "enpose");
    marker_frame_prefix_ = declare_parameter<std::string>("marker_frame_prefix", "marker_");
    retry_period_ = declare_parameter<double>("discovery_retry_period", 2.0);
    use_device_time_ = declare_parameter<bool>("use_device_timestamps", true);
    max_skew_rate_ = declare_parameter<double>("max_clock_skew_rate", 200e-6);

    // Reliability of the pose topics. A reliable publisher is compatible with
    // both reliable and best-effort subscribers; a best-effort one only with
    // best-effort subscribers — so reliable is the default.
    const std::string reliability = declare_parameter<std::string>("reliability", "reliable");
    pose_qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
    if (reliability == "best_effort") {
      pose_qos_.best_effort();
    } else {
      if (reliability != "reliable") {
        RCLCPP_WARN(get_logger(),
                    "Unknown reliability '%s'; using 'reliable'. Valid: reliable, best_effort.",
                    reliability.c_str());
      }
      pose_qos_.reliable();
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_stats_time_ = std::chrono::steady_clock::now();
    stats_timer_ = create_wall_timer(2s, std::bind(&EnposeNode::logStats, this));

    // All connecting and receiving happens on a dedicated thread: discovery and
    // receive(block=true) both sleep, so they must not run on the ROS executor,
    // and keeping them off it lets the node retry connections without blocking.
    running_ = true;
    worker_thread_ = std::thread(&EnposeNode::workerLoop, this);
  }

  ~EnposeNode() override
  {
    running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  // Find a sensor (retrying discovery until one appears) and then stream from it.
  // Runs until the node shuts down.
  void workerLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      std::unique_ptr<enpose::PoseStream> stream = tryConnect();
      if (!stream) {
        // No sensor yet; the periodic status log reports the warning. Wait
        // before retrying discovery so we do not busy-loop.
        interruptibleSleep(retry_period_);
        continue;
      }
      // New session: the device clock has an arbitrary epoch, so re-anchor the
      // offset estimator from this connection's first poses.
      resetClockOffset();
      connected_ = true;
      streamPoses(*stream);
      connected_ = false;

      // streamPoses only returns early on a stream error (in single-threaded
      // mode receive() can fail, e.g. ECONNREFUSED from an ICMP port-unreachable).
      // Back off before reconnecting so a persistent error cannot busy-loop.
      if (running_.load() && rclcpp::ok()) {
        interruptibleSleep(retry_period_);
      }
    }
  }

  // One connection attempt. Returns a live stream, or nullptr when there is no
  // sensor to connect to yet (none discovered, or none compatible).
  std::unique_ptr<enpose::PoseStream> tryConnect()
  {
    std::string target_ip = ip_;

    if (target_ip.empty()) {
      // Discover and choose the first compatible sensor.
      std::vector<enpose::DeviceInfo> devices;
      try {
        devices = enpose::discover();
      } catch (const enpose::Error & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Device discovery failed: %s", e.what());
        return nullptr;
      }
      for (const auto & device : devices) {
        if (device.compatible) {
          target_ip = device.ip;
          break;
        }
      }
      if (target_ip.empty()) {
        // No (compatible) sensor on the network; keep retrying.
        return nullptr;
      }
    }

    try {
      // Single-threaded mode: no library receiver thread; this node's worker
      // thread drains the socket itself on each receive() call.
      auto stream = std::make_unique<enpose::PoseStream>(target_ip, /*create_thread=*/false);
      RCLCPP_INFO(get_logger(), "Connected to Enpose sensor at %s.", target_ip.c_str());
      return stream;
    } catch (const enpose::Error & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to connect to %s: %s", target_ip.c_str(), e.what());
      return nullptr;
    }
  }

  // Receive and publish poses until the node shuts down. A sensor that sends no
  // poses for a while is normal and not an error — the library keeps the UDP
  // subscription alive — so we simply keep waiting. Only an unrecoverable stream
  // error returns, letting the worker loop re-establish the subscription.
  void streamPoses(enpose::PoseStream & stream)
  {
    while (running_.load() && rclcpp::ok()) {
      std::vector<enpose::MarkerPose> poses;
      try {
        // Blocks until a pose update arrives, up to a 3 s timeout.
        poses = stream.receive(/*block=*/true);
      } catch (const enpose::Error & e) {
        RCLCPP_WARN(get_logger(), "Pose stream error: %s. Reconnecting...", e.what());
        return;
      }

      if (poses.empty()) {
        continue;
      }

      if (use_device_time_) {
        // Map each pose's own device timestamp to ROS time, so frames batched
        // into one receive() keep their distinct capture instants.
        updateClockOffset(poses);
        for (const auto & pose : poses) {
          publishPose(pose, deviceToRosTime(pose.timestamp));
        }
      } else {
        // Fallback: stamp the whole batch with one receive time.
        const rclcpp::Time stamp = now();
        for (const auto & pose : poses) {
          publishPose(pose, stamp);
        }
      }
      poses_received_.fetch_add(poses.size(), std::memory_order_relaxed);
    }
  }

  void publishPose(const enpose::MarkerPose & pose, const rclcpp::Time & stamp)
  {
    // The library reports a row-major 3x3 rotation matrix (model frame to
    // world); ROS messages carry a quaternion, so convert it.
    const auto & r = pose.rotation;
    tf2::Matrix3x3 m(
      r[0], r[1], r[2],
      r[3], r[4], r[5],
      r[6], r[7], r[8]);
    tf2::Quaternion q;
    m.getRotation(q);
    q.normalize();

    const std::string child_frame = marker_frame_prefix_ + std::to_string(pose.marker_id);

    // PoseStamped on a per-marker topic.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.position.z = pose.z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    getPosePublisher(pose.marker_id, child_frame)->publish(pose_msg);

    // TF: frame_id -> marker_<id>.
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = pose.x;
    tf_msg.transform.translation.y = pose.y;
    tf_msg.transform.translation.z = pose.z;
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // --- Device-clock to ROS-clock offset estimation (worker thread only) ---

  // Drop the current offset estimate so it re-anchors from the next batch.
  void resetClockOffset()
  {
    offset_valid_ = false;
  }

  // Update the device->ROS time offset from one (non-empty) receive() batch,
  // using a one-sided minimum filter: the offset snaps down instantly to the
  // lowest-latency sample seen and is allowed to drift up only slowly, bounded
  // by `max_skew_rate_`, to track the relative skew of the two free-running
  // clocks. Must be called once per batch, before stamping its poses.
  void updateClockOffset(const std::vector<enpose::MarkerPose> & poses)
  {
    // The most recent (largest) device timestamp in the batch is the
    // lowest-latency sample: it left the device most recently.
    uint64_t t_dev_us = 0;
    for (const auto & pose : poses) {
      t_dev_us = std::max(t_dev_us, pose.timestamp);
    }

    // A device restart resets its clock toward zero, so a large backward jump
    // means the epoch changed: re-anchor the offset from this batch.
    if (offset_valid_ && t_dev_us < last_device_us_ &&
        (last_device_us_ - t_dev_us) > kRestartBackwardThresholdUs)
    {
      RCLCPP_WARN(get_logger(),
                  "Device timestamp jumped backward (%llu -> %llu us); "
                  "reinitializing clock offset.",
                  static_cast<unsigned long long>(last_device_us_),
                  static_cast<unsigned long long>(t_dev_us));
      offset_valid_ = false;
    }
    last_device_us_ = t_dev_us;

    const rclcpp::Time recv = now();
    const int64_t t_dev_ns = static_cast<int64_t>(t_dev_us) * 1000;
    const int64_t instant_offset = recv.nanoseconds() - t_dev_ns;

    if (!offset_valid_) {
      clock_offset_ns_ = instant_offset;
      offset_valid_ = true;
    } else {
      const double dt = std::max(0.0, (recv - last_offset_update_).seconds());
      const int64_t max_rise = static_cast<int64_t>(max_skew_rate_ * dt * 1e9);
      // Snap down to a lower-latency sample, else drift up by at most max_rise.
      clock_offset_ns_ = std::min(instant_offset, clock_offset_ns_ + max_rise);
    }
    last_offset_update_ = recv;
  }

  // Map a device timestamp (microseconds since device start) to ROS time via the
  // current offset. Precondition: offset_valid_ (updateClockOffset ran first).
  // The constructed Time carries the node clock's source so it compares cleanly
  // with now() and honors use_sim_time. Stamps never land in the future: the
  // offset is bounded above by recv - t_dev_ns for the batch's latest pose.
  rclcpp::Time deviceToRosTime(uint64_t device_us) const
  {
    const int64_t ros_ns = static_cast<int64_t>(device_us) * 1000 + clock_offset_ns_;
    return rclcpp::Time(ros_ns, now().get_clock_type());
  }

  // Lazily create a PoseStamped publisher per marker id. Only touched from the
  // worker thread.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  getPosePublisher(uint16_t marker_id, const std::string & child_frame)
  {
    auto it = pose_publishers_.find(marker_id);
    if (it != pose_publishers_.end()) {
      return it->second;
    }
    const std::string topic = "markers/" + child_frame + "/pose";
    auto publisher = create_publisher<geometry_msgs::msg::PoseStamped>(topic, pose_qos_);
    RCLCPP_INFO(get_logger(), "Detected new marker %u; publishing on %s",
                marker_id, publisher->get_topic_name());
    pose_publishers_.emplace(marker_id, publisher);
    return publisher;
  }

  void logStats()
  {
    const auto now_time = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now_time - last_stats_time_).count();
    const uint64_t count = poses_received_.exchange(0, std::memory_order_relaxed);
    last_stats_time_ = now_time;

    if (connected_.load()) {
      // Connected: report the rate, even when it is 0.0/s (an idle sensor).
      const double rate = elapsed > 0.0 ? static_cast<double>(count) / elapsed : 0.0;
      RCLCPP_INFO(get_logger(), "Average poses received: %.1f /s", rate);
    } else {
      // Still searching for a sensor.
      RCLCPP_WARN(get_logger(), "No Enpose sensor found yet; retrying discovery...");
    }
  }

  // Sleep for `seconds`, waking early if the node is shutting down so the worker
  // thread stays responsive to shutdown.
  void interruptibleSleep(double seconds)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (running_.load() && rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(50ms);
    }
  }

  // Parameters.
  std::string ip_;
  std::string frame_id_;
  std::string marker_frame_prefix_;
  double retry_period_{2.0};
  bool use_device_time_{true};
  double max_skew_rate_{200e-6};

  // Clock offset estimator (worker thread only; no synchronization needed).
  // ros_ns = device_us * 1000 + clock_offset_ns_.
  bool offset_valid_{false};
  int64_t clock_offset_ns_{0};
  uint64_t last_device_us_{0};
  rclcpp::Time last_offset_update_;
  static constexpr uint64_t kRestartBackwardThresholdUs = 1'000'000;  // 1 s

  rclcpp::QoS pose_qos_{rclcpp::KeepLast(10)};  // reliability set from the `reliability` param

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<uint16_t, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  pose_publishers_;

  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::atomic<uint64_t> poses_received_{0};
  std::chrono::steady_clock::time_point last_stats_time_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::thread worker_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnposeNode>());
  rclcpp::shutdown();
  return 0;
}
