# enpose_ros

A ROS 2 node for the [Enpose](https://enpose.tech) 6-DoF optical marker
tracking system. It connects to an Enpose sensor, streams marker poses, and
republishes them as ROS messages and TF.

## What it does

On startup the node connects to a sensor:

- by default it **discovers** sensors on the local network and connects to the
  first compatible one;
- alternatively, set the `ip` parameter to connect to a specific sensor
  directly.

For every detected marker it then publishes:

- a `geometry_msgs/PoseStamped` on `markers/marker_<id>/pose`
  (in the `frame_id` frame), and
- a TF transform `frame_id` → `marker_<id>`.

Every two seconds it logs the average number of poses received per second.

Until a sensor is found, the node keeps retrying discovery instead of exiting,
and the two-second status log reports a warning rather than a pose rate.
Connecting itself needs no retry logic: the pose stream is a self-maintaining UDP
subscription, so a connected sensor that is briefly unreachable or simply sends
no poses for a long time recovers on its own (the status log just reports
`0.0 /s` while it is idle).

## Building

The build pulls in the Enpose API (tag `0.1.0`) with CMake `FetchContent` and
compiles its Rust core, so a **Rust toolchain (`cargo`)** must be on `PATH`.

```bash
colcon build
source install/setup.bash
```

## Running

Connect to the first sensor found on the network:

```bash
ros2 run enpose_ros enpose_node
```

Connect to a specific sensor:

```bash
ros2 run enpose_ros enpose_node --ros-args -p ip:=192.168.1.42
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ip` | string | `""` | Sensor IPv4 address. Empty → discover and use the first compatible sensor. |
| `frame_id` | string | `enpose` | Parent/world frame for the published poses and TF. |
| `marker_frame_prefix` | string | `marker_` | Prefix for per-marker frame and topic names (e.g. `marker_7`). |
| `discovery_retry_period` | double | `2.0` | Seconds to wait between discovery attempts while no sensor is found. |
| `reliability` | string | `reliable` | QoS reliability of the pose topics: `reliable` or `best_effort` (see below). |
| `use_device_timestamps` | bool | `true` | Stamp poses from the device clock mapped into ROS time (see below). Set `false` to stamp with the ROS receive time instead. |
| `max_clock_skew_rate` | double | `200e-6` | Upper bound (s/s) on how fast the estimated device→ROS clock offset may drift up, tracking crystal skew. Only used when `use_device_timestamps` is true. |

### QoS

The pose topics default to **reliable** QoS. A reliable publisher is compatible
with both reliable and best-effort subscribers, whereas a best-effort publisher
is incompatible with reliable subscribers (they receive nothing). The default
therefore works with any subscriber, including rviz regardless of its Reliability
setting. Set `reliability` to `best_effort` for high-rate or lossy setups where
dropping samples is preferable to retransmission. The TF (`/tf`) output is always
reliable (the tf2_ros default).

### Timestamps

The sensor reports a free-running microsecond clock with an arbitrary epoch, so
its values can't be used as ROS times directly. With `use_device_timestamps`
(the default), the node estimates the offset between the device clock and ROS
time using a one-sided minimum filter — it snaps the offset down to the
lowest-latency packet seen and lets it drift up slowly (bounded by
`max_clock_skew_rate`) to follow the relative skew of the two clocks. Each pose
is then stamped by its *own* device timestamp plus that offset. This removes
receive-side jitter and preserves the distinct capture times of frames that the
library batched into a single update. Setting `use_device_timestamps` to `false`
falls back to stamping each batch with the ROS receive time.
