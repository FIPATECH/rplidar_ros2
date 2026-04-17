# rplidar_ros2

ROS 2 driver package for SLAMTEC RPLIDAR devices.

## Current workspace usage

- Historical integration in this workspace used a top-mounted `RPLIDAR A2M8`.
- The current hardware target is `RPLIDAR S2M1-R2`.
- The node remains generic and keeps the public ROS contracts owned by the robot bringup:
  - frame: `rplidar_top_link`
  - raw topic: `scan_top`
  - filtered topic: `scan_top_filtered`
  - bringup toggle: `enable_top_lidar`

## Recommended S2M1-R2 parameters

These defaults match the official SLAMTEC S2 documentation and the runtime probe
performed in this workspace on 2026-04-18:

- `serial_baudrate:=1000000`
- `scan_mode:=''`
  - leaving `scan_mode` empty lets the node select the lidar's typical mode
    dynamically
  - on the attached S2M1-R2 validated in this workspace, the node reported
    `current scan mode: Standard`
- `angle_compensate:=true`

## Notes

- The vendored SDK still reports version `1.12.0`, but the S2M1-R2 starts and
  publishes correctly at `1000000` baud in this workspace.
- Keep the node generic. Model-specific naming should live in robot bringup and
  robot documentation, not in the reusable driver executable.
