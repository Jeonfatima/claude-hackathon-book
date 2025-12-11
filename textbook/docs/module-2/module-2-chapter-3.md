---
title: "Chapter 3: Sensor Simulation (LiDAR, Cameras, IMU)"
sidebar_position: 4
---

# Chapter 3: Sensor Simulation (LiDAR, Cameras, IMU)

## Introduction

Sensor simulation is crucial for developing and testing robotic systems in virtual environments. This chapter explores how to simulate various sensor types including LiDAR, cameras, and IMU in simulation environments like Gazebo and Unity.

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure and simulate LiDAR sensors in Gazebo
- Set up camera sensors with realistic parameters
- Simulate IMU sensors with appropriate noise models
- Process simulated sensor data in ROS 2

## LiDAR Simulation in Gazebo

LiDAR sensors provide 2D or 3D range data that is essential for navigation and mapping:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Processing LiDAR Data in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        self.obstacle_threshold = 1.0  # meters

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)

        # Filter out invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]

        # Detect obstacles
        obstacles = valid_ranges < self.obstacle_threshold

        if np.any(obstacles):
            self.get_logger().info(f'Obstacles detected: {np.sum(obstacles)} points')

        # Calculate statistics
        if len(valid_ranges) > 0:
            avg_distance = np.mean(valid_ranges)
            self.get_logger().info(f'Average distance: {avg_distance:.2f}m')
```

## Camera Simulation

Camera sensors provide visual information for perception tasks:

```xml
<sensor name="camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Camera Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10
        )
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display processed image
            cv2.imshow('Camera Feed', cv_image)
            cv2.imshow('Edges', edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
```

## IMU Simulation

IMU sensors provide orientation, velocity, and acceleration data:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### IMU Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # For orientation estimation
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # w, x, y, z
        self.prev_time = None

    def imu_callback(self, msg):
        # Extract orientation (if available from sensor)
        orientation = msg.orientation
        orientation_quat = np.array([
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z
        ])

        # Extract angular velocity
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Extract linear acceleration
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Update orientation using angular velocity integration
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            self.update_orientation(angular_velocity, dt)

        self.prev_time = current_time

        # Log the processed data
        self.get_logger().info(f'Orientation: ({orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}, {orientation.w:.3f})')

    def update_orientation(self, angular_velocity, dt):
        # Simple integration to update orientation
        # In practice, you'd use more sophisticated methods like Madgwick or Mahony filters
        half_dt = 0.5 * dt

        # Convert angular velocity to quaternion increment
        omega_quat = np.array([0.0,
                              angular_velocity[0] * half_dt,
                              angular_velocity[1] * half_dt,
                              angular_velocity[2] * half_dt])

        # Update orientation quaternion
        orientation_update = self.quaternion_multiply(omega_quat, self.orientation)
        self.orientation += orientation_update
        self.orientation /= np.linalg.norm(self.orientation)  # Normalize

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])
```

## Multi-Sensor Fusion

Combining data from multiple sensors for improved perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class MultiSensorFusion(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Subscriptions to multiple sensors
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Publisher for fused state
        self.state_pub = self.create_publisher(PoseWithCovarianceStamped, 'fused_state', 10)

        # Internal state
        self.lidar_data = None
        self.imu_data = None
        self.robot_pose = np.zeros(3)  # x, y, theta

    def lidar_callback(self, msg):
        self.lidar_data = msg
        self.update_state()

    def imu_callback(self, msg):
        self.imu_data = msg
        self.update_state()

    def update_state(self):
        if self.lidar_data and self.imu_data:
            # Perform sensor fusion
            # This is a simplified example - real fusion would use Kalman filters, etc.

            # Use IMU for orientation
            orientation = self.imu_data.orientation
            # Use LiDAR for position relative to known landmarks
            # (simplified - real implementation would do SLAM)

            # Create and publish fused state
            pose_msg = PoseWithCovarianceStamped()
            # Fill in the pose and covariance
            self.state_pub.publish(pose_msg)
```

## Sensor Noise and Realism

To make simulations more realistic, proper noise models are essential:

```python
import numpy as np

def add_sensor_noise(data, noise_params):
    """
    Add realistic noise to sensor data
    noise_params: dict with 'mean' and 'stddev' keys
    """
    noise = np.random.normal(noise_params['mean'], noise_params['stddev'], size=data.shape)
    return data + noise

# Example usage for LiDAR with distance-dependent noise
def lidar_with_realistic_noise(raw_ranges, max_range=30.0):
    processed_ranges = []
    for r in raw_ranges:
        if np.isfinite(r):
            # Noise increases with distance (typical for LiDAR)
            noise_std = 0.01 + 0.005 * r  # 1cm + 0.5% of range
            noisy_range = r + np.random.normal(0, noise_std)
            processed_ranges.append(min(noisy_range, max_range))
        else:
            processed_ranges.append(r)
    return processed_ranges
```

## Performance Considerations

For efficient sensor simulation:
- Use appropriate update rates for each sensor type
- Configure realistic but computationally feasible parameters
- Use sensor plugins that match your computational budget
- Consider using sensor fusion algorithms that can handle simulated noise

## Summary

Sensor simulation is critical for developing robust robotic systems. By properly configuring and processing simulated LiDAR, camera, and IMU data, developers can create realistic testing environments that closely match real-world conditions.