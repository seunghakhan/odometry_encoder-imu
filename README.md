# 🦾 ROS2 Odometry System — Encoder + IMU Fusion

> Real-time odometry estimation using **encoder** and **IMU** data  
> Developed by **Han Seunghak (한승학)**  

---

## 🚀 Overview

This ROS2 package (`odo`) estimates the robot's odometry by fusing **encoder** and **IMU** data.  
It also includes a simple **Twist command publisher** for testing robot movement.

---

## 📁 Package Structure

```markdown
odo/
├── src/
│   ├── sensor_node.cpp           # Encoder & IMU preprocessing
│   ├── odo_node.cpp              # Odometry computation & TF broadcasting
│   └── twist_input_publisher.cpp # PID-based twist command publisher
├── config/
│   └── c_params.yaml             # Parameter settings for odometry node
├── launch/
│   └── odo.launch.py             # Launches all three nodes
├── CMakeLists.txt
└── package.xml

---

## 🧩 Node Description & Parameters

### 🟢 1. `sensor_node` — SensorPostProcess
**Role:**  
Reads raw encoder and IMU data → publishes calibrated and aligned signals.  
Used as the preprocessing stage before odometry fusion.

**Topics**
- **Subscribe:**  
  `/encoder_count` (`std_msgs/Int32`)  
  `/imu/data_raw` (`sensor_msgs/Imu`)
- **Publish:**  
  `/encoder_count_fixed` (`std_msgs/Int32`)  
  `/imu/data_fixed` (`sensor_msgs/Imu`)

**Core Logic**
- Stores first encoder reading as `enc_offset_` to define odometry origin.  
- Publishes `(encoder_raw - enc_offset_)` as fixed encoder count.  
- Buffers IMU readings for synchronization with encoder updates.  
- Ensures both encoder & IMU messages are valid before publishing.

---

### 🔵 2. `odo_node` — EncoderIMUOdometry
**Role:**  
Fuses encoder displacement and IMU orientation to estimate the robot’s odometry (`/odom`)  
and broadcasts the TF between `odom` → `base_link`.

**Topics**
- **Subscribe:**  
  `/encoder_count_fixed` (`std_msgs/Int32`)  
  `/imu/data_fixed` (`sensor_msgs/Imu`)
- **Publish:**  
  `/odom` (`nav_msgs/Odometry`)  
  TF: `odom → base_link`

**Key Parameters**
| Parameter | Description | Default |
|------------|--------------|----------|
| `odom_frame_id` | Odometry frame name | `"odom"` |
| `base_frame_id` | Base link frame name | `"base_link"` |
| `rate_hz` | Publish frequency | `10.0` |
| `encoder_min` | Encoder minimum value | `-32768` |
| `encoder_max` | Encoder maximum value | `32768` |

**Core Logic**
- Converts encoder delta to linear displacement (m).  
- Integrates IMU yaw angle to update orientation (θ).  
- Computes `(x, y, θ)` using differential drive kinematics.  
- Publishes `/odom` and broadcasts TF transform.  

---

### 🟣 3. `twist_input_publisher` — TestCommandPublisher
**Role:**  
Publishes `/cmd_vel` commands for simulation or PID testing.  
Generates smooth velocity control signals toward a target pose.

**Topics**
- **Publish:**  
  `/cmd_vel` (`geometry_msgs/Twist`)

**Key Parameters**
| Parameter | Description | Default |
|------------|--------------|----------|
| `goal_x`, `goal_y` | Target position (m) | `5.0`, `2.0` |
| `slowdown_dist` | Distance to start deceleration | `0.5` |
| `steering_slow_threshold` | Angle threshold to reduce steering | `20.0` |
| `Kp`, `Ki`, `Kd` | PID control gains | `5.0`, `2.0`, `0.5` |

**Core Logic**
- Computes error between current position and goal.  
- Calculates linear & angular velocity using PID controller.  
- Slows down near goal and publishes stable `/cmd_vel`.

---



