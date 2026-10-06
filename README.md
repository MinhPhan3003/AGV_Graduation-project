A Linux-based Autonomous Mobile Robot (AMR) developed using the ROS2 ecosystem on a Jetson Nano for high-level computing, seamlessly integrated with an ESP32 microcontroller over micro-ROS for low-level motor execution. The system implements SLAM fusion via LiDAR and a USB Camera to construct precise environmental maps, while leveraging the Nav2 stack for intelligent autonomous pathfinding and dynamic obstacle avoidance. This project showcases a robust hardware-software co-design tailored for modern industrial logistics and autonomous navigation.
This project is using YOLOv5 tiny for detecting human.

<img width="802" height="606" alt="AGV with object following task" src="https://github.com/user-attachments/assets/ffd22b59-4a65-4514-a5a8-52c74e20e26d" />
<img width="776" height="613" alt="Object followin task" src="https://github.com/user-attachments/assets/7aa80ab7-d68f-470f-974a-413abbf422aa" />
<img width="942" height="711" alt="image" src="https://github.com/user-attachments/assets/62411c89-583c-493e-b500-b1d256775cdc" />




## How the robot works

### Hardware
| Part | Role |
|---|---|
| Jetson Nano | Main computer, runs ROS 2, YOLOv5 and all control nodes |
| ESP32 | Low-level motor control, reads the wheel encoders |
| SICK LiDAR (`192.168.0.3`, Ethernet) | 2D scan in front of the robot (about ±95°), topic `/scan` |
| USB camera (`/dev/video2`, 640×480) | Person detection, field of view about 83° |
| MPU6050 IMU (I2C) | Angular velocity and acceleration, topic `/imu/out` |
| 4 ultrasonic sensors (front, rear, left, right) | Emergency stop, topics `/ultrasonic/*` |
| 2 driving wheels + 4 casters | Differential drive: wheel radius 0.07 m, wheel separation 0.402 m |

### Data flow
1. The camera image goes to YOLOv5n, which finds every person in the frame. SORT gives each person an ID and keeps the same ID from frame to frame.
2. The pixel position of the target person is converted to an angle. The LiDAR beam at that angle gives the distance to the person.
3. The distance and angle are published on `/person/tracking` (`agv_interfaces/PersonTracking`).
4. The controller node turns them into a velocity command (`TwistStamped`) on `agv_controller/cmd_vel`.
5. `diff_drive_controller` (ros2_control) converts the velocity into a speed for each wheel and also publishes wheel odometry on `/agv_controller/odom`.
6. The hardware interface `agv_firmware/AgvInterface` sends the wheel speeds to the ESP32 over serial (LibSerial, 115200 baud) and reads the measured speeds back.
7. The ultrasonic safety node watches the four ultrasonic sensors and publishes `/emergency_stop` when something is closer than 0.30 m.

> Note: the current code talks to the ESP32 over a plain serial port, not over micro-ROS.

### Serial protocol between the Jetson Nano and the ESP32
Each message is one line of text with the speed of each wheel in rad/s:
```
rp05.20,ln03.10,
```
- `r` / `l`: right / left wheel
- `p` / `n`: positive / negative direction
- the number: speed in rad/s, padded to two digits before the decimal point

The ESP32 answers in the same format with the measured wheel speeds. The hardware interface integrates them to get the wheel positions.

## How the code works

### ROS 2 packages
| Package | Contents |
|---|---|
| `agv_description` | URDF/xacro model, STL meshes, RViz and Gazebo launch files |
| `agv_firmware` | ros2_control hardware interface for the ESP32 (`src/agv_interface.cpp`), MPU6050 driver, hardware launch files |
| `agv_controller` | `diff_drive_controller` config, simple/noisy controllers, keyboard and joystick teleop, `twist_mux` |
| `agv_interfaces` | Custom message `PersonTracking` |
| `agv_image_processing` | Person detection, person-following controllers, ultrasonic safety node |
| `agv_bringup` | Launch files that start the whole system |

### Person detection (`range_detection_with_ultrasonic.py`)
The detector works as a state machine:

| State | Meaning | Next state |
|---|---|---|
| `IDLE` | Waiting for a person inside the detection zone (centre of the image) | `DETECTING` when a person enters the zone |
| `DETECTING` | Countdown (`start_delay`, default 4 s) while the person stays in the zone | `FOLLOWING` when the countdown ends, `IDLE` if the person leaves |
| `FOLLOWING` | Target is locked by its SORT ID, distance and angle are published | `STOPPING` when the target is lost |
| `STOPPING` | Countdown (`stop_delay`, default 5 s) waiting for the target to come back | `FOLLOWING` if the target is found, `IDLE` when the countdown ends |

The distance comes from the LiDAR: the camera angle is converted to a LiDAR angle (`lidar_angle = -camera_angle`), and the closest LiDAR beam within ±3° is used.

### Person following: direct controller (`controller_agv_speed_ultrasonic.py`)
This is the original controller, started by `image_processing.launch.py`. It uses two PI controllers:
- angular velocity from the camera angle (turn towards the person)
- linear velocity from the distance error (keep 1.0 m from the person)

It always steers straight at the person, so it cuts corners and stops as soon as the person leaves the camera view.

### Person following: trail controller (`person_path_follower.py`)
This newer controller is started by `image_processing_path_following.launch.py`. Instead of steering straight at the person, the robot follows the path the person walked.

1. **Person position on the floor.** The distance `d` and angle `θ` from the detector give the person position relative to the LiDAR, `(d·cosθ, d·sinθ)`. The LiDAR is 0.366 m in front of the robot centre, so that offset is added. The point is then moved into the `odom` frame with the robot pose `(x, y, yaw)` from wheel odometry. A point in the `odom` frame stays fixed on the floor while the robot moves and turns.
2. **LiDAR refinement.** The LiDAR scan is split into clusters (consecutive points closer than 12 cm). Clusters longer than 0.6 m, such as walls, are ignored. The camera estimate is moved to the centre of the nearby clusters, for example the two legs of the person.
3. **Breadcrumb queue.** Every time the person has moved at least 15 cm, the new position is pushed to the end of a FIFO queue (`collections.deque`, at most 300 points). The queue is the path the person walked.
4. **Pure pursuit.** The path is built from the robot position, the points in the queue and the current person position. The target point is 0.5 m ahead along this path. The robot turns towards the target point, and drives at a speed proportional to the remaining path length minus the 1.0 m following distance. When a person turns a corner, the target point stays on the person's path, so the robot drives to the corner first and only then turns.
5. **Removing old points.** A point is popped from the front of the queue when the robot is closer than 0.25 m to it, or has already passed it.
6. **Person out of the camera view.** For up to 3 s the person is tracked with the LiDAR only (the LiDAR sees about ±95°, the camera about ±41°).
7. **Person lost.** The robot drives to the last place it saw the person, then turns in the direction the person was walking so the camera can find them again.
8. **Safety.** The robot slows down for obstacles closer than 0.8 m in front of it and stops at 0.25 m. It stops when the person is closer than 0.55 m. The ultrasonic emergency stop has the highest priority.

The queue is published on `/person_follower/trail` (`nav_msgs/Path`, frame `odom`) and can be shown in RViz.

Main parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `follow_distance` | 1.0 m | Distance kept from the person, measured along the path |
| `lookahead_distance` | 0.5 m | Smaller follows corners more tightly, larger drives more smoothly |
| `breadcrumb_spacing` | 0.15 m | Distance between two points in the queue |
| `max_linear_vel` | 0.38 m/s | Maximum forward speed |
| `max_angular_vel` | 0.43 rad/s | Maximum turning speed |
| `lidar_only_timeout` | 3.0 s | How long to track with the LiDAR only after the camera loses the person |
| `obstacle_slow_dist` / `obstacle_stop_dist` | 0.8 m / 0.25 m | Obstacle distances for slowing down and stopping |

### Ultrasonic safety (`safety_ultrasonic.py`)
- Publishes `/emergency_stop = true` when any ultrasonic sensor reads less than 0.30 m in two checks in a row.
- Releases the stop when every sensor reads more than 0.35 m.
- Publishes the LED state on `/led_states`: 1 idle, 2 following, 3 stopping, 4 emergency.

## Running

```bash
# Build
colcon build
source install/setup.bash

# LiDAR + hardware interface + diff_drive_controller
ros2 launch agv_bringup following_robot.launch.py

# Person following, choose one:
ros2 launch agv_bringup image_processing.launch.py                  # direct controller
ros2 launch agv_bringup image_processing_path_following.launch.py   # trail controller

# Manual control with a joystick
ros2 launch agv_controller joystick_teleop.launch.py
```

The YOLOv5 code (`yolov5-object-tracking`, with `sort.py`) and the weights `yolov5n.pt` are not in this repository. The detector expects them at `/home/minh/agv_ws/src/agv_image_processing/yolov5-object-tracking` and `/home/minh/yolov5-object-tracking/yolov5n.pt`.
