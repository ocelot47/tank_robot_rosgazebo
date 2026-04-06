# four_wheeled_robot

Mô phỏng xe 4 bánh trên ROS 2 Humble + Gazebo Classic.

Phiên bản hiện tại có:
- Mô phỏng Gazebo với world tùy chỉnh.
- Điều khiển tay WASD (`/cmd_vel`).
- Lidar xuất `LaserScan` ở topic `/scan`.
- SLAM Toolbox để quét và lưu map.
- Nav2 + RViz để chọn goal và tự chạy né vật cản.
- Global planner custom: chọn `astar` hoặc `dijkstra`.

## 1) Yêu cầu hệ thống

- Ubuntu 22.04
- ROS 2 Humble

## 2) Cài dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-tf2-geometry-msgs
```

Nếu máy chưa init rosdep:

```bash
sudo rosdep init
rosdep update
```

## 3) Lấy code và build

### 3.1. Tạo workspace

```bash
mkdir -p ~/tank_ws/src
cd ~/tank_ws/src
```

### 3.2. Clone repo của bạn

```bash
git clone https://github.com/ocelot47/tank_robot_rosgazebo
```

### 3.3. Cài dependency theo package + build

```bash
cd ~/tank_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 3.4. Source môi trường (mỗi terminal mới)

```bash
source /opt/ros/humble/setup.bash
source ~/tank_ws/install/setup.bash
```

## 4) Chạy mô phỏng cơ bản

```bash
ros2 launch four_wheeled_robot launch_sim.launch.py
```

`launch_sim.launch.py` chỉ chạy Gazebo + spawn robot, không mở RViz.

### Các tham số hay dùng

```bash
# đổi world
ros2 launch four_wheeled_robot launch_sim.launch.py \
  world:=/home/$USER/tank_ws/src/four_wheeled_robot/worlds/training_map.world

# chạy headless
ros2 launch four_wheeled_robot launch_sim.launch.py gui:=false

# đổi vị trí spawn
ros2 launch four_wheeled_robot launch_sim.launch.py \
  spawn_x:=1.0 spawn_y:=-0.5 spawn_yaw:=1.57
```

## 5) Điều khiển tay WASD

Mở terminal mới và chạy:

```bash
ros2 run four_wheeled_robot wasd_teleop
```

Phím điều khiển:
- `w`: tiến
- `s`: lùi
- `a`: quay trái
- `d`: quay phải
- `x` hoặc `space`: dừng
- `q/z`: tăng/giảm tốc độ tiến
- `e/c`: tăng/giảm tốc độ quay

Default hiện tại của teleop:
- `linear_speed = 1.3`
- `angular_speed = 3.0`

## 6) SLAM: quét map và lưu map

### 6.1. Chạy SLAM stack

```bash
ros2 launch four_wheeled_robot slam_mapping.launch.py
```

Launch này chạy:
- Gazebo
- robot_state_publisher
- SLAM Toolbox online async
- RViz

### 6.2. Lái xe đi quét map

```bash
ros2 run four_wheeled_robot wasd_teleop
```

### 6.3. Lưu map sau khi quét xong

```bash
mkdir -p ~/tank_ws/src/four_wheeled_robot/maps
ros2 run nav2_map_server map_saver_cli -f ~/tank_ws/src/four_wheeled_robot/maps/my_map
```

Kết quả:
- `~/tank_ws/src/four_wheeled_robot/maps/my_map.yaml`
- `~/tank_ws/src/four_wheeled_robot/maps/my_map.pgm`

## 7) Nav2 + RViz: chọn điểm để xe tự chạy

```bash
ros2 launch four_wheeled_robot nav2_navigation.launch.py \
  map:=/home/$USER/tank_ws/src/four_wheeled_robot/maps/my_map.yaml
```

Launch này chạy:
- Gazebo + spawn robot
- AMCL localization
- Nav2 stack (controller, behavior, bt_navigator, waypoint_follower, velocity_smoother)
- Custom planner server
- RViz

### 7.1. Chọn thuật toán global planner

```bash
# A*
ros2 launch four_wheeled_robot nav2_navigation.launch.py \
  map:=/home/$USER/tank_ws/src/four_wheeled_robot/maps/my_map.yaml \
  planner_algorithm:=astar

# Dijkstra
ros2 launch four_wheeled_robot nav2_navigation.launch.py \
  map:=/home/$USER/tank_ws/src/four_wheeled_robot/maps/my_map.yaml \
  planner_algorithm:=dijkstra
```

Default hiện tại: `astar` (xem `launch/nav2_navigation.launch.py`).

### 7.2. Initial pose

Mặc định launch sẽ auto publish initial pose theo vị trí spawn.

Nếu muốn tự đặt bằng tay trong RViz:

```bash
ros2 launch four_wheeled_robot nav2_navigation.launch.py \
  map:=/home/$USER/tank_ws/src/four_wheeled_robot/maps/my_map.yaml \
  auto_initial_pose:=false
```

### 7.3. Quy trình thao tác trong RViz

1. Đợi log có `Managed nodes are active`.
2. Nếu cần, bấm `2D Pose Estimate` để đặt lại pose.
3. Bấm `Nav2 Goal` rồi click/drag để đặt goal.
4. Robot sẽ tính global path trước, rồi local planner bám path và né vật cản bằng lidar.

Lưu ý:
- Không chạy `wasd_teleop` cùng lúc với goal Nav2 (tránh tranh chấp `cmd_vel`).

## 8) Thông số điều hướng mặc định hiện tại (đã tune)

File: `config/nav2_params.yaml`

- Tốc độ tiến tối đa Nav2: `1.3 m/s`
- Tốc độ quay tối đa Nav2: `2.8 rad/s`
- Cho phép lùi: có (`min_vel_x < 0`)
- Goal tolerance:
  - `xy_goal_tolerance: 0.22`
  - `yaw_goal_tolerance: 0.45`

Nếu muốn chỉnh lại độ "nhạy" khi tới goal:
- `controller_server.ros__parameters.general_goal_checker.*`
- `controller_server.ros__parameters.FollowPath.RotateToGoal.*`

## 9) Tạo/chỉnh world trong Gazebo

1. Chạy `launch_sim.launch.py`.
2. Chỉnh object trong scene.
3. `File -> Save World As...`
4. Lưu vào:

```text
~/tank_ws/src/four_wheeled_robot/worlds/<ten_map>.world
```

5. Chạy lại:

```bash
ros2 launch four_wheeled_robot launch_sim.launch.py \
  world:=/home/$USER/tank_ws/src/four_wheeled_robot/worlds/<ten_map>.world
```

Khuyến nghị: không lưu sẵn robot `my_bot` trong world để tránh trùng entity khi spawn.

## 10) Các launch file chính

- `launch/launch_sim.launch.py`: Gazebo + spawn robot
- `launch/slam_mapping.launch.py`: Gazebo + SLAM + RViz
- `launch/nav2_navigation.launch.py`: Gazebo + AMCL + Nav2 + RViz
- `launch/navigation_custom.launch.py`: bringup Nav2 với custom planner

## 11) Troubleshooting nhanh

### 11.1. `Package 'four_wheeled_robot' not found`

Chưa source workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/tank_ws/install/setup.bash
```

### 11.2. `No module named 'xacro'`

Cài thiếu:

```bash
sudo apt install -y ros-humble-xacro
```

### 11.3. `gzserver ... exit code 255`

Thường do phiên Gazebo cũ còn chạy:

```bash
pkill gzserver || true
pkill gzclient || true
```

rồi launch lại.

### 11.4. Không thấy scan

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

### 11.5. Nav2 không nhận goal / frame lỗi

- Kiểm tra map path có đúng không.
- Nếu tắt auto initial pose (`auto_initial_pose:=false`), phải bấm `2D Pose Estimate` trước khi gửi goal.
- Kiểm tra TF:

```bash
ros2 topic echo /tf --once
```

## 12) Build nhanh sau khi sửa code

```bash
cd ~/tank_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select four_wheeled_robot --symlink-install
source install/setup.bash
```

