# gps_nav_robot

Автономный проезд мобильного робота из точки **A (−9, 0)** в точку **B (9, 0)** по GPS-waypoints с объездом препятствий в Gazebo.

| | |
|---|---|
| ROS | **ROS 2 Humble** (Ubuntu 22.04) |
| Симулятор | **Gazebo Classic 11** |
| Язык | C++17 (узлы), Python (launch, утилиты) |
| Навигация | Nav2 + robot_localization |
| Сенсоры | 2D-лидар, depth-камера (эмуляция Orbbec Astra), GPS, IMU |

## Демонстрация

- Видео: `https://disk.yandex.ru/i/6yP7Y4dDWWojpg`
- Rosbag: `https://disk.yandex.ru/d/m68UQoPmby0_Ig`

## Зависимости

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-xacro ros-humble-geographic-msgs
```

## Сборка

```bash
mkdir -p ~/gps_ws/src && cd ~/gps_ws/src
git clone <URL_РЕПОЗИТОРИЯ> gps_nav_robot
cd ~/gps_ws && source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Запуск (одна команда)

```bash
ros2 launch gps_nav_robot sim.launch.py
```

Миссия стартует автоматически. По завершении в логе:

```
MISSION COMPLETE: reached B in XX.X s, 0.YY m from finish centre
```

### Аргументы запуска

| Аргумент | По умолчанию | Назначение |
|---|---|---|
| `gui` | `true` | клиент Gazebo |
| `rviz` | `true` | RViz с готовой конфигурацией |
| `mission` | `true` | автозапуск GPS-миссии |
| `record` | `false` | запись rosbag |
| `world` | `worlds/gps_obstacles.world` | файл мира |

```bash
ros2 launch gps_nav_robot sim.launch.py record:=true
```

## Маршрут

Задаётся в `config/waypoints.yaml` в GPS-координатах.

| Точка | XY мира, м | lat | lon |
|---|---|---|---|
| WP1 | (−3, 3.5) | 55.75127544 | 38.99995222 |
| WP2 | (3, −3.5) | 55.75121256 | 39.00004778 |
| B | (9, 0) | 55.75124400 | 39.00014333 |

Пересчёт своих точек из координат мира в GPS:

```bash
ros2 run gps_nav_robot xy_to_gps.py -- -3,3.5 3,-3.5 9,0
```

## Структура

```
gps_nav_robot/
├── urdf/                       # модель робота и сенсоры (Xacro + Gazebo-плагины)
├── worlds/gps_obstacles.world  # мир: препятствия, зоны A/B, геопривязка
├── config/
│   ├── localization.yaml       # ekf_odom, ekf_map, navsat_transform
│   ├── nav2_params.yaml        # costmaps, planner, controller, behaviors
│   └── waypoints.yaml          # маршрут в GPS-координатах
├── bt/                         # дерево поведения Nav2
├── src/
│   ├── gps_waypoint_follower.cpp
│   └── sensor_monitor.cpp
├── scripts/xy_to_gps.py
├── launch/sim.launch.py
└── rviz/nav.rviz
```

## Топики

| Топик | Тип | Частота |
|---|---|---|
| `/scan` | LaserScan | 10 Гц |
| `/camera/depth/image_raw`, `/camera/points` | Image, PointCloud2 | 15 Гц |
| `/gps/fix` | NavSatFix | 10 Гц |
| `/imu/data` | Imu | 50 Гц |
| `/wheel/odom` | Odometry | 50 Гц |
| `/odometry/local`, `/odometry/global` | Odometry | 30 Гц |
| `/plan` | Path | 1 Гц |
| `/cmd_vel` | Twist | 20 Гц |
| `/diagnostics` | DiagnosticArray | 1 Гц |

## Проверка

```bash
ros2 topic hz /scan /gps/fix /camera/points /imu/data
ros2 topic echo /diagnostics --once
ros2 run tf2_tools view_frames
```
