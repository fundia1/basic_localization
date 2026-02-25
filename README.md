# Localization

ERP42 실외 주행 로봇의 로컬/글로벌 오도메트리 패키지.  
Wheel encoder + IMU + GPS를 Dual EKF로 융합하여 `map → odom → base_link` TF 체인을 발행합니다.

## 차량 (ERP42 old)

| 항목 | 값 |
|------|-----|
| 구동 | 후륜 구동, 전륜 조향 |
| 크기 (L×W×H) | 1570 × 1180 × 570 mm |
| Wheelbase | 1040 mm |
| Tread | 985 mm |
| 휠 | 13 inch (175/60R13) |
| 최고 속도 | 40 kph (제한 20 kph) |

## 센서 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/erp42_feedback` | `erp42_msgs/SerialFeedBack` | 속도, 조향각, 기어 (타임스탬프 없음) |
| `/imu/data` | `sensor_msgs/Imu` | 9-DOF AHRS IMU (방향, 각속도, 가속도) |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyne 32C 3D LiDAR |
| `/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | RTK GPS |

## 노드 구성

```
                              ┌─────────────────────────────────────────────┐
/erp42_feedback ──┐           │                                             │
                  ├─► wheel_odometry ──► /odometry/wheel ──┐               │
/imu/data ────────┘                                         ├─► local_ekf_node ──► /odometry/local
                  (sync_imu:=true 시)                       │    (odom → base_link)
/imu/data ──► imu_time_sync ──► /imu/data/synced ─────────┘
                                        │
                                        ├───────────────────┐
/ublox_gps_node/fix ──► gps_odometry ──► /odometry/gps      ├─► global_ekf_node ──► /odometry/global
                        (map_anchor.yaml 기준)               │    (map → odom)
                        /odometry/wheel ────────────────────┘
```

| 노드 | 역할 |
|------|------|
| `wheel_odometry` | 바퀴 오도메트리 (bicycle kinematic model) + IMU 초기 방향 설정 |
| `gps_odometry` | GPS → UTM 변환, datum 기준 로컬 좌표 발행 |
| `local_ekf_node` | EKF 센서 융합 — odom → base_link TF |
| `global_ekf_node` | EKF 센서 융합 — map → odom TF |
| `imu_time_sync` | IMU 타임스탬프를 wall clock으로 재발행 (선택) |

## 실행

```bash
# 빌드
colcon build --packages-select localization --symlink-install

# Dual EKF + GPS (map → odom → base_link)
ros2 launch localization dual_ekf_localization.launch.py

# Local only (odom → base_link)
ros2 launch localization local_localization.launch.py

# bag 재생 (IMU 타임스탬프 동기화)
ros2 launch localization dual_ekf_localization.launch.py sync_imu:=true
```

## TF 체인

```
map ──(global_ekf_node)──► odom ──(local_ekf_node)──► base_link
                                                        ├── imu_link    (앞 1m, 위 0.5m)
                                                        ├── gps         (앞 0.5m, 위 1.5m)
                                                        ├── velodyne    (앞 1m, 위 1m)
                                                        └── encoder_link (앞 1m, 오른쪽 0.5m)
```

## EKF 설정

| 파일 | 센서 | 사용 상태 |
|------|------|-----------|
| `ekf_local.yaml` | `odom0` (`/odometry/wheel`) | vx, vy=0 (no-slip) |
| | `imu0` (`/imu/data`) | yaw, vyaw |
| `ekf_global.yaml` | `odom0` (`/odometry/gps`) | x, y (GPS 위치) |
| | `odom1` (`/odometry/wheel`) | vx, vy=0 |
| | `imu0` (`/imu/data`) | yaw, vyaw |

## 패키지 구조

```
src/localization/
├── config/
│   ├── ekf_local.yaml
│   ├── ekf_global.yaml
│   └── map_anchor.yaml
├── launch/
│   ├── local_localization.launch.py
│   └── dual_ekf_localization.launch.py
├── localization/
│   ├── __init__.py
│   ├── wheel_odometry.py
│   ├── gps_odometry.py
│   └── imu_time_sync.py
├── package.xml
├── setup.py
└── setup.cfg
```
