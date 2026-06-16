# ADASim

ADASim 是一个面向嵌入式 Linux 方向的自动驾驶可视化上位机 MVP。

它的目标不是一开始做复杂算法，而是先做出一个能演示、能讲清楚、后续能部署到 ARM 板卡的实时 HMI 系统：

- Qt/C++ 主界面
- 自行车运动学模型
- Python 算法节点 TCP 控制
- LiDAR 点云模拟
- BFS 欧式聚类
- 轨迹绘制
- 滑动窗口回放
- 读写锁保护的数据中心

## 目录结构

```text
ADASim.pro
include/
  DataManager.h
  LidarWidget.h
  MainWindow.h
  ReplayManager.h
  SimulationWidget.h
  TcpControlServer.h
  VehicleModel.h
src/
  DataManager.cpp
  LidarWidget.cpp
  MainWindow.cpp
  ReplayManager.cpp
  SimulationWidget.cpp
  TcpControlServer.cpp
  VehicleModel.cpp
  main.cpp
scripts/
  algorithm_node.py
docs/
  roadmap.md
deploy/
  adasim.service
```

## 运行方式

1. 用 Qt Creator 打开 `ADASim.pro`
2. 选择 Desktop Qt Kit
3. 编译并运行
4. 点击界面上的“启动”
5. 启动 Python 算法节点

```bash
python scripts/algorithm_node.py
```

Python 节点会通过 TCP 发送控制报文：

```json
{"type":"CONTROL_KINEMATIC","speed":7.5,"steering":0.12}
```

Qt 端使用一行一个 JSON 的协议，通过 `\n` 分帧，解决 TCP 粘包和半包问题。

## UDP 模拟雷达

启动模拟 LiDAR 发送器：

```bash
python scripts/fake_lidar_sender.py
```

该脚本会向 `127.0.0.1:8850` 发送 UDP 点云包：

```json
{"type":"LIDAR_POINTS","points":[[12.0,3.5,0.8],[12.4,3.2,0.8]]}
```

Qt 端接收后会执行点云显示与 BFS 聚类。没有外部 UDP 数据时，程序会自动回退到内部模拟点云。

## SQLite 运行记录

程序运行后会在可执行文件同级目录生成 `adasim_run.db`，用于保存：

- `vehicle_state`：车辆位姿、速度、转角、里程、点云数量、聚类数量、制动状态
- `control_packet`：Python 算法节点发送的控制量
- `event_log`：虚拟障碍物、自动刹停、系统事件

## 配置文件

程序启动时会尝试读取可执行文件同级目录下的：

```text
config/adasim.json
```

示例：

```json
{
  "tcp_port": 8848,
  "udp_lidar_port": 8850,
  "can_interface": "vcan0",
  "opendrive_map": "maps/demo.xodr",
  "auto_brake_distance": 15.0,
  "auto_brake_width": 4.0,
  "state_log_interval_ms": 200,
  "can_tx_interval_ms": 50
}
```

虚拟机测试用：

```json
"can_interface": "vcan0"
```

RK3588 真实 CAN 用：

```json
"can_interface": "can0"
```

切换 OpenDRIVE 地图：

```json
"opendrive_map": "maps/Town01.xodr"
```

如果使用 CARLA，可以在 CARLA server 启动后导出地图：

```bash
python scripts/export_carla_xodr.py --town Town01 --output maps/Town01.xodr
```

If CARLA is installed from the official Windows zip package, the OpenDRIVE maps
can also be copied directly from:

```text
E:\Linux\CARLA\CARLA_0.9.16\CarlaUE4\Content\Carla\Maps\OpenDrive
```

## 下一步增强

- 迁移到 ARM Linux 板卡
- 配置 EGLFS 全屏运行
- 配置 systemd 开机自启动

## Documentation

- `docs/architecture.md`：系统架构与数据流
- `docs/linux_deploy.md`：Ubuntu 虚拟机编译运行
- `docs/socketcan.md`：vcan0 / SocketCAN 验证
- `docs/rk3588_plan.md`：RK3588 部署路线
- `docs/resume.md`：简历写法与面试讲法
