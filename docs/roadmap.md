# ADASim Implementation Roadmap

## Phase 1: PC Demo

- Qt 主窗口
- 仿真画布
- 车辆运动学模型
- 轨迹绘制
- LiDAR 模拟点云
- BFS 聚类
- TCP 控制协议
- Python 算法节点
- 回放滑条

## Phase 2: Embedded Linux

- 安装 ARM 板卡 Qt 运行环境
- 交叉编译 Qt 程序
- 配置 EGLFS 或 Wayland
- 全屏运行 ADASim
- systemd 开机自启动
- 记录 CPU、内存、FPS

## Phase 3: Vehicle I/O

- UDP 接收模拟传感器数据
- SocketCAN 下发速度和转角控制指令
- 串口读取 IMU/GPS 模拟数据
- SQLite 保存运行日志和关键帧

## Resume Points

- Qt/C++ 可视化上位机
- Embedded Linux 部署
- TCP/UDP/SocketCAN 通信
- 多线程数据流架构
- 点云聚类与渲染优化
- 车辆运动学闭环
- 滑动窗口回放
