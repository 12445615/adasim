# ADASim Architecture

ADASim is an embedded vehicle HMI and simulation demo for ADAS/autonomous-driving style workflows.

The project focuses on the embedded software chain rather than heavy autonomous-driving algorithms:

- Qt/C++ HMI
- TCP algorithm control input
- UDP LiDAR-like sensor input
- SocketCAN control output
- SQLite runtime logging
- replay and fault-injection support

## Runtime Topology

```text
Python algorithm_node.py
  TCP 8848
  speed / steering JSON
        |
        v
+-----------------------------+
| ADASim Qt/C++ HMI           |
|-----------------------------|
| MainWindow                  |
| SimulationWidget            |
| LidarWidget                 |
| VehicleModel                |
| TcpControlServer            |
| UdpLidarReceiver            |
| CanBusManager               |
| ReplayManager               |
| LogDatabase                 |
+-----------------------------+
        ^
        |
  UDP 8850
  simulated point cloud
Python fake_lidar_sender.py

ADASim
  |
  | SocketCAN frame 0x321
  v
vcan0 / can0
```

## Main Data Flow

```text
TCP control packet
↓
parse speed / steering
↓
VehicleModel bicycle kinematics
↓
update x / y / yaw / mileage
↓
draw vehicle and trajectory
↓
write SQLite state log
↓
send SocketCAN control frame
```

## LiDAR Data Flow

```text
UDP LiDAR packet or internal simulated points
↓
append mouse-injected virtual obstacle points
↓
BFS Euclidean clustering
↓
draw radar view with colored clusters
↓
record point count and cluster count
```

## Fault Injection Flow

```text
right-click on simulation canvas
↓
screen coordinate -> world coordinate
↓
world coordinate -> vehicle-local LiDAR coordinate
↓
generate virtual point cluster
↓
cluster and visualize
↓
if obstacle is in front safety zone, trigger auto brake
↓
record event to SQLite
```

## Key Modules

`VehicleModel`

Implements a simplified bicycle kinematic model:

```text
x += v * cos(yaw) * dt
y += v * sin(yaw) * dt
yaw += v / L * tan(steering) * dt
```

`TcpControlServer`

Receives one-line JSON control packets from the Python algorithm node. The line-buffer protocol avoids TCP sticky-packet and half-packet issues.

`UdpLidarReceiver`

Receives external simulated LiDAR point cloud packets through UDP. If no UDP data is available, ADASim falls back to internal simulated points.

`CanBusManager`

On Linux, sends control frames through SocketCAN. It uses `vcan0` during virtual-machine testing and can be changed to `can0` on RK3588 or another embedded board.

`LogDatabase`

Stores runtime data in SQLite:

- vehicle state
- control packets
- fault-injection events
- auto-brake events

`ReplayManager`

Keeps recent frame snapshots in a sliding window. Replay restores vehicle state, trajectory, point cloud, clusters, virtual obstacles, and logs.
