# Linux Build And Run

This document describes how to build and run ADASim on an x86_64 Ubuntu virtual machine.

## Install Dependencies

```bash
sudo apt update
sudo apt install build-essential qtbase5-dev qt5-qmake libqt5sql5-sqlite python3
```

For SocketCAN testing:

```bash
sudo apt install can-utils
```

## Build

```bash
cd /home/why/zidongjiashi
mkdir -p build-linux
cd build-linux
qmake ../ADASim.pro
make -j4
```

The output binary is:

```bash
/home/why/zidongjiashi/build-linux/ADASim
```

## Run ADASim

Open a terminal in the Ubuntu desktop:

```bash
cd /home/why/zidongjiashi/build-linux
./ADASim
```

Click `启动` in the UI.

## Run TCP Algorithm Node

Open another terminal:

```bash
cd /home/why/zidongjiashi
python3 scripts/algorithm_node.py
```

This node sends control packets to ADASim:

```json
{"type":"CONTROL_KINEMATIC","speed":7.0,"steering":0.12}
```

## Run UDP Fake LiDAR Node

Open another terminal:

```bash
cd /home/why/zidongjiashi
python3 scripts/fake_lidar_sender.py
```

This node sends point cloud packets to ADASim over UDP port `8850`.

## SQLite Output

After running ADASim, the database file is generated in the executable directory:

```bash
/home/why/zidongjiashi/build-linux/adasim_run.db
```

Useful queries:

```bash
sqlite3 /home/why/zidongjiashi/build-linux/adasim_run.db ".tables"
sqlite3 /home/why/zidongjiashi/build-linux/adasim_run.db "select * from event_log order by id desc limit 10;"
sqlite3 /home/why/zidongjiashi/build-linux/adasim_run.db "select id,x,y,yaw,speed,steering,mileage,point_count,cluster_count,braking from vehicle_state order by id desc limit 10;"
```
