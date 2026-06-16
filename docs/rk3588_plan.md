# RK3588 Deployment Plan

RK3588 is a good target for ADASim because it has much more CPU/GPU/NPU headroom than i.MX6ULL-class boards.

## Recommended Positioning

ADASim on RK3588 should be positioned as:

```text
Embedded Linux vehicle HMI and control terminal
```

It can demonstrate:

- Qt/C++ HMI deployment
- TCP/UDP network communication
- SocketCAN control output
- SQLite local logging
- sensor visualization
- fault injection and replay
- optional AI/OpenCV extension later

## Suggested Deployment Route

```text
Windows Qt development
↓
x86_64 Ubuntu VM build verification
↓
RK3588 native build
↓
full-screen display
↓
SocketCAN / UDP / serial validation
↓
systemd startup
```

## Native Build On RK3588

Install dependencies:

```bash
sudo apt update
sudo apt install build-essential qtbase5-dev qt5-qmake libqt5sql5-sqlite python3 can-utils
```

Copy source to RK3588:

```bash
scp -r xiangmu2chezai user@rk3588-ip:/home/user/adasim
```

Build:

```bash
cd /home/user/adasim
mkdir -p build-rk3588
cd build-rk3588
qmake ../ADASim.pro
make -j4
./ADASim
```

## Display Options

For debugging:

```bash
./ADASim
```

For full-screen embedded-style operation:

```bash
export QT_QPA_PLATFORM=eglfs
./ADASim
```

If EGLFS is not available, try:

```bash
export QT_QPA_PLATFORM=linuxfb
./ADASim
```

On Ubuntu Desktop images, X11/Wayland is usually easiest during development.

## CAN On RK3588

Example:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
candump can0
```

ADASim currently opens `vcan0` by default. For RK3588 real CAN, change it to `can0` in `MainWindow.cpp`:

```cpp
canBusManager_.open("can0");
```

Later this can be moved into a config file.

## systemd Startup

Install binary:

```bash
sudo mkdir -p /opt/adasim
sudo cp ADASim /opt/adasim/
```

Install service:

```bash
sudo cp deploy/adasim.service /etc/systemd/system/adasim.service
sudo systemctl daemon-reload
sudo systemctl enable adasim.service
sudo systemctl start adasim.service
```

Check logs:

```bash
journalctl -u adasim.service -f
```

## Later Extensions

- make CAN interface configurable
- add serial IMU/GPS receiver
- add OpenCV camera preview
- add RKNN inference placeholder
- add CPU/memory/FPS monitor
- save deployment logs for the resume
