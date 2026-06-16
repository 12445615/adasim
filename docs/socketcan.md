# SocketCAN Validation

ADASim includes a Linux SocketCAN output module.

During virtual-machine development, use `vcan0`. On RK3588 or another embedded board, replace it with the real CAN interface such as `can0`.

## Create vcan0

Run these commands in the Ubuntu virtual machine:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
```

If `vcan0` already exists:

```bash
sudo ip link set up vcan0
```

## Monitor CAN Frames

Install tools:

```bash
sudo apt install can-utils
```

Start monitor:

```bash
candump vcan0
```

## Run ADASim

```bash
cd /home/why/zidongjiashi/build-linux
./ADASim
```

Then start the TCP control node:

```bash
cd /home/why/zidongjiashi
python3 scripts/algorithm_node.py
```

## Expected Frame

ADASim sends CAN frame `0x321` every 50 ms.

Frame layout:

```text
CAN ID: 0x321
DLC: 8
data[0..1]: speed in cm/s, signed int16, big endian
data[2..3]: steering in mrad, signed int16, big endian
data[4]: braking flag, 1 means auto brake active
data[5..7]: reserved
```

Example:

```text
vcan0  321   [8]  02 BC 00 78 00 00 00 00
```

When auto brake is triggered:

```text
vcan0  321   [8]  00 00 00 78 01 00 00 00
```

## Real Board Notes

On RK3588, use a real CAN interface:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
candump can0
```

Then change the code startup interface from `vcan0` to `can0`, or make it configurable later.
