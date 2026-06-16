import json
import math
import socket
import time


HOST = "127.0.0.1"
PORT = 8848
WHEEL_BASE = 1.35


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def choose_lookahead_point(position, path, lookahead):
    if not path:
        return None

    nearest_index = min(range(len(path)), key=lambda i: distance2(position, path[i]))
    target = path[-1]
    for point in path[nearest_index:]:
        if distance2(position, point) >= lookahead * lookahead:
            target = point
            break
    return target


def pure_pursuit_control(telemetry):
    vehicle = telemetry.get("vehicle", {})
    path = telemetry.get("path", [])
    if len(path) < 2:
        return 0.0, 0.0, "waiting_path"

    x = float(vehicle.get("x", 0.0))
    y = float(vehicle.get("y", 0.0))
    yaw = float(vehicle.get("yaw", 0.0))
    position = (x, y)

    goal = path[-1]
    goal_distance = math.sqrt(distance2(position, goal))
    if goal_distance < 2.0:
        return 0.0, 0.0, "arrived"

    lookahead = max(4.0, min(8.0, goal_distance * 0.35))
    target = choose_lookahead_point(position, path, lookahead)
    if target is None:
        return 0.0, 0.0, "waiting_path"

    target_yaw = math.atan2(target[1] - y, target[0] - x)
    alpha = wrap_angle(target_yaw - yaw)
    steering = math.atan2(2.0 * WHEEL_BASE * math.sin(alpha), lookahead)
    steering = max(-1.15, min(1.15, steering))

    speed = 4.0 * (1.0 - min(abs(alpha), 1.2) / 1.8)
    speed = max(1.2, min(4.5, speed))
    if abs(alpha) > 1.25:
        speed = 0.8

    return speed, steering, "tracking"


def send_control(sock, speed, steering, status):
    packet = {
        "type": "CONTROL_KINEMATIC",
        "algorithm": "pure_pursuit",
        "status": status,
        "speed": speed,
        "steering": steering,
    }
    sock.sendall((json.dumps(packet) + "\n").encode("utf-8"))


def run_client():
    with socket.create_connection((HOST, PORT), timeout=3) as sock:
        print(f"connected to ADASim at {HOST}:{PORT}")
        file = sock.makefile("r", encoding="utf-8")
        last_print = 0.0
        for line in file:
            line = line.strip()
            if not line:
                continue
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            if packet.get("type") != "TELEMETRY":
                continue

            speed, steering, status = pure_pursuit_control(packet)
            send_control(sock, speed, steering, status)

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                path_size = len(packet.get("path", []))
                print(
                    f"{status}: path={path_size} speed={speed:.2f} steering={steering:.3f}"
                )


def main():
    while True:
        try:
            run_client()
        except OSError as exc:
            print(f"waiting for ADASim telemetry: {exc}")
            time.sleep(1.0)


if __name__ == "__main__":
    main()
