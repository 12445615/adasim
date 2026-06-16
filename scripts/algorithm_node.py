import json
import math
import socket
import time


HOST = "127.0.0.1"
PORT = 8848
WHEEL_BASE = 1.35
MAX_STEERING = 1.0
MAX_SPEED = 3.2


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


def clamp(value, low, high):
    return max(low, min(high, value))


def project_to_path(position, path):
    best = {
        "segment": 0,
        "point": path[0],
        "distance2": distance2(position, path[0]),
        "t": 0.0,
        "signed_error": 0.0,
        "heading": 0.0,
    }

    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx = bx - ax
        vy = by - ay
        length2 = vx * vx + vy * vy
        if length2 < 1e-6:
            continue

        wx = position[0] - ax
        wy = position[1] - ay
        t = clamp((wx * vx + wy * vy) / length2, 0.0, 1.0)
        px = ax + vx * t
        py = ay + vy * t
        dx = position[0] - px
        dy = position[1] - py
        d2 = dx * dx + dy * dy
        if d2 < best["distance2"]:
            cross = vx * (position[1] - ay) - vy * (position[0] - ax)
            sign = 1.0 if cross > 0.0 else -1.0
            best = {
                "segment": i,
                "point": (px, py),
                "distance2": d2,
                "t": t,
                "signed_error": sign * math.sqrt(d2),
                "heading": math.atan2(vy, vx),
            }

    return best


def choose_lookahead_point(path, projection, lookahead):
    current = projection["point"]
    remaining = lookahead
    start = projection["segment"]

    for i in range(start, len(path) - 1):
        end = path[i + 1]
        seg_len = math.hypot(end[0] - current[0], end[1] - current[1])
        if seg_len >= remaining and seg_len > 1e-6:
            ratio = remaining / seg_len
            return (
                current[0] + (end[0] - current[0]) * ratio,
                current[1] + (end[1] - current[1]) * ratio,
            )
        remaining -= seg_len
        current = end

    return path[-1]


def pure_pursuit_control(telemetry):
    vehicle = telemetry.get("vehicle", {})
    path = telemetry.get("path", [])
    if len(path) < 2:
        return 0.0, 0.0, "waiting_path", 0.0, 0.0

    x = float(vehicle.get("x", 0.0))
    y = float(vehicle.get("y", 0.0))
    yaw = float(vehicle.get("yaw", 0.0))
    position = (x, y)

    goal = path[-1]
    goal_distance = math.sqrt(distance2(position, goal))
    if goal_distance < 2.5:
        return 0.0, 0.0, "arrived", 0.0, 0.0

    projection = project_to_path(position, path)
    lateral_error = projection["signed_error"]
    heading_error = wrap_angle(projection["heading"] - yaw)

    lookahead = clamp(3.5 + abs(float(vehicle.get("speed", 0.0))) * 0.35, 3.5, 7.0)
    target = choose_lookahead_point(path, projection, lookahead)
    target_yaw = math.atan2(target[1] - y, target[0] - x)
    alpha = wrap_angle(target_yaw - yaw)

    pure_pursuit = math.atan2(2.0 * WHEEL_BASE * math.sin(alpha), lookahead)
    stanley = heading_error + math.atan2(0.75 * lateral_error, max(1.0, abs(float(vehicle.get("speed", 0.0)))))
    steering = clamp(0.55 * pure_pursuit + 0.45 * stanley, -MAX_STEERING, MAX_STEERING)

    tracking_error = abs(heading_error) + min(abs(lateral_error) * 0.18, 0.8)
    speed = MAX_SPEED * (1.0 - min(tracking_error, 1.4) / 1.9)
    speed = clamp(speed, 0.8, MAX_SPEED)
    if abs(heading_error) > 1.35 or abs(lateral_error) > 6.0:
        speed = 0.6
        steering = clamp(heading_error + math.atan2(lateral_error, 3.0), -MAX_STEERING, MAX_STEERING)

    return speed, -steering, "tracking", lateral_error, heading_error


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

            speed, steering, status, lateral_error, heading_error = pure_pursuit_control(packet)
            send_control(sock, speed, steering, status)

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                path_size = len(packet.get("path", []))
                print(
                    f"{status}: path={path_size} speed={speed:.2f} steering={steering:.3f} "
                    f"cte={lateral_error:.2f} heading={heading_error:.2f}"
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
