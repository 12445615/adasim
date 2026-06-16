import json
import math
import socket
import time


HOST = "127.0.0.1"
PORT = 8848


def clamp(value, low, high):
    return max(low, min(high, value))


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


def nearest_path_segment(position, path, last_index):
    segment_count = len(path) - 1
    search_start = clamp(last_index - 1, 0, max(0, segment_count - 1))
    search_end = min(segment_count - 1, search_start + 8)

    nearest_segment = search_start
    nearest_t = 0.0
    nearest_distance2 = float("inf")

    for i in range(search_start, search_end + 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        abx = bx - ax
        aby = by - ay
        length2 = abx * abx + aby * aby
        if length2 < 1e-6:
            continue

        apx = position[0] - ax
        apy = position[1] - ay
        t = clamp((apx * abx + apy * aby) / length2, 0.0, 1.0)
        px = ax + abx * t
        py = ay + aby * t
        d2 = (px - position[0]) * (px - position[0]) + (py - position[1]) * (py - position[1])
        if d2 < nearest_distance2:
            nearest_distance2 = d2
            nearest_segment = i
            nearest_t = t

    return nearest_segment, nearest_t, nearest_distance2


def lookahead_point(path, nearest_segment, nearest_t, distance):
    current = (
        path[nearest_segment][0] + (path[nearest_segment + 1][0] - path[nearest_segment][0]) * nearest_t,
        path[nearest_segment][1] + (path[nearest_segment + 1][1] - path[nearest_segment][1]) * nearest_t,
    )

    remaining = distance
    for i in range(nearest_segment, len(path) - 1):
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


class InternalStyleController:
    def __init__(self):
        self.route_path_index = 1

    def reset_if_path_changed(self, telemetry):
        active_goal_index = telemetry.get("active_goal_index", 0)
        path_size = len(telemetry.get("path", []))
        key = (active_goal_index, path_size)
        if getattr(self, "_path_key", None) != key:
            self._path_key = key
            self.route_path_index = 1

    def control(self, telemetry):
        self.reset_if_path_changed(telemetry)

        vehicle = telemetry.get("vehicle", {})
        path = telemetry.get("path", [])
        if len(path) < 2:
            return 0.0, 0.0, "waiting_path", 0.0

        x = float(vehicle.get("x", 0.0))
        y = float(vehicle.get("y", 0.0))
        yaw = float(vehicle.get("yaw", 0.0))
        position = (x, y)

        nearest_segment, nearest_t, nearest_d2 = nearest_path_segment(position, path, self.route_path_index)
        self.route_path_index = max(self.route_path_index, nearest_segment + 1)

        target = lookahead_point(path, nearest_segment, nearest_t, 6.0)
        target_yaw = math.atan2(target[1] - y, target[0] - x)
        yaw_error = wrap_angle(target_yaw - yaw)

        if abs(yaw_error) > 1.15:
            steering = 1.25 if yaw_error > 0.0 else -1.25
            return 0.0, steering, "aligning", math.sqrt(nearest_d2)

        steering = clamp(yaw_error * 1.65, -1.15, 1.15)
        speed = max(1.8, 4.5 * (1.0 - min(abs(yaw_error), 1.2) / 1.8))

        goal = path[-1]
        if distance2(goal, position) < 9.0:
            return 0.0, steering, "arrived", math.sqrt(nearest_d2)

        return speed, steering, "tracking", math.sqrt(nearest_d2)


def send_control(sock, speed, steering, status):
    packet = {
        "type": "CONTROL_KINEMATIC",
        "algorithm": "internal_style_python",
        "status": status,
        "speed": speed,
        "steering": steering,
    }
    sock.sendall((json.dumps(packet) + "\n").encode("utf-8"))


def run_client():
    controller = InternalStyleController()
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

            speed, steering, status, path_error = controller.control(packet)
            send_control(sock, speed, steering, status)

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(
                    f"{status}: goal={packet.get('active_goal_index', 0) + 1} "
                    f"path={len(packet.get('path', []))} speed={speed:.2f} "
                    f"steering={steering:.3f} path_error={path_error:.2f}"
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
