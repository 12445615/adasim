import json
import math
import random
import socket
import time


HOST = "127.0.0.1"
PORT = 8850


def make_cluster(cx, cy, count=55, spread=1.0):
    points = []
    for _ in range(count):
        points.append([
            cx + random.uniform(-spread, spread),
            cy + random.uniform(-spread, spread),
            0.8,
        ])
    return points


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.time()
    print(f"sending fake LiDAR UDP packets to {HOST}:{PORT}")

    while True:
        t = time.time() - start
        points = []
        points += make_cluster(14 + 2 * math.sin(t), 7, 50)
        points += make_cluster(23, -8 + 2 * math.cos(t * 0.7), 55)
        points += make_cluster(32, 2, 50)

        for _ in range(70):
            points.append([
                random.uniform(0, 40),
                random.uniform(-38, 38),
                0.2,
            ])

        packet = {
            "type": "LIDAR_POINTS",
            "points": points,
        }
        sock.sendto(json.dumps(packet).encode("utf-8"), (HOST, PORT))
        time.sleep(0.08)


if __name__ == "__main__":
    main()
