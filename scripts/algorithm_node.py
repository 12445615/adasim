import json
import math
import socket
import time


HOST = "127.0.0.1"
PORT = 8848


def main():
    while True:
        try:
            with socket.create_connection((HOST, PORT), timeout=3) as sock:
                print(f"connected to ADASim at {HOST}:{PORT}")
                start = time.time()
                while True:
                    t = time.time() - start
                    packet = {
                        "type": "CONTROL_KINEMATIC",
                        "speed": 7.0 + 1.5 * math.sin(t * 0.35),
                        "steering": 0.18 * math.sin(t * 0.8),
                    }
                    sock.sendall((json.dumps(packet) + "\n").encode("utf-8"))
                    time.sleep(0.05)
        except OSError as exc:
            print(f"waiting for ADASim: {exc}")
            time.sleep(1.0)


if __name__ == "__main__":
    main()
