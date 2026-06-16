import argparse
from pathlib import Path

import carla


def main():
    parser = argparse.ArgumentParser(description="Export a CARLA town map as OpenDRIVE .xodr")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--output", default="maps/Town01.xodr")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.load_world(args.town)
    xodr = world.get_map().to_opendrive()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xodr, encoding="utf-8")
    print(f"exported {args.town} to {output}")


if __name__ == "__main__":
    main()
