from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path

from .geometry import make_local_transformer
from .osm_parser import parse_elements
from .overpass import load_or_download
from .world_generator import write_world


def default_px4_dir() -> Path:
    env = os.environ.get("PX4_AUTOPILOT_DIR")
    if env:
        return Path(env).expanduser()
    return Path("~/PX4-Autopilot").expanduser()


def main():
    parser = argparse.ArgumentParser(
        description="Generate a Drone Rakshak Gazebo world from any GPS location."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    gen = sub.add_parser("generate", help="Download OSM data and generate a Gazebo world.")
    gen.add_argument("--lat", type=float, required=True, help="Latitude in degrees.")
    gen.add_argument("--lon", type=float, required=True, help="Longitude in degrees.")
    gen.add_argument("--radius", type=float, default=500.0, help="Map radius in metres (default: 500).")
    gen.add_argument("--refresh", action="store_true", help="Ignore cached OSM JSON and download again.")
    gen.add_argument(
        "--output",
        type=Path,
        default=Path("~/drone_ws/generated_maps").expanduser(),
        help="Directory for generated map assets."
    )
    gen.add_argument(
        "--px4-world-dir",
        type=Path,
        default=None,
        help="PX4 Gazebo worlds directory. Default: ~/PX4-Autopilot/Tools/simulation/gz/worlds"
    )
    gen.add_argument("--run-gazebo", action="store_true", help="Launch Gazebo directly with the generated SDF (no PX4).")
    gen.add_argument("--run-px4", action="store_true", help="Generate the map and then start the existing PX4 x500 target.")
    gen.add_argument("--verbose", action="store_true")

    args = parser.parse_args()

    if args.command == "generate":
        lat, lon, radius = args.lat, args.lon, args.radius

        if not (-90.0 <= lat <= 90.0):
            parser.error("--lat must be between -90 and 90.")
        if not (-180.0 <= lon <= 180.0):
            parser.error("--lon must be between -180 and 180.")
        if radius < 50 or radius > 1500:
            parser.error("--radius must be between 50 m and 1500 m for this MVP.")

        px4_world_dir = (
            args.px4_world_dir.expanduser()
            if args.px4_world_dir
            else default_px4_dir() / "Tools/simulation/gz/worlds"
        )
        args.output.mkdir(parents=True, exist_ok=True)
        cache_dir = args.output / "cache"

        print(f"[1/5] Location: {lat:.6f}, {lon:.6f}")
        print(f"[1/5] Radius:   {radius:.1f} m")

        print("[2/5] Downloading / loading OpenStreetMap data...")
        osm = load_or_download(lat, lon, radius, cache_dir, refresh=args.refresh)
        print(f"[2/5] OSM elements: {len(osm.get('elements', []))}")

        print("[3/5] Converting GPS coordinates to local metres...")
        transformer = make_local_transformer(lat, lon)
        features = parse_elements(osm, transformer, radius)

        print("[4/5] Generating Gazebo meshes...")
        world_name, world_file, map_dir = write_world(
            out_dir=args.output,
            world_dir=px4_world_dir,
            lat=lat,
            lon=lon,
            radius_m=radius,
            features=features,
        )

        print("[5/5] Done.")
        print("")
        print("Feature counts:")
        for key, value in features.stats.items():
            print(f"  {key:10s}: {value}")

        print("")
        print(f"World: {world_file}")
        print(f"Assets: {map_dir}")
        print("")
        print("Start Drone Rakshak with:")
        print(f"  cd {default_px4_dir()}")
        print("  source ~/drone_ws/.venv/bin/activate")
        print("  source /opt/ros/jazzy/setup.bash")
        print(f"  source {default_px4_dir()}/build/px4_sitl_default/rootfs/gz_env.sh 2>/dev/null || true")
        print(f"  PX4_GZ_WORLD={world_name} make px4_sitl gz_x500")

        if args.run_gazebo and args.run_px4:
            raise SystemExit("Choose only one of --run-gazebo or --run-px4.")

        if args.run_gazebo:
            print("")
            print("[optional] Launching Gazebo directly (without PX4)...")
            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = str(map_dir) + os.pathsep + env.get("GZ_SIM_RESOURCE_PATH", "")
            subprocess.run(["gz", "sim", "-v", "3", str(world_file)], check=False)

        if args.run_px4:
            print("")
            print("[5/5] Starting existing PX4 x500...")
            env = os.environ.copy()
            env["PX4_GZ_WORLD"] = world_name
            env["PX4_SIM_MODEL"] = "gz_x500"
            subprocess.run(
                ["make", "px4_sitl", "gz_x500"],
                cwd=str(default_px4_dir()),
                env=env,
                check=False,
            )
