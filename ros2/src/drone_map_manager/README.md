# Drone Rakshak Map Manager

This package is the first implementation stage of the "choose any GPS location -> generate Gazebo environment" workflow.

It downloads OpenStreetMap geometry around a GPS coordinate and generates:
- 3D building meshes with collision
- road meshes with collision
- water surfaces
- green/park/forest surfaces
- a Gazebo SDF world
- `spherical_coordinates` anchored to the requested GPS location

It does not modify PX4 GZBridge, the X500 model, LiDAR, camera, radar, fusion, avoidance, or offboard controller.

## 1. Install dependencies

```bash
source ~/drone_ws/.venv/bin/activate

python -m pip install --upgrade pip
python -m pip install requests shapely pyproj
```

## 2. Copy this package into the ROS 2 workspace

```bash
cp -r drone_map_manager ~/drone_ws/src/
```

Or, after extracting this package into your Downloads directory:

```bash
cp -r ~/Downloads/drone_map_manager ~/drone_ws/src/
```

## 3. Build

```bash
cd ~/drone_ws
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/.venv/bin/activate

colcon build --packages-select drone_map_manager
source ~/drone_ws/install/setup.bash
```

## 4. Generate a map

Example:

```bash
ros2 run drone_map_manager generate   --lat 28.6139   --lon 77.2090   --radius 500
```

The program will generate a world under:

```text
~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

and assets under:

```text
~/drone_ws/generated_maps/
```

## 5. Start your existing PX4 X500 simulation

Use the world name printed by the generator:

```bash
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash

PX4_GZ_WORLD=<printed_world_name> make px4_sitl gz_x500
```

Or let the generator start the existing PX4 target automatically:

```bash
ros2 run drone_map_manager generate   --lat 28.6139   --lon 77.2090   --radius 500   --run-px4
```

Do NOT change the base X500 model. PX4 still spawns the existing `x500`; the generated SDF is the environment only.

## 6. Try another location

Only change the coordinates:

```bash
ros2 run drone_map_manager generate   --lat 19.0760   --lon 72.8777   --radius 500
```

or:

```bash
ros2 run drone_map_manager generate   --lat 40.7128   --lon -74.0060   --radius 500
```

## Important

- Public Overpass instances are intended for reasonable/small query loads. This MVP limits the generated radius to 1.5 km.
- OSM geometry is geographic/vector data; this first version generates flat geometry. Real elevation, satellite imagery, detailed 3D roofs, trees and additional tagged objects are the next stage.
- Keep the attribution in project documentation: `Map data © OpenStreetMap contributors` and ODbL.
