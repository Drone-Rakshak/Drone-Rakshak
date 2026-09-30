from __future__ import annotations

import json
import math
import re
from pathlib import Path

from shapely.ops import unary_union

from .mesh import MeshBuilder
from .osm_parser import FeatureSet


def safe_name(lat: float, lon: float) -> str:
    s = f"drone_rakshak_map_{lat:.5f}_{lon:.5f}"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", s)


def rgba(r, g, b, a=1.0):
    return f"{r:.3f} {g:.3f} {b:.3f} {a:.3f}"


def write_world(
    out_dir: Path,
    world_dir: Path,
    lat: float,
    lon: float,
    radius_m: float,
    features: FeatureSet,
    elevation_m: float = 0.0,
):
    world_name = safe_name(lat, lon)
    map_dir = out_dir / world_name
    map_dir.mkdir(parents=True, exist_ok=True)

    buildings = MeshBuilder()
    roads = MeshBuilder()
    water = MeshBuilder()
    green = MeshBuilder()

    for poly, height in features.buildings:
        buildings.add_extruded_polygon(poly, 0.0, height)

    for line, width in features.roads:
        buffered = line.buffer(width / 2.0, cap_style=2, join_style=2)
        if buffered.geom_type == "Polygon":
            roads.add_flat_polygon(buffered, 0.03, thickness=0.03)
        else:
            for p in buffered.geoms:
                roads.add_flat_polygon(p, 0.03, thickness=0.03)

    for poly in features.water:
        water.add_flat_polygon(poly, 0.02, thickness=0.01)

    for poly in [*features.parks, *features.wood]:
        green.add_flat_polygon(poly, 0.012, thickness=0.01)

    building_mesh = map_dir / "buildings.stl"
    road_mesh = map_dir / "roads.stl"
    water_mesh = map_dir / "water.stl"
    green_mesh = map_dir / "green.stl"

    buildings.write_stl(building_mesh)
    roads.write_stl(road_mesh)
    water.write_stl(water_mesh)
    green.write_stl(green_mesh)

    ground_half = radius_m + 25.0
    world_file = world_dir / f"{world_name}.sdf"
    world_file.parent.mkdir(parents=True, exist_ok=True)

    common_visuals = []
    if roads.triangles:
        common_visuals.append(
            f"""
    <model name="osm_roads">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>file://{road_mesh}</uri></mesh></geometry>
          <material>
            <ambient>{rgba(0.18,0.18,0.18)}</ambient>
            <diffuse>{rgba(0.28,0.28,0.28)}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>file://{road_mesh}</uri></mesh></geometry>
        </collision>
      </link>
    </model>
"""
        )

    if buildings.triangles:
        common_visuals.append(
            f"""
    <model name="osm_buildings">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>file://{building_mesh}</uri></mesh></geometry>
          <material>
            <ambient>{rgba(0.55,0.48,0.38)}</ambient>
            <diffuse>{rgba(0.72,0.63,0.50)}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>file://{building_mesh}</uri></mesh></geometry>
        </collision>
      </link>
    </model>
"""
        )

    if water.triangles:
        common_visuals.append(
            f"""
    <model name="osm_water">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>file://{water_mesh}</uri></mesh></geometry>
          <material>
            <ambient>{rgba(0.08,0.25,0.58)}</ambient>
            <diffuse>{rgba(0.12,0.38,0.80)}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        )

    if green.triangles:
        common_visuals.append(
            f"""
    <model name="osm_green">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>file://{green_mesh}</uri></mesh></geometry>
          <material>
            <ambient>{rgba(0.18,0.32,0.10)}</ambient>
            <diffuse>{rgba(0.25,0.50,0.15)}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        )

    world_xml = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world_name}">
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{lat:.9f}</latitude_deg>
      <longitude_deg>{lon:.9f}</longitude_deg>
      <elevation>{elevation_m:.3f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="drone_rakshak_physics" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <scene>
      <ambient>{rgba(0.55,0.55,0.55)}</ambient>
      <background>{rgba(0.70,0.80,0.92)}</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 150 0.35 -0.45 0.25</pose>
      <diffuse>{rgba(0.9,0.9,0.9)}</diffuse>
      <specular>{rgba(0.15,0.15,0.15)}</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_half*2:.1f} {ground_half*2:.1f}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_half*2:.1f} {ground_half*2:.1f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>{rgba(0.20,0.32,0.16)}</ambient>
            <diffuse>{rgba(0.32,0.48,0.22)}</diffuse>
          </material>
        </visual>
      </link>
    </model>

{''.join(common_visuals)}

  </world>
</sdf>
"""
    world_file.write_text(world_xml, encoding="utf-8")

    metadata = {
        "world_name": world_name,
        "latitude": lat,
        "longitude": lon,
        "radius_m": radius_m,
        "spherical_coordinates": True,
        "features": features.stats,
        "meshes": {
            "buildings": str(building_mesh),
            "roads": str(road_mesh),
            "water": str(water_mesh),
            "green": str(green_mesh),
        },
        "attribution": "Map data © OpenStreetMap contributors",
        "license": "Open Database License (ODbL)",
    }
    (map_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )

    return world_name, world_file, map_dir
