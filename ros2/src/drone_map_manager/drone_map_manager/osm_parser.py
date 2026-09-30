from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any

from .geometry import iter_polygons, line_from_latlon, polygon_from_latlon


@dataclass
class FeatureSet:
    buildings: list
    roads: list
    water: list
    parks: list
    wood: list
    stats: dict[str, int]


ROAD_WIDTHS = {
    "motorway": 14.0,
    "trunk": 12.0,
    "primary": 10.0,
    "secondary": 9.0,
    "tertiary": 8.0,
    "residential": 6.0,
    "unclassified": 5.0,
    "service": 4.0,
    "living_street": 5.0,
    "pedestrian": 3.0,
    "cycleway": 2.0,
    "footway": 1.6,
    "path": 1.2,
}


def parse_height(tags: dict[str, Any]) -> float:
    raw = tags.get("height")
    if raw:
        m = re.search(r"[-+]?\d+(?:\.\d+)?", str(raw))
        if m:
            value = float(m.group(0))
            if 1.5 <= value <= 300:
                return value

    raw_levels = tags.get("building:levels")
    if raw_levels:
        m = re.search(r"[-+]?\d+(?:\.\d+)?", str(raw_levels))
        if m:
            levels = float(m.group(0))
            if 1 <= levels <= 100:
                return max(3.0, levels * 3.0)

    return 8.0


def parse_elements(data: dict, transformer, radius_m: float) -> FeatureSet:
    nodes = {
        int(e["id"]): (float(e["lon"]), float(e["lat"]))
        for e in data.get("elements", [])
        if e.get("type") == "node"
        and "lon" in e
        and "lat" in e
    }

    # Small clip region in local coordinates. This prevents accidental giant
    # polygons from entering the Gazebo world.
    from shapely.geometry import box
    clip = box(-radius_m - 20, -radius_m - 20, radius_m + 20, radius_m + 20)

    buildings = []
    roads = []
    water = []
    parks = []
    wood = []

    for e in data.get("elements", []):
        if e.get("type") != "way":
            continue

        refs = e.get("nodes", [])
        tags = e.get("tags", {})
        coords = [nodes[r] for r in refs if r in nodes]

        if len(coords) < 2:
            continue

        if "building" in tags and len(coords) >= 4:
            poly = polygon_from_latlon(transformer, coords, clip)
            for p in iter_polygons(poly):
                buildings.append((p, parse_height(tags)))

        highway = tags.get("highway")
        if highway:
            line = line_from_latlon(transformer, coords, clip)
            if line is not None:
                roads.append((line, ROAD_WIDTHS.get(highway, 4.0)))

        natural = tags.get("natural")
        if natural == "water" and len(coords) >= 4:
            poly = polygon_from_latlon(transformer, coords, clip)
            water.extend(iter_polygons(poly))

        if (
            natural == "wood"
            or tags.get("landuse") == "forest"
        ) and len(coords) >= 4:
            poly = polygon_from_latlon(transformer, coords, clip)
            wood.extend(iter_polygons(poly))

        if (
            tags.get("landuse") == "grass"
            or tags.get("leisure") == "park"
        ) and len(coords) >= 4:
            poly = polygon_from_latlon(transformer, coords, clip)
            parks.extend(iter_polygons(poly))

    return FeatureSet(
        buildings=buildings,
        roads=roads,
        water=water,
        parks=parks,
        wood=wood,
        stats={
            "buildings": len(buildings),
            "roads": len(roads),
            "water": len(water),
            "parks": len(parks),
            "wood": len(wood),
        },
    )
