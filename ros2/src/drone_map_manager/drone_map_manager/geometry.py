from __future__ import annotations

import math
from typing import Iterable

from pyproj import Transformer
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import transform
from shapely.validation import make_valid


def make_local_transformer(lat: float, lon: float):
    # Local azimuthal-equidistant projection centered on the requested GPS origin.
    # x ~= east, y ~= north in metres.
    local_crs = (
        f"+proj=aeqd +lat_0={lat} +lon_0={lon} "
        "+datum=WGS84 +units=m +no_defs"
    )
    forward = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
    return forward


def clean_polygon(poly: Polygon) -> Polygon | MultiPolygon | None:
    if poly.is_empty:
        return None
    if not poly.is_valid:
        poly = make_valid(poly)
    if poly.is_empty:
        return None
    if poly.geom_type not in ("Polygon", "MultiPolygon"):
        return None
    return poly


def ring_to_xy(transformer, coords: Iterable[tuple[float, float]]):
    return [transformer.transform(lon, lat) for lon, lat in coords]


def polygon_from_latlon(
    transformer,
    coords: Iterable[tuple[float, float]],
    clip_box,
):
    xy = ring_to_xy(transformer, coords)
    if len(xy) < 4:
        return None
    poly = Polygon(xy)
    poly = clean_polygon(poly)
    if poly is None:
        return None
    try:
        poly = poly.intersection(clip_box)
    except Exception:
        poly = poly.buffer(0).intersection(clip_box)
    return clean_polygon(poly)


def line_from_latlon(
    transformer,
    coords: Iterable[tuple[float, float]],
    clip_box,
):
    xy = ring_to_xy(transformer, coords)
    if len(xy) < 2:
        return None
    line = LineString(xy)
    if line.is_empty:
        return None
    try:
        line = line.intersection(clip_box)
    except Exception:
        return None
    if line.is_empty:
        return None
    return line


def iter_polygons(geom):
    if geom is None or geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield geom
    elif geom.geom_type == "MultiPolygon":
        for p in geom.geoms:
            if not p.is_empty:
                yield p
