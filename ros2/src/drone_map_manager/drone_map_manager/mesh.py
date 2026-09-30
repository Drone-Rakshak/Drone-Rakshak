from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

from shapely.geometry import Polygon
from shapely.ops import triangulate


def _normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / norm, ny / norm, nz / norm


class MeshBuilder:
    def __init__(self):
        self.triangles: list[tuple[tuple[float, float, float], ...]] = []

    def add_tri(self, a, b, c):
        self.triangles.append((tuple(a), tuple(b), tuple(c)))

    def add_box(self, x1, y1, x2, y2, z0, z1):
        # axis-aligned cuboid
        p000 = (x1, y1, z0)
        p100 = (x2, y1, z0)
        p110 = (x2, y2, z0)
        p010 = (x1, y2, z0)
        p001 = (x1, y1, z1)
        p101 = (x2, y1, z1)
        p111 = (x2, y2, z1)
        p011 = (x1, y2, z1)

        # bottom
        self.add_tri(p000, p110, p100)
        self.add_tri(p000, p010, p110)
        # top
        self.add_tri(p001, p101, p111)
        self.add_tri(p001, p111, p011)
        # sides
        self.add_tri(p000, p100, p101)
        self.add_tri(p000, p101, p001)
        self.add_tri(p100, p110, p111)
        self.add_tri(p100, p111, p101)
        self.add_tri(p110, p010, p011)
        self.add_tri(p110, p011, p111)
        self.add_tri(p010, p000, p001)
        self.add_tri(p010, p001, p011)

    def add_extruded_polygon(self, poly: Polygon, z0: float, z1: float):
        if poly.is_empty or poly.area < 1e-4:
            return

        # Bottom/top faces: keep only Delaunay triangles fully inside the polygon.
        for tri in triangulate(poly):
            if tri.area < 1e-7 or not poly.covers(tri):
                continue
            coords = list(tri.exterior.coords)[:3]
            a = (coords[0][0], coords[0][1], z0)
            b = (coords[1][0], coords[1][1], z0)
            c = (coords[2][0], coords[2][1], z0)
            self.add_tri(a, c, b)

            a = (coords[0][0], coords[0][1], z1)
            b = (coords[1][0], coords[1][1], z1)
            c = (coords[2][0], coords[2][1], z1)
            self.add_tri(a, b, c)

        # Outer boundary and holes.
        rings = [poly.exterior, *poly.interiors]
        for ring in rings:
            pts = list(ring.coords)
            for p0, p1 in zip(pts, pts[1:]):
                x0, y0 = p0
                x1, y1 = p1
                a = (x0, y0, z0)
                b = (x1, y1, z0)
                c = (x1, y1, z1)
                d = (x0, y0, z1)
                self.add_tri(a, b, c)
                self.add_tri(a, c, d)

    def add_flat_polygon(self, poly: Polygon, z: float, thickness: float = 0.0):
        if thickness > 0:
            self.add_extruded_polygon(poly, z - thickness, z)
            return

        if poly.is_empty or poly.area < 1e-4:
            return
        for tri in triangulate(poly):
            if tri.area < 1e-7 or not poly.covers(tri):
                continue
            coords = list(tri.exterior.coords)[:3]
            self.add_tri(
                (coords[0][0], coords[0][1], z),
                (coords[1][0], coords[1][1], z),
                (coords[2][0], coords[2][1], z),
            )

    def write_stl(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            f.write("solid drone_rakshak_map\n")
            for tri in self.triangles:
                n = _normal(*tri)
                f.write(f"  facet normal {n[0]:.8f} {n[1]:.8f} {n[2]:.8f}\n")
                f.write("    outer loop\n")
                for p in tri:
                    f.write(f"      vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            f.write("endsolid drone_rakshak_map\n")
