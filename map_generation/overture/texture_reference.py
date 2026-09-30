#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import Request, urlopen

HOME = Path.home()
TERRAIN_ROOT = HOME / "gazebo_terrain_generator"
GENERATED_ROOT = HOME / "drone_ws" / "generated_maps"
OSM2WORLD_ROOT = HOME / "OSM2World"
WORK_ROOT = HOME / "drone_ws" / "drone_rakshak_worlds"
PORT = 8081

PYX_RES = 65535

def log(msg):
    print(f"[DR] {msg}", flush=True)

def run(cmd, cwd=None, env=None, check=False):
    log("$ " + " ".join(map(str, cmd)))
    return subprocess.run(cmd, cwd=cwd, env=env, check=check)

def start_bg(cmd, cwd=None, env=None, logfile=None):
    log("START " + " ".join(map(str, cmd)))
    if logfile:
        logfile.parent.mkdir(parents=True, exist_ok=True)
        f = open(logfile, "ab", buffering=0)
        return subprocess.Popen(cmd, cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT)
    return subprocess.Popen(cmd, cwd=cwd, env=env)

def kill_matching(patterns):
    for pat in patterns:
        try:
            subprocess.run(["pkill", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

def ensure_terrain_server():
    # Preserve the user's existing 8081 setup, but auto-fix an unmodified 8080 server.py.
    server = TERRAIN_ROOT / "scripts" / "server.py"
    if server.exists():
        txt = server.read_text(errors="ignore")
        if "port=8080" in txt and "port=8081" not in txt:
            server.write_text(txt.replace("port=8080", "port=8081"))
            log("Changed terrain generator port 8080 -> 8081.")

    # Check port.
    probe = subprocess.run(
        ["bash", "-lc", "ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(:|\\])8081$'"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode == 0:
        log("Terrain generator already running on port 8081.")
        return None

    env = os.environ.copy()
    env["GAZEBO_TERRAIN_OUTPUT_PATH"] = str(GENERATED_ROOT)
    log_path = WORK_ROOT / "terrain_generator.log"
    proc = start_bg(
        ["uv", "run", "scripts/server.py"],
        cwd=TERRAIN_ROOT,
        env=env,
        logfile=log_path,
    )
    for _ in range(60):
        probe = subprocess.run(
            ["bash", "-lc", "ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(:|\\])8081$'"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if probe.returncode == 0:
            log("Terrain generator ready at http://localhost:8081")
            return proc
        time.sleep(1)
    raise RuntimeError("Terrain generator did not open port 8081. See ~/drone_ws/drone_rakshak_worlds/terrain_generator.log")

def open_browser():
    url = "http://localhost:8081"
    for cmd in (["explorer.exe", url], ["xdg-open", url]):
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            log(f"Opened {url}")
            return
        except Exception:
            continue
    log(f"Open manually: {url}")

def snapshot_worlds():
    out = {}
    if not GENERATED_ROOT.exists():
        return out
    for d in GENERATED_ROOT.iterdir():
        if not d.is_dir():
            continue
        p = d / f"{d.name}.world"
        if p.exists():
            st = p.stat()
            out[str(p)] = (st.st_mtime_ns, st.st_size)
    return out

def wait_for_new_world(before, timeout=None):
    started = time.time()
    log("Waiting for a new terrain world. Use the browser to choose a location, draw the area, set spawn, and click Generate Terrain.")
    while True:
        now = snapshot_worlds()
        candidates = []
        for p, sig in now.items():
            if p not in before or before[p] != sig:
                candidates.append((sig[0], Path(p)))
        if candidates:
            candidates.sort(reverse=True)
            path = candidates[0][1]
            # wait until size is stable
            last = -1
            stable = 0
            for _ in range(30):
                size = path.stat().st_size
                if size == last:
                    stable += 1
                else:
                    stable = 0
                last = size
                if stable >= 3:
                    log(f"Detected completed terrain world: {path}")
                    return path
                time.sleep(1)
        if timeout and time.time() - started > timeout:
            return None
        time.sleep(2)

def parse_sdf(world_path):
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("World SDF has no <world>")

    spherical = world.find("spherical_coordinates")
    if spherical is None:
        raise RuntimeError("World has no spherical_coordinates")
    lat = float(spherical.findtext("latitude_deg"))
    lon = float(spherical.findtext("longitude_deg"))

    hm = world.find(".//visual/geometry/heightmap")
    if hm is None:
        raise RuntimeError("Could not find terrain visual heightmap")

    uri = hm.findtext("uri")
    size = [float(x) for x in hm.findtext("size").split()]
    pos = [float(x) for x in hm.findtext("pos", default="0 0 0").split()]

    hm_path = (world_path.parent / uri).resolve()

    return {
        "root": root,
        "world": world,
        "center_lat": lat,
        "center_lon": lon,
        "heightmap_uri": uri,
        "heightmap_path": hm_path,
        "size_x": size[0],
        "size_y": size[1],
        "size_z": size[2],
        "pos_x": pos[0],
        "pos_y": pos[1],
        "pos_z": pos[2],
    }

def bbox_from_center_size(lat, lon, sx, sy, margin_m=8.0):
    half_x = sx / 2 + margin_m
    half_y = sy / 2 + margin_m
    dlat = half_y / 111320.0
    dlon = half_x / (111320.0 * math.cos(math.radians(lat)))
    return [lat - dlat, lon - dlon, lat + dlat, lon + dlon]

def bbox_str(bbox):
    s, w, n, e = bbox
    return f"{s:.10f},{w:.10f},{n:.10f},{e:.10f}"

def download(url, out):
    out.parent.mkdir(parents=True, exist_ok=True)
    req = Request(url, headers={"User-Agent": "DroneRakshak/1.0"})
    log(f"Downloading {out.name}")
    with urlopen(req, timeout=180) as r, open(out, "wb") as f:
        shutil.copyfileobj(r, f)
    if out.stat().st_size < 1000:
        raise RuntimeError(f"Downloaded file is suspiciously small: {out}")
    log(f"Downloaded {out.stat().st_size / 1024:.1f} KiB")

def download_osm(bbox, out):
    s, w, n, e = bbox
    url = (
        "https://api.openstreetmap.org/api/0.6/map"
        f"?bbox={w:.10f},{s:.10f},{e:.10f},{n:.10f}"
    )
    download(url, out)

def write_overture_properties(src, dst):
    data = json.loads(src.read_text())
    for feat in data.get("features", []):
        props = feat.setdefault("properties", {})
        if props.get("height") in (None, "", 0):
            if props.get("num_floors"):
                props["building:levels"] = props["num_floors"]
        else:
            props["height"] = props["height"]
    dst.write_text(json.dumps(data))
    return len(data.get("features", []))

def sample_height(px, py, image, bbox, size_z, pose_z):
    # Same mapping used by the terrain generator's buildings generator:
    # latitude maps north->top, longitude maps west->right.
    south, west, north, east = bbox
    w, h = image.size
    x = int((px - west) / (east - west) * w)
    y = int((north - py) / (north - south) * h)
    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))
    val = image.getpixel((x, y))
    return (val / PYX_RES) * size_z + pose_z - 0.1


def _landmark_sources():
    return {
        "KIET": {
            "keywords": ("kiet", "krishna institute of engineering", "kiet group"),
            "style": "kiet",
            "source": "https://www.kiet.edu/about/infrastructure/",
        },
        "I.T.S.": {
            "keywords": ("i.t.s", "i.t.s.", "institute of technology and science",
                         "its institute", "its mohan nagar", "its mohannagar"),
            "style": "its",
            "source": "https://ug.its.edu.in/photo-gallery-2025",
        },
    }


def _make_building_texture(path, style, label=""):
    from PIL import Image, ImageDraw, ImageFont
    import hashlib
    import random

    path = Path(path)
    if path.exists():
        return

    profiles = {
        "kiet": ((224,216,198),(153,54,42),(40,58,68)),
        "its": ((194,175,145),(101,62,43),(43,57,63)),
        "education": ((213,208,194),(133,78,53),(47,65,74)),
        "commercial": ((186,192,194),(57,92,109),(30,60,76)),
        "industrial": ((151,157,157),(77,91,93),(45,61,67)),
        "res_a": ((226,215,194),(166,110,72),(46,67,74)),
        "res_b": ((210,201,188),(111,122,102),(45,64,73)),
        "res_c": ((222,202,181),(132,83,68),(41,61,72)),
    }

    base, accent, window = profiles.get(
        style,
        profiles["res_a"]
    )

    rng = random.Random(
        int(hashlib.sha256(
            f"{style}:{label}".encode()
        ).hexdigest()[:8], 16)
    )

    img = Image.new("RGB", (512,512), base)
    px = img.load()

    for y in range(512):
        for x in range(512):
            d = rng.randint(-4,4)
            px[x,y] = tuple(
                max(0,min(255,c+d))
                for c in base
            )

    draw = ImageDraw.Draw(img)

    floor = 72

    for y in range(0,512,floor):
        draw.rectangle(
            (0,y,512,y+4),
            fill=(115,110,103)
        )

        draw.rectangle(
            (0,y+5,512,y+9),
            fill=accent
        )

        for x in range(18,492,58):
            draw.rectangle(
                (x,y+16,x+34,y+56),
                fill=window
            )

            draw.rectangle(
                (x+4,y+20,x+30,y+24),
                fill=(115,139,148)
            )

    if style in ("kiet","its","education"):
        for x in (70,438):
            draw.rectangle(
                (x,0,x+12,512),
                fill=accent
            )

    if style in ("kiet","its") and label:
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                22
            )
        except Exception:
            font = ImageFont.load_default()

        draw.rectangle(
            (8,8,190,42),
            fill=(245,241,231)
        )

        draw.text(
            (14,12),
            label[:18],
            fill=accent,
            font=font
        )

    path.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    img.save(path, "PNG")


def _building_uv(mesh):
    import numpy as np

    mesh.unmerge_vertices()

    vertices = np.asarray(mesh.vertices)
    uv = np.zeros(
        (len(vertices),2),
        dtype=float
    )

    if len(vertices) == 0:
        return uv

    mn = vertices.min(axis=0)
    mx = vertices.max(axis=0)

    sx, sy, sz = [
        max(float(v),0.1)
        for v in (mx-mn)
    ]

    for face_index, face in enumerate(mesh.faces):
        normal = mesh.face_normals[face_index]

        for vertex_index in face:
            x,y,z = vertices[vertex_index]

            if abs(normal[2]) > 0.55:
                uv[vertex_index] = [
                    (x-mn[0])/sx*2,
                    (y-mn[1])/sy*2
                ]

            elif abs(normal[0]) >= abs(normal[1]):
                uv[vertex_index] = [
                    (y-mn[1])/sy*2,
                    (z-mn[2])/sz
                ]

            else:
                uv[vertex_index] = [
                    (x-mn[0])/sx*2,
                    (z-mn[2])/sz
                ]

    return uv


def _osm_named_landmarks(osm_source):
    import xml.etree.ElementTree as ET

    if not osm_source:
        return []

    try:
        root = ET.parse(osm_source).getroot()
    except Exception as e:
        log(f"Landmark parse skipped: {e}")
        return []

    education_values = {
        "college",
        "university",
        "school",
        "institute",
        "campus",
    }

    landmarks = []

    # --------------------------------------------------------
    # Named OSM nodes
    # --------------------------------------------------------
    for node in root.findall("node"):
        lat = node.get("lat")
        lon = node.get("lon")

        if lat is None or lon is None:
            continue

        tags = {
            t.get("k", ""): t.get("v", "")
            for t in node.findall("tag")
        }

        name = (
            tags.get("name")
            or tags.get("name:en")
            or tags.get("official_name")
        )

        if not name:
            continue

        text = str(name).lower()
        amenity = str(
            tags.get("amenity", "")
        ).lower()

        building = str(
            tags.get("building", "")
        ).lower()

        if not (
            amenity in education_values
            or building in education_values
            or any(
                k in text
                for k in (
                    "kiet",
                    "i.t.s",
                    "its college",
                    "its institute",
                    "institute of technology",
                    "university",
                    "college",
                    "school",
                    "institute",
                )
            )
        ):
            continue

        landmarks.append({
            "name": str(name),
            "lat": float(lat),
            "lon": float(lon),
            "tags": tags,
        })

    # --------------------------------------------------------
    # Named OSM ways / campus polygons
    # --------------------------------------------------------
    points = {}

    for node in root.findall("node"):
        if node.get("lat") and node.get("lon"):
            points[node.get("id")] = (
                float(node.get("lat")),
                float(node.get("lon"))
            )

    for way in root.findall("way"):
        tags = {
            t.get("k", ""): t.get("v", "")
            for t in way.findall("tag")
        }

        name = (
            tags.get("name")
            or tags.get("name:en")
            or tags.get("official_name")
        )

        if not name:
            continue

        text = str(name).lower()

        amenity = str(
            tags.get("amenity", "")
        ).lower()

        building = str(
            tags.get("building", "")
        ).lower()

        if not (
            amenity in education_values
            or building in education_values
            or any(
                k in text
                for k in (
                    "kiet",
                    "i.t.s",
                    "its college",
                    "its institute",
                    "college",
                    "university",
                    "school",
                    "institute",
                )
            )
        ):
            continue

        refs = [
            nd.get("ref")
            for nd in way.findall("nd")
        ]

        pts = [
            points[r]
            for r in refs
            if r in points
        ]

        if len(pts) < 2:
            continue

        landmarks.append({
            "name": str(name),
            "lat": sum(v[0] for v in pts) / len(pts),
            "lon": sum(v[1] for v in pts) / len(pts),
            "tags": tags,
        })

    # Remove near-duplicate entries.
    unique = {}

    for item in landmarks:
        key = (
            item["name"].lower().strip(),
            round(item["lat"], 5),
            round(item["lon"], 5),
        )
        unique[key] = item

    landmarks = list(
        unique.values()
    )

    log(
        f"Named education landmarks: "
        f"{len(landmarks)}"
    )

    for item in landmarks:
        log(
            f"  landmark: {item['name']} "
            f"@ {item['lat']:.7f}, "
            f"{item['lon']:.7f}"
        )

    return landmarks



def _places_landmarks(places_source):
    import json
    from shapely.geometry import shape

    path = Path(places_source)

    if not path.exists():
        log(f"Overture places file not found: {path}")
        return []

    try:
        data = json.loads(path.read_text())
    except Exception as e:
        log(f"Overture places parse failed: {e}")
        return []

    education_words = (
        "college",
        "university",
        "school",
        "institute",
        "campus",
        "academy",
    )

    landmarks = []

    for feature in data.get("features", []):
        props = feature.get("properties", {}) or {}
        geometry_data = feature.get("geometry")

        if not geometry_data:
            continue

        try:
            geometry = shape(geometry_data)

            if geometry.is_empty:
                continue

            point = (
                geometry
                if geometry.geom_type == "Point"
                else geometry.centroid
            )

            lat = float(point.y)
            lon = float(point.x)

        except Exception:
            continue

        text = json.dumps(
            props,
            default=str
        ).lower()

        if not any(
            word in text
            for word in education_words
        ):
            continue

        name = ""

        names = props.get("names")

        if isinstance(names, dict):
            primary = names.get("primary")

            if isinstance(primary, str):
                name = primary
            elif isinstance(primary, dict):
                name = (
                    primary.get("value")
                    or primary.get("name")
                    or ""
                )

        if not name:
            for key in (
                "name",
                "official_name",
                "common_name",
            ):
                value = props.get(key)
                if isinstance(value, str):
                    name = value
                    break

        if not name:
            continue

        website = ""

        websites = props.get("websites")

        if isinstance(websites, list):
            for item in websites:
                if isinstance(item, str):
                    website = item
                    break

                if isinstance(item, dict):
                    website = (
                        item.get("value")
                        or item.get("url")
                        or ""
                    )

                    if website:
                        break

        elif isinstance(websites, str):
            website = websites

        landmarks.append({
            "name": str(name),
            "lat": lat,
            "lon": lon,
            "website": website,
            "properties": props,
        })

    # Remove duplicate place records.
    unique = {}

    for place in landmarks:
        key = (
            place["name"].strip().lower(),
            round(place["lat"], 5),
            round(place["lon"], 5),
        )

        unique[key] = place

    landmarks = list(unique.values())

    log(
        f"Overture education places: "
        f"{len(landmarks)}"
    )

    for place in landmarks:
        log(
            f"  place: {place['name']} "
            f"@ {place['lat']:.7f}, "
            f"{place['lon']:.7f}"
        )

        if place["website"]:
            log(
                f"    website: {place['website']}"
            )

    return landmarks


def _distance_m(lat1,lon1,lat2,lon2):
    import math

    return math.hypot(
        (lat2-lat1)*111320.0,
        (lon2-lon1)
        *111320.0
        *math.cos(
            math.radians(
                (lat1+lat2)/2
            )
        )
    )


def _building_style(props, lat, lon, landmarks):
    import hashlib
    import re

    sources = _landmark_sources()

    values = []

    for key in (
        "name",
        "official_name",
        "common_name",
        "primary_name",
        "class",
        "subtype",
    ):
        if props.get(key):
            values.append(
                str(props[key])
            )

    names = props.get("names")

    if isinstance(names,dict):
        for value in names.values():
            if isinstance(value,dict):
                name = (
                    value.get("value")
                    or value.get("name")
                )

                if name:
                    values.append(str(name))

    text = " ".join(values).lower()

    if any(
        k in text
        for k in sources["KIET"]["keywords"]
    ):
        return (
            "kiet",
            "KIET",
            sources["KIET"]["source"]
        )

    if (
        any(
            k in text
            for k in sources["I.T.S."]["keywords"]
        )
        or re.search(
            r"\bi\.?t\.?s\.?\b",
            text
        )
    ):
        return (
            "its",
            "I.T.S.",
            sources["I.T.S."]["source"]
        )

    best = None
    best_distance = 80.0

    for landmark in landmarks:
        distance = _distance_m(
            lat,
            lon,
            landmark["lat"],
            landmark["lon"]
        )

        if distance < best_distance:
            best = landmark
            best_distance = distance

    if best:
        name = best["name"]
        lower = name.lower()

        if "kiet" in lower:
            return (
                "kiet",
                name,
                sources["KIET"]["source"]
            )

        if (
            "i.t.s" in lower
            or "institute of technology and science" in lower
            or re.search(r"\bits\b",lower)
        ):
            return (
                "its",
                name,
                sources["I.T.S."]["source"]
            )

        if any(
            k in lower
            for k in (
                "college",
                "university",
                "institute",
                "school",
                "campus"
            )
        ):
            return (
                "education",
                name,
                "OpenStreetMap"
            )

    building_class = str(
        props.get("class")
        or props.get("subtype")
        or ""
    ).lower()

    if any(
        k in building_class
        for k in ("commercial","retail","office")
    ):
        style = "commercial"

    elif any(
        k in building_class
        for k in ("industrial","warehouse")
    ):
        style = "industrial"

    elif any(
        k in building_class
        for k in (
            "education",
            "school",
            "college",
            "university"
        )
    ):
        style = "education"

    else:
        styles = (
            "res_a",
            "res_b",
            "res_c"
        )

        seed = int(
            hashlib.sha256(
                f"{lat:.7f},{lon:.7f}".encode()
            ).hexdigest()[:8],
            16
        )

        style = styles[
            seed % len(styles)
        ]

    return style, style, ""



def _make_style_texture(path, style, label="", roof=False):
    from PIL import Image, ImageDraw, ImageFont
    import hashlib
    import random

    path = Path(path)

    if path.exists():
        return

    facade_profiles = {
        "kiet": ((224,216,198),(153,54,42),(40,58,68)),
        "its": ((194,175,145),(101,62,43),(43,57,63)),
        "education": ((213,208,194),(133,78,53),(47,65,74)),
        "commercial": ((186,192,194),(57,92,109),(30,60,76)),
        "industrial": ((151,157,157),(77,91,93),(45,61,67)),
        "res_a": ((226,215,194),(166,110,72),(46,67,74)),
        "res_b": ((210,201,188),(111,122,102),(45,64,73)),
        "res_c": ((222,202,181),(132,83,68),(41,61,72)),
    }

    roof_profiles = {
        "kiet": ((76,67,61),(125,52,40)),
        "its": ((78,69,60),(115,79,52)),
        "education": ((82,82,78),(112,82,58)),
        "commercial": ((69,77,80),(98,108,111)),
        "industrial": ((68,72,72),(92,99,98)),
        "res_a": ((82,75,68),(121,91,67)),
        "res_b": ((76,79,75),(106,110,100)),
        "res_c": ((82,71,67),(118,81,69)),
    }

    if roof:
        base, accent = roof_profiles.get(
            style,
            roof_profiles["res_a"]
        )

        rng = random.Random(
            int(
                hashlib.sha256(
                    f"roof:{style}:{label}".encode()
                ).hexdigest()[:8],
                16,
            )
        )

        img = Image.new(
            "RGB",
            (512,512),
            base
        )

        draw = ImageDraw.Draw(img)

        for y in range(0, 512, 32):
            shade = tuple(
                max(
                    0,
                    min(
                        255,
                        c + rng.randint(-7,7)
                    )
                )
                for c in accent
            )

            draw.rectangle(
                (0,y,512,y+3),
                fill=shade
            )

            for x in range(-32, 544, 48):
                draw.line(
                    (
                        x,
                        y,
                        x+24,
                        y+32
                    ),
                    fill=tuple(
                        max(
                            0,
                            min(
                                255,
                                c + rng.randint(-8,8)
                            )
                        )
                        for c in accent
                    ),
                    width=2
                )

        path.parent.mkdir(
            parents=True,
            exist_ok=True
        )
        img.save(path, "PNG")
        return

    base, accent, window = facade_profiles.get(
        style,
        facade_profiles["res_a"]
    )

    rng = random.Random(
        int(
            hashlib.sha256(
                f"facade:{style}:{label}".encode()
            ).hexdigest()[:8],
            16,
        )
    )

    img = Image.new(
        "RGB",
        (512,512),
        base
    )

    px = img.load()

    for y in range(512):
        for x in range(512):
            d = rng.randint(-4,4)
            px[x,y] = tuple(
                max(
                    0,
                    min(
                        255,
                        c+d
                    )
                )
                for c in base
            )

    draw = ImageDraw.Draw(img)

    for y in range(0,512,72):
        draw.rectangle(
            (0,y,512,y+4),
            fill=(115,110,103)
        )

        draw.rectangle(
            (0,y+5,512,y+9),
            fill=accent
        )

        for x in range(18,492,58):
            draw.rectangle(
                (x,y+16,x+34,y+56),
                fill=window
            )

            draw.rectangle(
                (x+4,y+20,x+30,y+24),
                fill=(115,139,148)
            )

    if style in (
        "kiet",
        "its",
        "education"
    ):
        for x in (70,438):
            draw.rectangle(
                (x,0,x+12,512),
                fill=accent
            )

    if style in (
        "kiet",
        "its"
    ) and label:
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                22
            )
        except Exception:
            font = ImageFont.load_default()

        draw.rectangle(
            (8,8,190,42),
            fill=(245,241,231)
        )

        draw.text(
            (14,12),
            label[:18],
            fill=accent,
            font=font
        )

    path.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    img.save(path, "PNG")


def build_overture_glb(
    geojson_in,
    out_glb,
    terrain,
    osm_source=None,
    places_source=None
):
    import json
    import trimesh
    import numpy as np

    from PIL import Image
    from pyproj import Transformer
    from shapely.geometry import (
        shape,
        box,
        mapping
    )

    sys.path.insert(
        0,
        str(TERRAIN_ROOT / "scripts")
    )

    from utils.buildings_generator import GeoJSONToDAE

    image = Image.open(
        terrain["heightmap_path"]
    )

    if image.mode not in (
        "I",
        "I;16",
        "I;16B",
        "I;16L"
    ):
        image = image.convert("I")

    bbox = bbox_from_center_size(
        terrain["center_lat"],
        terrain["center_lon"],
        terrain["size_x"],
        terrain["size_y"],
        margin_m=0
    )

    boundaries = {
        "southwest": [
            bbox[0],
            bbox[1]
        ],
        "northeast": [
            bbox[2],
            bbox[3]
        ]
    }

    data = json.loads(
        Path(geojson_in).read_text()
    )

    terrain_clip = box(
        bbox[1],
        bbox[0],
        bbox[3],
        bbox[2]
    )

    clipped_features = []

    for feature in data.get(
        "features",
        []
    ):
        geometry_data = feature.get(
            "geometry"
        )

        if not geometry_data:
            continue

        try:
            geometry = shape(
                geometry_data
            ).intersection(
                terrain_clip
            )

            if geometry.is_empty:
                continue

            if not geometry.is_valid:
                geometry = geometry.buffer(0)

            if geometry.is_empty:
                continue

            clipped_feature = dict(
                feature
            )

            clipped_feature[
                "geometry"
            ] = mapping(geometry)

            clipped_features.append(
                clipped_feature
            )

        except Exception:
            continue

    clipped_geojson = out_glb.with_name(
        out_glb.stem
        + "_clipped.geojson"
    )

    clipped_geojson.write_text(
        json.dumps({
            "type":"FeatureCollection",
            "features":clipped_features
        })
    )

    log(
        f"Overture clipped: "
        f"{len(data.get('features',[]))} -> "
        f"{len(clipped_features)} buildings"
    )

    generator = GeoJSONToDAE(
        str(clipped_geojson),
        str(
            out_glb.with_suffix(
                ".tmp.dae"
            )
        )
    )

    generator.center_lat = (
        terrain["center_lat"]
    )

    generator.center_lon = (
        terrain["center_lon"]
    )

    generator.size_z = (
        terrain["size_z"]
    )

    generator.pose_z = (
        terrain["pos_z"]
    )

    generator.heightmap = image
    generator.heightmap_z_resolution = 65535
    generator.bounds = boundaries

    gdf = generator.load()

    to_wgs84 = Transformer.from_crs(
        gdf.crs,
        "EPSG:4326",
        always_xy=True
    )

    if places_source:
        landmarks = _places_landmarks(
            places_source
        )
    else:
        landmarks = _osm_named_landmarks(
            osm_source
        )

    texture_dir = (
        out_glb.parent
        / "building_textures"
    )

    texture_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    import numpy as np

    landmarks = _osm_named_landmarks(
        osm_source
    )

    texture_dir = (
        out_glb.parent /
        "building_textures"
    )

    texture_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    texture_cache = {}
    scene = trimesh.Scene()
    style_counts = {}

    for feature_index, row in gdf.iterrows():
        props = row.drop(
            "geometry"
        ).to_dict()

        for part_index, geometry in enumerate(
            generator.flatten_geometry(
                row.geometry
            )
        ):
            if geometry.is_empty:
                continue

            centroid = geometry.centroid

            lon, lat = to_wgs84.transform(
                centroid.x,
                centroid.y
            )

            height = max(
                generator.get_height(props),
                generator.DEFAULT_HEIGHT
            )

            if geometry.geom_type == "Polygon":
                mesh = generator.handle_polygon(
                    geometry,
                    height
                )

            elif geometry.geom_type == "Point":
                mesh = generator.handle_point(
                    geometry
                )

            elif geometry.geom_type == "LineString":
                mesh = generator.handle_line(
                    geometry
                )

            else:
                mesh = None

            if mesh is None:
                continue

            mesh.apply_translation([
                0,
                0,
                generator.get_pixel_elevation(
                    lat,
                    lon
                )
            ])

            style, label, source = _building_style(
                props,
                lat,
                lon,
                landmarks
            )

            style_counts[style] = (
                style_counts.get(style, 0) + 1
            )

            if style not in texture_cache:
                facade_path = (
                    texture_dir /
                    f"{style}_facade.png"
                )

                roof_path = (
                    texture_dir /
                    f"{style}_roof.png"
                )

                _make_style_texture(
                    facade_path,
                    style,
                    label,
                    roof=False
                )

                _make_style_texture(
                    roof_path,
                    style,
                    label,
                    roof=True
                )

                texture_cache[style] = {
                    "facade":
                        Image.open(
                            facade_path
                        ).convert("RGB"),

                    "roof":
                        Image.open(
                            roof_path
                        ).convert("RGB"),
                }

            # ------------------------------------------------
            # Overture building meshes are Y-up:
            # X = horizontal
            # Y = height
            # Z = horizontal
            #
            # Split the mesh into:
            #   1. walls
            #   2. roofs
            #
            # They then get DIFFERENT materials.
            # ------------------------------------------------
            mesh = mesh.copy()
            mesh.unmerge_vertices()

            normals = np.asarray(
                mesh.face_normals
            )

            roof_faces = (
                np.abs(normals[:,1]) > 0.55
            )

            wall_faces = ~roof_faces

            def create_submesh(mask):
                if not np.any(mask):
                    return None

                sub = mesh.submesh(
                    [mask],
                    append=True,
                    repair=False
                )

                if isinstance(sub, list):
                    if not sub:
                        return None
                    sub = sub[0]

                sub.remove_unreferenced_vertices()
                sub.unmerge_vertices()

                return sub

            wall_mesh = create_submesh(
                wall_faces
            )

            roof_mesh = create_submesh(
                roof_faces
            )

            # ----------------------------------------------
            # WALL UVs
            # ----------------------------------------------
            if wall_mesh is not None and len(wall_mesh.faces):
                verts = np.asarray(
                    wall_mesh.vertices,
                    dtype=float
                )

                uv = np.zeros(
                    (len(verts),2),
                    dtype=float
                )

                mn = verts.min(axis=0)
                mx = verts.max(axis=0)

                sx = max(
                    float(mx[0]-mn[0]),
                    0.1
                )

                sy = max(
                    float(mx[1]-mn[1]),
                    0.1
                )

                sz = max(
                    float(mx[2]-mn[2]),
                    0.1
                )

                for fi, face in enumerate(
                    wall_mesh.faces
                ):
                    n = wall_mesh.face_normals[fi]

                    for vi in face:
                        x,y,z = verts[vi]

                        if abs(n[0]) >= abs(n[2]):
                            u = (
                                z-mn[2]
                            ) / sz
                        else:
                            u = (
                                x-mn[0]
                            ) / sx

                        v = (
                            y-mn[1]
                        ) / sy

                        uv[vi] = [
                            u % 1.0,
                            v % 1.0
                        ]

                wall_mesh.visual = (
                    trimesh.visual.texture.TextureVisuals(
                        uv=uv,
                        material=(
                            trimesh.visual.material.SimpleMaterial(
                                image=texture_cache[
                                    style
                                ]["facade"],
                                name=f"{style}_facade"
                            )
                        )
                    )
                )

                scene.add_geometry(
                    wall_mesh,
                    node_name=(
                        f"building_{feature_index}_"
                        f"{part_index}_walls"
                    ),
                    geom_name=(
                        f"building_{feature_index}_"
                        f"{part_index}_walls"
                    )
                )

            # ----------------------------------------------
            # ROOF UVs
            # ----------------------------------------------
            if roof_mesh is not None and len(roof_mesh.faces):
                verts = np.asarray(
                    roof_mesh.vertices,
                    dtype=float
                )

                uv = np.zeros(
                    (len(verts),2),
                    dtype=float
                )

                mn = verts.min(axis=0)
                mx = verts.max(axis=0)

                sx = max(
                    float(mx[0]-mn[0]),
                    0.1
                )

                sz = max(
                    float(mx[2]-mn[2]),
                    0.1
                )

                for fi, face in enumerate(
                    roof_mesh.faces
                ):
                    for vi in face:
                        x,y,z = verts[vi]

                        uv[vi] = [
                            ((x-mn[0])/sx) % 1.0,
                            ((z-mn[2])/sz) % 1.0
                        ]

                # IMPORTANT:
                # Keep roofs separate from facade textures.
                # For this test, roofs use a plain material so it is
                # visually obvious that facade textures are only applied
                # to vertical building walls.
                roof_colors = {
                    "kiet": [92, 75, 65, 255],
                    "its": [88, 76, 64, 255],
                    "education": [96, 92, 84, 255],
                    "commercial": [92, 96, 98, 255],
                    "industrial": [82, 86, 86, 255],
                    "res_a": [96, 84, 72, 255],
                    "res_b": [88, 91, 85, 255],
                    "res_c": [96, 80, 74, 255],
                }

                roof_color = roof_colors.get(
                    style,
                    [90, 85, 80, 255]
                )

                roof_mesh.visual = (
                    trimesh.visual.ColorVisuals(
                        face_colors=np.tile(
                            roof_color,
                            (len(roof_mesh.faces), 1)
                        )
                    )
                )

                scene.add_geometry(
                    roof_mesh,
                    node_name=(
                        f"building_{feature_index}_"
                        f"{part_index}_roof"
                    ),
                    geom_name=(
                        f"building_{feature_index}_"
                        f"{part_index}_roof"
                    )
                )

    if not scene.geometry:
        raise RuntimeError(
            "No Overture building meshes were generated."
        )

    out_glb.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    obj_path = out_glb.with_suffix(".obj")

    mtl_name = (
        obj_path.stem +
        ".mtl"
    )

    exported, assets = (
        trimesh.exchange.obj.export_obj(
            scene,
            include_normals=True,
            include_texture=True,
            return_texture=True,
            write_texture=False,
            mtl_name=mtl_name
        )
    )

    obj_path.write_text(
        exported,
        encoding="utf-8"
    )

    for filename, payload in assets.items():
        target = (
            obj_path.parent /
            filename
        )

        if isinstance(payload, str):
            payload = payload.encode(
                "utf-8"
            )

        target.write_bytes(payload)

    manifest = {
        "landmark_sources":
            _landmark_sources(),
        "styles":
            style_counts,
        "texture_directory":
            str(texture_dir),
        "materials":
            "separate wall and roof materials",
    }

    (
        out_glb.parent /
        "building_texture_manifest.json"
    ).write_text(
        json.dumps(
            manifest,
            indent=2
        )
    )

    log(
        "Building textures: "
        + ", ".join(
            f"{k}={v}"
            for k,v in sorted(
                style_counts.items()
            )
        )
    )

    log(
        f"Overture textured OBJ: "
        f"{obj_path}"
    )

def read_glb(path):
    data = path.read_bytes()
    magic, version, total = __import__("struct").unpack_from("<4sII", data, 0)
    if magic != b"glTF":
        raise RuntimeError(f"{path} is not a GLB")
    chunks = []
    import struct
    off = 12
    json_idx = None
    while off < len(data):
        size, kind = struct.unpack_from("<II", data, off)
        payload = data[off + 8:off + 8 + size]
        chunks.append((kind, payload))
        if kind == 0x4E4F534A:
            json_idx = len(chunks) - 1
        off += 8 + size
    doc = json.loads(chunks[json_idx][1].decode("utf-8").rstrip("\x00 "))
    return chunks, json_idx, doc

def write_glb(chunks, json_idx, doc, out):
    import struct
    b = json.dumps(doc, separators=(",", ":"), ensure_ascii=False).encode()
    b += b" " * ((4 - len(b) % 4) % 4)
    chunks[json_idx] = (0x4E4F534A, b)
    total = 12 + sum(8 + len(p) for _, p in chunks)
    with open(out, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        for kind, payload in chunks:
            f.write(struct.pack("<II", len(payload), kind))
            f.write(payload)

def filter_remove_roots(doc, prefixes=("Building", "SurfaceArea", "HighVoltagePowerTower")):
    nodes = doc.get("nodes", [])
    children_map = {i: list(n.get("children", [])) for i, n in enumerate(nodes)}
    remove = set()

    def collect(i):
        if i in remove:
            return
        remove.add(i)
        for c in children_map.get(i, []):
            collect(c)

    for i, n in enumerate(nodes):
        name = str(n.get("name", ""))
        if any(name.startswith(p) for p in prefixes):
            collect(i)

    for n in nodes:
        if "children" in n:
            n["children"] = [c for c in n["children"] if c not in remove]

    for s in doc.get("scenes", []):
        s["nodes"] = [i for i in s.get("nodes", []) if i not in remove]

    return remove

def glb_mesh_bounds(doc):
    out = {}
    for mi, mesh in enumerate(doc.get("meshes", [])):
        bounds = None
        for prim in mesh.get("primitives", []):
            ai = prim.get("attributes", {}).get("POSITION")
            if ai is None:
                continue
            acc = doc["accessors"][ai]
            if "min" not in acc or "max" not in acc:
                continue
            mn, mx = acc["min"], acc["max"]
            if bounds is None:
                bounds = [mn[:], mx[:]]
            else:
                for j in range(3):
                    bounds[0][j] = min(bounds[0][j], mn[j])
                    bounds[1][j] = max(bounds[1][j], mx[j])
        if bounds:
            out[mi] = bounds
    return out


def clip_osm_roads_to_bbox(src, out, bbox):
    import xml.etree.ElementTree as ET

    min_lat, min_lon, max_lat, max_lon = bbox

    root = ET.parse(src).getroot()

    nodes = {}
    node_elements = {}

    for n in root.findall("node"):
        nid = n.attrib["id"]
        lat = float(n.attrib["lat"])
        lon = float(n.attrib["lon"])

        nodes[nid] = (lat, lon)
        node_elements[nid] = n

    kept_node_ids = set()
    road_ways = []

    def inside(nid):
        if nid not in nodes:
            return False

        lat, lon = nodes[nid]

        return (
            min_lat <= lat <= max_lat
            and min_lon <= lon <= max_lon
        )

    for way in root.findall("way"):
        tags = {
            t.attrib.get("k", ""): t.attrib.get("v", "")
            for t in way.findall("tag")
        }

        # OSM highways only.
        if "highway" not in tags:
            continue

        refs = [
            nd.attrib["ref"]
            for nd in way.findall("nd")
        ]

        segment = []

        def flush():
            if len(segment) >= 2:
                road_ways.append(
                    (dict(tags), list(segment))
                )

        for ref in refs:
            if inside(ref):
                segment.append(ref)
                kept_node_ids.add(ref)
            else:
                flush()
                segment = []

        flush()

    osm = ET.Element(
        "osm",
        version="0.6",
        generator="DroneRakshak road clipper"
    )

    # Nodes used by kept road segments.
    for nid in kept_node_ids:
        osm.append(node_elements[nid])

    next_id = -1

    for tags, refs in road_ways:
        way = ET.SubElement(
            osm,
            "way",
            id=str(next_id)
        )
        next_id -= 1

        for ref in refs:
            ET.SubElement(
                way,
                "nd",
                ref=ref
            )

        for k, v in tags.items():
            ET.SubElement(
                way,
                "tag",
                k=k,
                v=v
            )

    ET.ElementTree(osm).write(
        out,
        encoding="utf-8",
        xml_declaration=True
    )

    log(
        f"OSM roads clipped: "
        f"{len(road_ways)} road segments, "
        f"{len(kept_node_ids)} nodes"
    )

def osm2world_origin_from_osm(osm_file):
    import xml.etree.ElementTree as ET

    root = ET.parse(osm_file).getroot()

    lats = []
    lons = []

    for node in root.findall("node"):
        lat = node.get("lat")
        lon = node.get("lon")

        if lat is None or lon is None:
            continue

        lats.append(float(lat))
        lons.append(float(lon))

    if not lats:
        raise RuntimeError(f"No OSM nodes found in {osm_file}")

    return (
        (min(lats) + max(lats)) / 2.0,
        (min(lons) + max(lons)) / 2.0,
    )


def drape_osm_glb(src, out, terrain, osm_source=None):
    from PIL import Image
    chunks, ji, doc = read_glb(src)
    removed = filter_remove_roots(doc)
    bounds = glb_mesh_bounds(doc)
    image = Image.open(terrain["heightmap_path"]).convert("I")
    bbox = bbox_from_center_size(
        terrain["center_lat"],
        terrain["center_lon"],
        terrain["size_x"],
        terrain["size_y"],
        margin_m=0,
    )

    nodes = doc.get("nodes", [])
    translated = 0
    skipped = 0
    south, west, north, east = bbox
    iw, ih = image.size

    # OSM2World places (0,0,0) at the geographic center of the input
    # node bounds. With complete_ways, those bounds can extend far beyond
    # the requested terrain bbox. Move the OSM2World local coordinate frame
    # so the terrain center becomes (0,0,0) in Gazebo.
    if osm_source is not None:
        osm_origin_lat, osm_origin_lon = osm2world_origin_from_osm(osm_source)
    else:
        osm_origin_lat = terrain["center_lat"]
        osm_origin_lon = terrain["center_lon"]

    lon_scale = 111320.0 * math.cos(
        math.radians(terrain["center_lat"])
    )

    origin_shift_x = (
        osm_origin_lon - terrain["center_lon"]
    ) * lon_scale

    origin_shift_north = (
        osm_origin_lat - terrain["center_lat"]
    ) * 111320.0

    log(
        f"OSM2World origin: "
        f"{osm_origin_lat:.8f}, {osm_origin_lon:.8f}; "
        f"shift: X={origin_shift_x:.2f} m, "
        f"North={origin_shift_north:.2f} m"
    )

    # IMPORTANT:
    # The OSM2World GLB is still physically located in its original
    # coordinate frame. Move the complete OSM scene so that the terrain
    # center becomes (0,0,0).
    #
    # OSM2World:
    #   X = east
    #   Z = south
    #
    # Therefore:
    #   X shift = origin_lon - terrain_lon
    #   Z shift = -(north shift)
    horizontal_shift = [
        origin_shift_x,
        0.0,
        -origin_shift_north,
    ]

    for scene in doc.get("scenes", []):
        for root_index in scene.get("nodes", []):
            root_node = doc["nodes"][root_index]
            old_tr = root_node.get("translation", [0.0, 0.0, 0.0])
            root_node["translation"] = [
                old_tr[0] + horizontal_shift[0],
                old_tr[1],
                old_tr[2] + horizontal_shift[2],
            ]

    log(
        f"Applied OSM horizontal translation: "
        f"X={horizontal_shift[0]:.2f} m, "
        f"Z={horizontal_shift[2]:.2f} m"
    )

    terrain_half_x = terrain["size_x"] / 2.0
    terrain_half_y = terrain["size_y"] / 2.0

    for n in nodes:
        if "mesh" not in n:
            continue

        mi = n["mesh"]

        if mi not in bounds:
            continue

        mn, mx = bounds[mi]

        # Mesh bounds in OSM2World coordinates -> terrain-centered
        # Gazebo coordinates.
        mesh_min_x = mn[0] + origin_shift_x
        mesh_max_x = mx[0] + origin_shift_x

        mesh_min_north = -mx[2] + origin_shift_north
        mesh_max_north = -mn[2] + origin_shift_north

        x_m = (mesh_min_x + mesh_max_x) / 2.0
        north_m = (mesh_min_north + mesh_max_north) / 2.0

        node_name = str(n.get("name", ""))

        # Roads are allowed to cross the terrain boundary. Keep the road
        # when its center lies inside the selected terrain.
        if node_name.startswith("Road "):
            if (
                x_m < -terrain_half_x
                or x_m > terrain_half_x
                or north_m < -terrain_half_y
                or north_m > terrain_half_y
            ):
                skipped += 1
                continue

        # Other meshes must fit completely inside the terrain.
        else:
            if (
                mesh_min_x < -terrain_half_x
                or mesh_max_x > terrain_half_x
                or mesh_min_north < -terrain_half_y
                or mesh_max_north > terrain_half_y
            ):
                skipped += 1
                continue

        # Map local metres to a lat/lon sample point.
        lat = terrain["center_lat"] + north_m / 111320.0
        lon = terrain["center_lon"] + x_m / (
            111320.0 * math.cos(math.radians(terrain["center_lat"]))
        )

        px = int((lon - west) / (east - west) * iw)
        py = int((north - lat) / (north - south) * ih)
        px = max(0, min(px, iw - 1))
        py = max(0, min(py, ih - 1))

        val = image.getpixel((px, py))
        ground = (val / PYX_RES) * terrain["size_z"] + terrain["pos_z"] - 0.1

        tr = n.get("translation", [0.0, 0.0, 0.0])
        n["translation"] = [tr[0], ground, tr[2]]
        translated += 1

    doc["_drone_rakshak_removed_nodes"] = len(removed)
    doc["_drone_rakshak_translated_nodes"] = translated
    doc["_drone_rakshak_skipped_nodes"] = skipped
    write_glb(chunks, ji, doc, out)
    log(f"OSM cleaned: removed={len(removed)}, translated={translated}, skipped={skipped}")

def add_model(world_root, name, uri, pose=None, collision=False, material=False):
    world = world_root.find("world")
    model = ET.Element("model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    if pose:
        ET.SubElement(model, "pose").text = pose
    link = ET.SubElement(model, "link", {"name": "link"})

    vis = ET.SubElement(link, "visual", {"name": "visual"})
    geom = ET.SubElement(vis, "geometry")
    mesh = ET.SubElement(geom, "mesh")
    ET.SubElement(mesh, "uri").text = uri
    if material:
        mat = ET.SubElement(vis, "material")
        ET.SubElement(mat, "ambient").text = "0.70 0.70 0.70 1"
        ET.SubElement(mat, "diffuse").text = "0.82 0.82 0.82 1"
        ET.SubElement(mat, "specular").text = "0.05 0.05 0.05 1"

    if collision:
        col = ET.SubElement(link, "collision", {"name": "collision"})
        cgeom = ET.SubElement(col, "geometry")
        cmesh = ET.SubElement(cgeom, "mesh")
        ET.SubElement(cmesh, "uri").text = uri

    world.append(model)

def remove_generated_building_model(world_root):
    world = world_root.find("world")
    for model in list(world.findall("model")):
        name = model.get("name", "").lower()
        if "buildings" in name:
            world.remove(model)

def gazebo_file_uri(path):
    from urllib.parse import quote

    resolved = str(Path(path).resolve())

    # Keep @ unescaped because Gazebo in this setup does not
    # resolve %40 correctly in local file URIs.
    return "file://" + quote(
        resolved,
        safe="/:@-._~!$&'()*+,;=@"
    )


def build_final_world(terrain_world, osm_glb, overture_dae, final_world):
    root = ET.parse(terrain_world).getroot()

    # Remove the terrain generator's original building model.
    # Keep only the Overture building layer that we add below.
    remove_generated_building_model(root)

    for parent in root.iter():
        for model in list(parent.findall("model")):
            name = model.get("name", "")

            if (
                name.endswith("_buildings")
                and name != "drone_rakshak_overture_buildings"
            ):
                parent.remove(model)
                log(f"Removed legacy building model: {name}")

    terrain_dir = Path(terrain_world).resolve().parent
    mesh_dir = terrain_dir / "mesh"

    # Gazebo resolves relative mesh/texture URIs relative to the final world.
    # Keep a mesh/ directory beside the generated world.
    final_mesh_dir = final_world.parent / "mesh"

    if final_mesh_dir.exists() or final_mesh_dir.is_symlink():
        if final_mesh_dir.is_symlink() or final_mesh_dir.is_file():
            final_mesh_dir.unlink()
        else:
            import shutil
            shutil.rmtree(final_mesh_dir)

    if mesh_dir.exists():
        try:
            final_mesh_dir.symlink_to(mesh_dir, target_is_directory=True)
        except OSError:
            import shutil
            shutil.copytree(mesh_dir, final_mesh_dir)

    # Rewrite the terrain generator's relative mesh URIs to absolute file URIs.
    uri_map = {
        "mesh/height_map.png":
            gazebo_file_uri(mesh_dir / "height_map.png"),

        "mesh/aerial.png":
            gazebo_file_uri(mesh_dir / "aerial.png"),

        "mesh/normal_map.png":
            gazebo_file_uri(mesh_dir / "normal_map.png"),

        "mesh/buildings.dae":
            gazebo_file_uri(mesh_dir / "buildings.dae"),
    }

    for uri in root.findall(".//uri"):
        value = (uri.text or "").strip()
        if value in uri_map:
            target = Path(uri_map[value].replace("file://", ""))
            if target.exists():
                uri.text = uri_map[value]
            else:
                # The terrain generator may legitimately have no buildings file.
                uri.text = value

    osm_uri = gazebo_file_uri(osm_glb)
    overture_uri = gazebo_file_uri(overture_dae)

    add_model(
        root,
        "drone_rakshak_osm_objects",
        osm_uri,
        pose="0 0 0 1.57079632679 0 0",
        collision=False,
    )
    terrain = parse_sdf(terrain_world)
    add_model(
        root,
        "drone_rakshak_overture_buildings",
        overture_uri,
        pose=f"0 {terrain['pos_y']} 0 0 0 0",
        collision=True,
        material=False,
    )

    # Convert any remaining relative terrain asset paths to absolute file:// URIs.
    # This includes aerial.png, normal_map.png, height_map.png, buildings.dae, etc.
    for element in root.iter():
        if element.text:
            value = element.text.strip()
            if value.startswith("mesh/"):
                element.text = gazebo_file_uri(terrain_dir / value)

    ET.indent(root, space="  ")
    final_world.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(final_world, encoding="utf-8", xml_declaration=True)
    return final_world

def discover_ros_exe(exe_name):
    try:
        out = subprocess.check_output(["bash", "-lc", "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source ~/drone_ws/install/setup.bash >/dev/null 2>&1; ros2 pkg executables"], text=True)
    except Exception:
        return None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1] == exe_name:
            return parts[0], parts[1]
    return None

def start_ros_and_qgc(logdir):
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    # Micro XRCE Agent
    if shutil.which("MicroXRCEAgent") and subprocess.call(["bash", "-lc", "pgrep -f 'MicroXRCEAgent udp4 -p 8888' >/dev/null 2>&1"]) != 0:
        start_bg(["MicroXRCEAgent", "udp4", "-p", "8888"], env=env, logfile=logdir / "microxrce.log")

    # Launch known Drone Rakshak executables when discoverable.
    for exe in ["radar_receiver", "radar_fusion", "obstacle_detector", "avoidance_controller", "offboard_controller"]:
        found = discover_ros_exe(exe)
        if not found:
            continue
        pkg, ex = found
        cmd = [
            "bash", "-lc",
            f"source /opt/ros/jazzy/setup.bash && source ~/drone_ws/install/setup.bash && exec ros2 run {pkg} {ex}"
        ]
        start_bg(cmd, env=env, logfile=logdir / f"{exe}.log")
        time.sleep(1)

    qgc = HOME / "Downloads" / "QGroundControl-x86_64.AppImage"
    if qgc.exists() and subprocess.call(["bash", "-lc", "pgrep -f 'QGroundControl-x86_64.AppImage' >/dev/null 2>&1"]) != 0:
        start_bg([str(qgc)], cwd=qgc.parent, logfile=logdir / "qgroundcontrol.log")

def stop_sim():
    """Stop only the current Drone Rakshak simulation cleanly."""

    patterns = [
        "gz sim",
        "gz-gui",
        "make px4_sitl gz_x500",
        "build/px4_sitl_default/bin/px4",
    ]

    for pattern in patterns:
        subprocess.run(
            ["bash", "-lc", f"pkill -TERM -f '{pattern}' || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # Give Gazebo / PX4 time to release ports and graphics resources.
    deadline = time.time() + 6

    while time.time() < deadline:
        alive = False

        for pattern in patterns:
            rc = subprocess.run(
                ["bash", "-lc", f"pgrep -f '{pattern}' >/dev/null 2>&1"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode

            if rc == 0:
                alive = True
                break

        if not alive:
            break

        time.sleep(0.5)

    # Hard cleanup if anything survived.
    for pattern in patterns:
        subprocess.run(
            ["bash", "-lc", f"pkill -KILL -f '{pattern}' || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    time.sleep(2)


def start_sim(final_world, world_workdir):
    logdir = world_workdir / "logs"
    logdir.mkdir(parents=True, exist_ok=True)

    # --------------------------------------------------------
    # Gazebo server
    # --------------------------------------------------------
    server_env = os.environ.copy()
    server_env["QT_OPENGL"] = "software"
    server_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    server_env["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"

    server = start_bg(
        [
            "gz",
            "sim",
            "-s",
            "-r",
            "-v",
            "4",
            str(final_world),
        ],
        env=server_env,
        logfile=logdir / "gz_server.log",
    )

    # Wait for server startup and make sure it did not immediately die.
    for _ in range(20):
        time.sleep(0.5)

        if server.poll() is not None:
            raise RuntimeError(
                "Gazebo server exited while loading the world. "
                f"See {logdir / 'gz_server.log'}"
            )

    # --------------------------------------------------------
    # Gazebo GUI
    #
    # Important:
    #   GUI uses XCB/WSLg.
    #   Do NOT force llvmpipe here.
    # --------------------------------------------------------
    gui_env = os.environ.copy()
    gui_env.pop("WAYLAND_DISPLAY", None)
    gui_env["QT_QPA_PLATFORM"] = "xcb"

    gui = start_bg(
        [
            "gz",
            "sim",
            "-g",
            "-v",
            "4",
        ],
        env=gui_env,
        logfile=logdir / "gz_gui.log",
    )

    if gui is None:
        raise RuntimeError(
            "Could not start Gazebo GUI."
        )

    time.sleep(2)

    # --------------------------------------------------------
    # PX4
    # --------------------------------------------------------
    px4_env = os.environ.copy()
    px4_env["PX4_GZ_STANDALONE"] = "1"
    px4_env["PX4_SIM_MODEL"] = "gz_x500"
    px4_env["PX4_SYS_AUTOSTART"] = "4001"
    px4_env["PX4_GZ_MODEL_POSE"] = "0,0,10,0,0,0"

    start_bg(
        [
            "env",
            "PX4_GZ_STANDALONE=1",
            "make",
            "px4_sitl",
            "gz_x500",
        ],
        cwd=HOME / "PX4-Autopilot",
        env=px4_env,
        logfile=logdir / "px4.log",
    )

    time.sleep(5)

    start_ros_and_qgc(logdir)

    log(f"Simulation started. World: {final_world}")
    log(f"Logs: {logdir}")

    return server


def process_world(world_path):
    terrain = parse_sdf(world_path)
    bbox = bbox_from_center_size(
        terrain["center_lat"], terrain["center_lon"],
        terrain["size_x"], terrain["size_y"],
        margin_m=0,
    )

    WORK_ROOT.mkdir(parents=True, exist_ok=True)

    # Sanitize generated-world directory names for Gazebo.
    import re

    safe_name = re.sub(
        r"[^A-Za-z0-9_.-]+",
        "_",
        world_path.parent.name
    ).strip("_") or "world"

    map_work = WORK_ROOT / safe_name
    map_work.mkdir(parents=True, exist_ok=True)

    # Rebuild derived assets whenever the source terrain world changes.
    # This handles generating a new terrain using the same folder/name.
    source_sig = (
        world_path.stat().st_mtime_ns,
        world_path.stat().st_size,
    )

    state_file = map_work / ".terrain_source_signature"

    old_sig = None

    if state_file.exists():
        try:
            old_sig = tuple(
                int(x)
                for x in state_file.read_text().split(",")
            )
        except Exception:
            old_sig = None

    if old_sig != source_sig:
        for name in (
            "osm.osm",
            "osm_clipped.osm",
            "osm_roads_clipped.osm",
            "osm_raw.glb",
            "osm_objects_draped.glb",
            "overture_buildings.geojson",
            "overture_buildings_normalized.geojson",
            "overture_places.geojson",
            "overture_buildings.obj",
            "overture_buildings.mtl",
            "building_texture_manifest.json",
        ):
            target = map_work / name
            if target.exists():
                target.unlink()

        texture_dir = map_work / "building_textures"

        if texture_dir.exists():
            shutil.rmtree(texture_dir)

        for name in (
            f"{world_path.stem}_drone_rakshak.world",
        ):
            target = map_work / name
            if target.exists():
                target.unlink()

        state_file.write_text(
            f"{source_sig[0]},{source_sig[1]}"
        )

        log(
            "Terrain source changed: rebuilt derived map assets."
        )

    osm_file = map_work / "osm.osm"
    clipped_osm_file = map_work / "osm_clipped.osm"
    roads_osm_file = map_work / "osm_roads_clipped.osm"
    raw_osm_glb = map_work / "osm_raw.glb"
    clean_osm_glb = map_work / "osm_objects_draped.glb"
    overture_raw = map_work / "overture_buildings.geojson"
    overture_norm = map_work / "overture_buildings_normalized.geojson"
    overture_places = map_work / "overture_places.geojson"
    overture_dae = map_work / "overture_buildings.obj"
    final_world = map_work / f"{world_path.stem}_drone_rakshak.world"

    log(f"Center: {terrain['center_lat']:.8f}, {terrain['center_lon']:.8f}")
    log(f"Terrain size: {terrain['size_x']:.2f} x {terrain['size_y']:.2f} m")
    log(f"BBOX: {bbox_str(bbox)}")

    if not osm_file.exists():
        download_osm(bbox, osm_file)

    # Crop OSM data to the selected terrain before OSM2World conversion.
    # This prevents long roads, railway ways, power infrastructure and
    # other geographic features from extending far outside the terrain.
    if not clipped_osm_file.exists():
        run(
            [
                "osmium",
                "extract",
                "--strategy=complete_ways",
                f"--bbox={bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]}",
                str(osm_file),
                "-o",
                str(clipped_osm_file),
            ],
            check=True,
        )

    # Build the road-only OSM after the clipped source exists.
    clip_osm_roads_to_bbox(
        clipped_osm_file,
        roads_osm_file,
        bbox
    )

    if not raw_osm_glb.exists():
        run(
            [
                "./osm2world.sh",
                "--input", str(roads_osm_file),
                "--input_mode", "FILE",
                "--lod", "2",
                "--output", str(raw_osm_glb),
            ],
            cwd=OSM2WORLD_ROOT,
            check=False,
        )
    if not raw_osm_glb.exists():
        raise RuntimeError("OSM2World did not produce a GLB.")

    if not clean_osm_glb.exists():
        drape_osm_glb(raw_osm_glb, clean_osm_glb, terrain, roads_osm_file)

    if not overture_raw.exists():
        run(
            [
                "uv", "run", "--with", "overturemaps",
                "overturemaps", "download",
                f"--bbox={bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]}",
                "-f", "geojson",
                "--type=building",
                "-o", str(overture_raw),
            ],
            cwd=TERRAIN_ROOT,
            check=True,
        )

    if not overture_norm.exists():
        count = write_overture_properties(overture_raw, overture_norm)
        log(f"Overture buildings: {count}")

    # Download Overture Places for the exact selected area.
    if not overture_places.exists():
        run(
            [
                "uv", "run", "--with", "overturemaps",
                "overturemaps", "download",
                f"--bbox={bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]}",
                "-f", "geojson",
                "--type=place",
                "-o", str(overture_places),
            ],
            cwd=TERRAIN_ROOT,
            check=True,
        )

    if not overture_dae.exists():
        build_overture_glb(
            overture_norm,
            overture_dae,
            terrain,
            osm_file,
            overture_places,
        )

    if not final_world.exists():
        build_final_world(world_path, clean_osm_glb, overture_dae, final_world)

    return final_world

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-px4", action="store_true")
    ap.add_argument("--restart", action="store_true")
    args = ap.parse_args()

    WORK_ROOT.mkdir(
        parents=True,
        exist_ok=True
    )

    if args.restart:
        stop_sim()

    ensure_terrain_server()
    open_browser()

    # Snapshot BEFORE waiting for generated terrain.
    before = snapshot_worlds()

    current_path = None
    current_sig = None

    log(
        "Waiting for terrain. "
        "Generate locations repeatedly; "
        "this launcher stays running."
    )

    try:
        while True:

            # ------------------------------------------------
            # Wait for ANY new or modified generated world.
            # ------------------------------------------------
            world = wait_for_new_world(
                before
            )

            # Read its current signature.
            current_snapshot = snapshot_worlds()
            detected_sig = current_snapshot.get(
                str(world)
            )

            # Prevent duplicate processing of the exact same file
            # signature.
            if (
                current_path == world.resolve()
                and current_sig == detected_sig
            ):
                before = current_snapshot
                continue

            log(
                f"Building Drone Rakshak environment "
                f"for {world.parent.name}"
            )

            try:
                final_world = process_world(
                    world
                )

            except Exception as e:
                log(
                    f"PIPELINE FAILED: {e}"
                )

                log(
                    "Terrain generator remains running. "
                    "Generate another terrain after fixing the issue."
                )

                # Important: refresh snapshot so the same failed
                # world does not endlessly trigger again.
                before = snapshot_worlds()
                continue

            # ------------------------------------------------
            # Switch simulation.
            # ------------------------------------------------
            stop_sim()

            final_work = (
                WORK_ROOT /
                re.sub(
                    r"[^A-Za-z0-9_.-]+",
                    "_",
                    world.parent.name
                ).strip("_")
            )

            if args.no_px4:
                log(
                    "Launching Gazebo only (--no-px4)."
                )

                server_env = os.environ.copy()
                server_env["QT_OPENGL"] = "software"
                server_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
                server_env[
                    "MESA_LOADER_DRIVER_OVERRIDE"
                ] = "llvmpipe"

                server = start_bg(
                    [
                        "gz",
                        "sim",
                        "-s",
                        "-r",
                        "-v",
                        "4",
                        str(final_world),
                    ],
                    logfile=(
                        final_work /
                        "logs" /
                        "gz_server.log"
                    ),
                    env=server_env,
                )

                time.sleep(6)

                if server.poll() is not None:
                    raise RuntimeError(
                        "Gazebo server exited during startup."
                    )

                gui_env = os.environ.copy()
                gui_env.pop(
                    "WAYLAND_DISPLAY",
                    None
                )
                gui_env[
                    "QT_QPA_PLATFORM"
                ] = "xcb"

                start_bg(
                    [
                        "gz",
                        "sim",
                        "-g",
                        "-v",
                        "4",
                    ],
                    logfile=(
                        final_work /
                        "logs" /
                        "gz_gui.log"
                    ),
                    env=gui_env,
                )

            else:
                start_sim(
                    final_world,
                    final_work
                )

            current_path = world.resolve()

            # Refresh the signature AFTER processing.
            after = snapshot_worlds()
            current_sig = after.get(
                str(world)
            )

            # This is the critical part:
            # the next loop immediately watches for a DIFFERENT
            # generated terrain, without restarting this launcher.
            before = after

            log(
                "Ready. Generate another terrain in the browser; "
                "the current simulation will be stopped and the "
                "new terrain will automatically become active."
            )

    except KeyboardInterrupt:
        log(
            "Stopping Drone Rakshak launcher."
        )

        stop_sim()



if __name__ == "__main__":
    main()
