from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import requests


OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]


def build_query(lat: float, lon: float, radius_m: float) -> str:
    # Keep this query focused on geometry needed by the Gazebo MVP.
    r = int(radius_m)
    return f"""
[out:json][timeout:120];
(
  way["building"](around:{r},{lat},{lon});
  way["highway"](around:{r},{lat},{lon});
  way["natural"="water"](around:{r},{lat},{lon});
  way["natural"="wood"](around:{r},{lat},{lon});
  way["landuse"="forest"](around:{r},{lat},{lon});
  way["landuse"="grass"](around:{r},{lat},{lon});
  way["leisure"="park"](around:{r},{lat},{lon});
);
out body;
>;
out skel qt;
"""


def download_osm(lat: float, lon: float, radius_m: float, cache_file: Path) -> dict[str, Any]:
    cache_file.parent.mkdir(parents=True, exist_ok=True)

    query = build_query(lat, lon, radius_m)
    headers = {
        "User-Agent": "DroneRakshakMapManager/0.1 (educational simulation project)"
    }

    errors: list[str] = []
    for endpoint in OVERPASS_ENDPOINTS:
        try:
            response = requests.post(
                endpoint,
                data={"data": query},
                headers=headers,
                timeout=(15, 150),
            )
            response.raise_for_status()
            data = response.json()
            cache_file.write_text(json.dumps(data), encoding="utf-8")
            return data
        except Exception as exc:
            errors.append(f"{endpoint}: {exc}")
            time.sleep(1.0)

    raise RuntimeError(
        "All Overpass endpoints failed. Errors:\n" + "\n".join(errors)
    )


def load_or_download(
    lat: float,
    lon: float,
    radius_m: float,
    cache_dir: Path,
    refresh: bool = False,
) -> dict[str, Any]:
    key = f"{lat:.6f}_{lon:.6f}_{int(radius_m)}".replace("-", "m")
    cache_file = cache_dir / f"osm_{key}.json"

    if cache_file.exists() and not refresh:
        return json.loads(cache_file.read_text(encoding="utf-8"))

    return download_osm(lat, lon, radius_m, cache_file)
