#!/usr/bin/env python

import os
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np


class Utils:
    @staticmethod
    def make_quad_key(tile_x, tile_y, level):
        quadkey = ""
        for i in range(level):
            bit = level - i
            digit = ord('0')
            mask = 1 << (bit - 1)
            if (tile_x & mask) != 0:
                digit += 1
            if (tile_y & mask) != 0:
                digit += 2
            quadkey += chr(digit)
        return quadkey

    @staticmethod
    def qualify_url(url, x, y, z, api_key=''):
        replace_map = {
            "x": str(x),
            "y": str(y),
            "z": str(z),
            "quad": Utils.make_quad_key(x, y, z),
            "key": api_key,
        }
        for k, value in replace_map.items():
            url = url.replace(f"{{{k}}}", value)
        return url

    @staticmethod
    def download_file(url, destination, x, y, z, api_key=''):
        """
        Download one satellite tile.

        The requested source is tried first. If it fails, Esri World
        Imagery is used as a fallback. Failed requests are retried.
        Only a successful image response is written to disk.
        """

        primary = Utils.qualify_url(url, x, y, z, api_key)

        fallback = (
            "https://server.arcgisonline.com/"
            "ArcGIS/rest/services/World_Imagery/MapServer/"
            f"tile/{z}/{y}/{x}"
        )

        candidates = []

        if primary:
            candidates.append(primary)

        if fallback not in candidates:
            candidates.append(fallback)

        headers = {
            "User-Agent": "DroneRakshakTerrain/1.0",
            "Accept": "image/png,image/jpeg,image/webp,*/*",
            "Connection": "close",
        }

        last_code = -1

        for candidate in candidates:
            for attempt in range(3):
                tmp = destination + f".part-{os.getpid()}"

                try:
                    req = urllib.request.Request(
                        candidate,
                        headers=headers,
                    )

                    with urllib.request.urlopen(req, timeout=20) as resp:
                        code = getattr(resp, "status", 200)
                        content_type = (
                            resp.headers.get("Content-Type", "")
                            .lower()
                        )
                        data = resp.read()

                    # Reject HTML/error pages masquerading as successful
                    # HTTP responses.
                    if code != 200:
                        last_code = code
                        raise RuntimeError(
                            f"HTTP {code}"
                        )

                    if len(data) <= 100:
                        raise RuntimeError(
                            f"response too small ({len(data)} bytes)"
                        )

                    if "text/html" in content_type:
                        raise RuntimeError(
                            f"HTML response ({content_type})"
                        )

                    # Validate that OpenCV can actually decode the tile.
                    arr = np.frombuffer(data, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                    if img is None:
                        raise RuntimeError(
                            "response is not a valid image"
                        )

                    with open(tmp, "wb") as f:
                        f.write(data)

                    os.replace(tmp, destination)

                    print(
                        f"Tile downloaded: "
                        f"x={x} y={y} z={z} "
                        f"source={'fallback' if candidate == fallback else 'primary'}"
                    )

                    return 200

                except Exception as e:
                    last_code = getattr(e, "code", last_code)

                    print(
                        f"Tile attempt failed: "
                        f"x={x} y={y} z={z} "
                        f"attempt={attempt + 1}/3: {e}"
                    )

                    if attempt < 2:
                        time.sleep(1.5 * (attempt + 1))

                finally:
                    try:
                        if os.path.exists(tmp):
                            os.remove(tmp)
                    except OSError:
                        pass

        print(
            f"Tile download FAILED permanently: "
            f"x={x} y={y} z={z}"
        )

        return last_code

class ConcatImage:
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @staticmethod
    def stitch_flat_tiles(tiles_dir: str, zoom_level: int, missing_fill=None):
        """
        Stitch flat tile files ([zoom,y,x].png) from tiles_dir into a single image.

        Tiles at zoom levels other than zoom_level are ignored.
        Missing tiles within the bounding rectangle are filled with missing_fill
        (a pre-built numpy array matching one tile's shape), or omitted if None.

        Args:
            tiles_dir (str): Directory containing flat [zoom,y,x].png tile files.
            zoom_level (int): Zoom level to filter tiles by.
            missing_fill: Optional numpy array used to fill gaps (e.g. gray tile).

        Returns:
            tuple: (stitched_image, tile_map, x_min, x_max, y_min, y_max)
                   stitched_image — cv2 numpy array of the combined image
                   tile_map       — dict mapping (x, y) → file path
                   x/y min/max    — bounding tile coordinate extents
        """
        tile_map = {}
        for fname in os.listdir(tiles_dir):
            if not fname.endswith('.png'):
                continue
            parts = fname[1:-5].split(',')  # strip '[' and '].png'
            if len(parts) != 3:
                continue
            try:
                a, b, c = (int(parts[0]), int(parts[1]), int(parts[2]))
            except ValueError:
                continue
            # Accept both [z,y,x].png and [x,y,z].png.
            if zoom_level is not None and a == zoom_level:
                z, y, x = a, b, c
            elif zoom_level is not None and c == zoom_level:
                x, y, z = a, b, c
            elif zoom_level is None:
                if c <= 30 and a > 30:
                    x, y, z = a, b, c
                else:
                    z, y, x = a, b, c
            else:
                continue
            tile_map[(x, y)] = os.path.join(tiles_dir, fname)
        if not tile_map:
            pngs = [f for f in os.listdir(tiles_dir) if f.lower().endswith('.png')]
            raise RuntimeError(f"No satellite tiles matched zoom {zoom_level}. Found {len(pngs)} PNG tile(s). The downloader/stitcher filename convention may be incompatible.")

        x_min = min(x for x, y in tile_map)
        x_max = max(x for x, y in tile_map)
        y_min = min(y for x, y in tile_map)
        y_max = max(y for x, y in tile_map)

        # Read one tile to get dimensions
        sample_img = cv2.imread(next(iter(tile_map.values())))
        tile_h, tile_w = sample_img.shape[:2]
        n_cols = x_max - x_min + 1
        n_rows = y_max - y_min + 1

        # Pre-allocate the full output buffer. np.full with a scalar is a single C memset —
        # no temporary copies. Gray (128) fills missing tiles with no extra logic needed.
        fill_value = missing_fill[0, 0, 0] if missing_fill is not None else 0
        stitched_image = np.full((n_rows * tile_h, n_cols * tile_w, 3), fill_value, dtype=np.uint8)

        # Each tile writes to a non-overlapping region so parallel reads are thread-safe.
        # ThreadPoolExecutor overlaps disk I/O across cores — the main bottleneck at scale.
        def place_tile(args):
            (x, y), path = args
            img = cv2.imread(path)
            if img is None:
                return
            row = y - y_min
            col = x - x_min
            stitched_image[row * tile_h:(row + 1) * tile_h, col * tile_w:(col + 1) * tile_w] = img

        with ThreadPoolExecutor() as executor:
            executor.map(place_tile, tile_map.items())

        return stitched_image, tile_map, x_min, x_max, y_min, y_max
