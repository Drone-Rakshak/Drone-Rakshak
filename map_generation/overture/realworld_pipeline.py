#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import shutil
import shlex
import signal
import subprocess
import sys
import time
import threading
import gc
import pty
import select
import termios
import tty
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import Request, urlopen

# The textured Overture exporter needs trimesh/numpy/scipy and related
# packages.  Keep the normal Drone Rakshak workflow unchanged: if the
# system Python does not provide them, transparently re-run this same
# script inside a temporary isolated uv environment.  No venv activation
# or system-wide pip installation is required.
if os.environ.get("DRONE_RAKSHAK_UV_TEXTURE_ENV") != "1":
    try:
        import importlib.util
        _texture_deps_ready = (
            importlib.util.find_spec("trimesh") is not None
            and importlib.util.find_spec("numpy") is not None
            and importlib.util.find_spec("PIL") is not None
            and importlib.util.find_spec("shapely") is not None
            and importlib.util.find_spec("pyproj") is not None
        )
    except Exception:
        _texture_deps_ready = False

    if not _texture_deps_ready:
        _uv = shutil.which("uv")
        if not _uv:
            raise RuntimeError(
                "Python texture dependencies are missing and uv was not found. "
                "Install uv or run the script from an environment containing "
                "trimesh, numpy, pillow, shapely and pyproj."
            )

        _env = os.environ.copy()
        _user_site = str(Path.home() / ".local" / "lib" / "python3.12" / "site-packages")
        _env["PYTHONPATH"] = _user_site + (os.pathsep + _env["PYTHONPATH"] if _env.get("PYTHONPATH") else "")
        _env["DRONE_RAKSHAK_UV_TEXTURE_ENV"] = "1"
        _cmd = [
            _uv, "run", "--isolated",
            "--with", "numpy<2",
            "--with", "scipy",
            "--with", "trimesh",
            "--with", "pillow",
            "--with", "shapely",
            "--with", "pyproj",
            "python3", str(Path(__file__).resolve()),
            *sys.argv[1:],
        ]
        os.execvpe(_uv, _cmd, _env)

HOME = Path.home()
_USER_SITE = HOME / ".local" / "lib" / "python3.12" / "site-packages"
os.environ["PYTHONPATH"] = str(_USER_SITE) + (os.pathsep + os.environ["PYTHONPATH"] if os.environ.get("PYTHONPATH") else "")
TERRAIN_ROOT = HOME / "gazebo_terrain_generator"
GENERATED_ROOT = HOME / "drone_ws" / "generated_maps"
OSM2WORLD_ROOT = HOME / "OSM2World"
WORK_ROOT = HOME / "drone_ws" / "drone_rakshak_worlds"
PORT = 8081

# Processes started by this pipeline. Keep explicit handles instead of broad
# pkill -f patterns so generating map N+1 cannot kill the launcher or leave
# map N processes consuming RAM.
ACTIVE_PROCS = []
SHUTDOWN_EVENT = threading.Event()
MAX_SATELLITE_ZOOM = 16
MAX_HEIGHTMAP_SIZE = 1025

PYX_RES = 65535

# Overall Overture building scale.  The building center stays fixed, so
# enlarging buildings does not move them away from their real-world location.
BUILDING_SCALE_X = 1.00   # keep original footprint width/east-west
BUILDING_SCALE_Y = 1.00   # keep original footprint depth/north-south
BUILDING_SCALE_Z = 2.00   # double building height

def log(msg):
    print(f"[DR] {msg}", flush=True)

def run(cmd, cwd=None, env=None, check=False):
    log("$ " + " ".join(map(str, cmd)))
    return subprocess.run(cmd, cwd=cwd, env=env, check=check)

def _register_proc(proc):
    try:
        ACTIVE_PROCS.append(proc)
    except Exception:
        pass
    return proc


def start_bg(cmd, cwd=None, env=None, logfile=None, show_output=False, interactive=False):
    log("START " + " ".join(map(str, cmd)))

    if interactive:
        logfile.parent.mkdir(parents=True, exist_ok=True) if logfile else None
        pid, master_fd = pty.fork()
        if pid == 0:
            if cwd:
                os.chdir(cwd)
            child_env = os.environ.copy()
            if env:
                child_env.update(env)
            os.execvpe(cmd[0], cmd, child_env)
            os._exit(127)

        class PtyProcess:
            def __init__(self, pid, master_fd):
                self.pid = pid
                self.master_fd = master_fd
                self.returncode = None

            def poll(self):
                if self.returncode is not None:
                    return self.returncode
                try:
                    rpid, status = os.waitpid(self.pid, os.WNOHANG)
                    if rpid == 0:
                        return None
                    self.returncode = os.waitstatus_to_exitcode(status)
                    return self.returncode
                except ChildProcessError:
                    return self.returncode if self.returncode is not None else 0

            def terminate(self):
                try:
                    os.killpg(self.pid, signal.SIGTERM)
                except ProcessLookupError:
                    try:
                        os.kill(self.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass

            def kill(self):
                try:
                    os.killpg(self.pid, signal.SIGKILL)
                except ProcessLookupError:
                    try:
                        os.kill(self.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

        proc = PtyProcess(pid, master_fd)
        _register_proc(proc)

        def _interactive_pty():
            old_attrs = None
            try:
                if sys.stdin.isatty():
                    old_attrs = termios.tcgetattr(sys.stdin.fileno())
                    tty.setraw(sys.stdin.fileno())
                log_file = open(logfile, "ab", buffering=0) if logfile else None
                try:
                    while True:
                        if proc.poll() is not None:
                            break
                        readable, _, _ = select.select(
                            [master_fd, sys.stdin.fileno()] if sys.stdin.isatty() else [master_fd],
                            [], [], 0.2
                        )
                        if master_fd in readable:
                            try:
                                data = os.read(master_fd, 4096)
                            except OSError:
                                break
                            if not data:
                                break
                            if log_file:
                                log_file.write(data)
                            os.write(sys.stdout.fileno(), data)
                        if sys.stdin.isatty() and sys.stdin.fileno() in readable:
                            try:
                                data = os.read(sys.stdin.fileno(), 4096)
                            except OSError:
                                break
                            if data:
                                if b"\x03" in data:
                                    SHUTDOWN_EVENT.set()
                                    log("Ctrl+C received: stopping Drone Rakshak cleanly...")
                                    try:
                                        stop_tracked_processes(force=False)
                                    except Exception:
                                        pass
                                    break
                                os.write(master_fd, data)
                finally:
                    if log_file:
                        log_file.close()
            except Exception as exc:
                log(f"PX4 interactive console stopped: {exc}")
            finally:
                if old_attrs is not None:
                    try:
                        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_attrs)
                    except Exception:
                        pass

        threading.Thread(target=_interactive_pty, daemon=True).start()
        return proc

    popen_kwargs = dict(cwd=cwd, env=env)
    # A separate process group lets us stop a ROS/Gazebo command and its
    # children together without matching unrelated processes.
    popen_kwargs["start_new_session"] = True

    if logfile and show_output:
        logfile.parent.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            bufsize=1, text=True, **popen_kwargs
        )
        def _forward_output():
            try:
                with open(logfile, "a", buffering=1) as f:
                    for line in proc.stdout:
                        f.write(line); f.flush(); print(line, end="", flush=True)
            except Exception as exc:
                log(f"Output forwarding stopped: {exc}")
        threading.Thread(target=_forward_output, daemon=True).start()
        return _register_proc(proc)

    if logfile:
        logfile.parent.mkdir(parents=True, exist_ok=True)
        f = open(logfile, "ab", buffering=0)
        proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, **popen_kwargs)
        return _register_proc(proc)

    return _register_proc(subprocess.Popen(cmd, **popen_kwargs))


def stop_tracked_processes(force=False):
    """Stop only processes started by this pipeline run."""
    alive = []
    for proc in list(ACTIVE_PROCS):
        try:
            if proc.poll() is None:
                alive.append(proc)
        except Exception:
            pass
    if not alive:
        ACTIVE_PROCS.clear()
        return

    sig_name = "KILL" if force else "TERM"
    log(f"Stopping {len(alive)} tracked simulation process(es) ({sig_name})...")
    for proc in alive:
        try:
            (proc.kill() if force else proc.terminate())
        except Exception:
            pass

    if not force:
        deadline = time.time() + 4
        while time.time() < deadline:
            remaining = []
            for proc in alive:
                try:
                    if proc.poll() is None:
                        remaining.append(proc)
                except Exception:
                    pass
            if not remaining:
                break
            time.sleep(0.2)
        for proc in remaining if 'remaining' in locals() else []:
            try:
                proc.kill()
            except Exception:
                pass
    ACTIVE_PROCS.clear()


def kill_matching(patterns):
    # Retained for compatibility with older code; intentionally avoids pkill.
    return

def patch_terrain_generator_for_memory():
    """Patch the terrain generator for reliable tiles and bounded RAM."""
    server = TERRAIN_ROOT / "scripts" / "server.py"
    utils = TERRAIN_ROOT / "scripts" / "utils" / "utils.py"
    changed = False
    if server.exists():
        txt = server.read_text(errors="ignore")
        original = txt
        txt = re.sub(
            r"(?m)^\s*x = int\(postvars\['x'\]\)\n\s*y = int\(postvars\['y'\]\)\n\s*zoom = int\(postvars\['z'\]\)",
            "    requested_zoom = int(postvars['z'])\n    zoom = min(requested_zoom, MAX_SATELLITE_ZOOM)\n    x = int(postvars['x'])\n    y = int(postvars['y'])\n    if requested_zoom > zoom:\n        factor = 2 ** (requested_zoom - zoom)\n        x //= factor\n        y //= factor",
            txt, count=1)
        txt = re.sub(r"(?m)^(\s*)zoom_level = int\(postvars\['maxZoom'\]\)", r"\1zoom_level = min(int(postvars['maxZoom']), MAX_SATELLITE_ZOOM)", txt)
        txt = re.sub(r"(?m)^(\s*)zoom_level = int\(postvars\['zoomLevel'\]\)", r"\1zoom_level = min(int(postvars['zoomLevel']), MAX_SATELLITE_ZOOM)", txt)
        txt = txt.replace("target_heightmap_size = compute_auto_heightmap_size(bounds, dem_resolution)", "target_heightmap_size = min(compute_auto_heightmap_size(bounds, dem_resolution), MAX_HEIGHTMAP_SIZE)")
        txt = txt.replace("target_heightmap_size = int(target_heightmap_size_raw)", "target_heightmap_size = min(int(target_heightmap_size_raw), MAX_HEIGHTMAP_SIZE)")
        if 'MAX_SATELLITE_ZOOM = 16' not in txt and 'app = Flask(__name__)' in txt:
            txt=txt.replace('app = Flask(__name__)', 'MAX_SATELLITE_ZOOM = 16\nMAX_HEIGHTMAP_SIZE = 1025\n\napp = Flask(__name__)',1)
        if txt!=original:
            server.write_text(txt); changed=True
    if utils.exists():
        txt=utils.read_text(errors="ignore")
        original=txt
        pat=re.compile(r"    @staticmethod\n    def download_file\(url, destination, x, y, z, api_key=''\):\n.*?(?=\n    class ConcatImage:)",re.S)
        repl="""    @staticmethod
    def download_file(url, destination, x, y, z, api_key=''):
        import time
        primary = Utils.qualify_url(url, x, y, z, api_key)
        fallback = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        candidates = []
        for candidate in (primary, fallback):
            if candidate and candidate not in candidates:
                candidates.append(candidate)
        for candidate in candidates:
            for attempt in range(3):
                try:
                    req=urllib.request.Request(candidate,headers={'User-Agent':'DroneRakshakTerrain/1.0','Accept':'image/png,image/jpeg,*/*','Connection':'close'})
                    with urllib.request.urlopen(req, timeout=20) as resp:
                        data=resp.read()
                    if len(data)>1000:
                        with open(destination,'wb') as f: f.write(data)
                        return 200
                except Exception as exc:
                    if attempt==2: print(f"Tile download failed: {candidate}: {exc}")
                time.sleep(0.75*(attempt+1))
        try:
            import cv2, numpy as np
            cv2.imwrite(destination, np.full((256,256,3),128,dtype=np.uint8))
            return 200
        except Exception:
            return -1

"""
        txt,n=pat.subn(repl,txt,count=1)
        if n: changed=True
        else:
            marker="\n    class ConcatImage:"
            if marker in txt:
                txt=txt.replace(marker,"\n"+repl+marker,1)
                changed=True
        old="""            parts = fname[1:-5].split(',')  # strip '[' and '].png'
            z, y, x = int(parts[0]), int(parts[1]), int(parts[2])
            if zoom_level is not None and z != zoom_level:
                continue
            tile_map[(x, y)] = os.path.join(tiles_dir, fname)
"""
        new="""            parts = fname[1:-5].split(',')
            if len(parts) != 3:
                continue
            try:
                a,b,c=(int(parts[0]),int(parts[1]),int(parts[2]))
            except ValueError:
                continue
            if zoom_level is not None and a == zoom_level:
                z,y,x=a,b,c
            elif zoom_level is not None and c == zoom_level:
                x,y,z=a,b,c
            else:
                continue
            tile_map[(x,y)] = os.path.join(tiles_dir,fname)
"""
        if old in txt:
            txt=txt.replace(old,new,1); changed=True
        if txt!=original:
            utils.write_text(txt); changed=True
    log("Terrain generator patches installed: zoom<=16, heightmap<=1025, retries/fallback.")
    return changed

def ensure_terrain_server():
    terrain_changed = patch_terrain_generator_for_memory()
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
        if terrain_changed:
            log("Restarting terrain generator so the memory guard takes effect...")
            subprocess.run(
                ["bash", "-lc", "pkill -TERM -f 'scripts/server.py' || true"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            time.sleep(2)
            probe = subprocess.run(
                ["bash", "-lc", "ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(:|\\])8081$'"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
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
        if SHUTDOWN_EVENT.is_set():
            return None
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

def _ensure_esri_satellite_texture(terrain):
    from io import BytesIO
    from PIL import Image
    mesh_dir=Path(terrain["heightmap_path"]).parent
    aerial=mesh_dir/"aerial.png"
    south,west,north,east=bbox_from_center_size(terrain["center_lat"],terrain["center_lon"],terrain["size_x"],terrain["size_y"],margin_m=0)
    z=MAX_SATELLITE_ZOOM; tile_px=256; n=2**z
    def xy(lat,lon):
        lat=max(-85.05112878,min(85.05112878,lat)); xr=(lon+180)/360*n; lr=math.radians(lat); yr=(1-math.asinh(math.tan(lr))/math.pi)/2*n; return int(xr),int(yr)
    x0,y1=xy(south,west); x1,y0=xy(north,east)
    if x1<x0:x0,x1=x1,x0
    if y1<y0:y0,y1=y1,y0
    if (x1-x0+1)*(y1-y0+1)>25:
        cx,cy=xy(terrain["center_lat"],terrain["center_lon"]); x0,x1=cx-1,cx+1; y0,y1=cy-1,cy+1
    mosaic=Image.new("RGB",((x1-x0+1)*tile_px,(y1-y0+1)*tile_px)); good=0
    for ty in range(y0,y1+1):
        for tx in range(x0,x1+1):
            url=f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{ty}/{tx}"
            try:
                req=Request(url,headers={"User-Agent":"DroneRakshak/1.0"})
                with urlopen(req,timeout=20) as r: img=Image.open(BytesIO(r.read())).convert("RGB")
                if img.size!=(tile_px,tile_px): img=img.resize((tile_px,tile_px),Image.Resampling.BILINEAR)
                mosaic.paste(img,((tx-x0)*tile_px,(ty-y0)*tile_px)); good+=1
            except Exception as exc:
                log(f"Satellite fallback tile failed {z}/{ty}/{tx}: {exc}")
                mosaic.paste(Image.new("RGB",(tile_px,tile_px),(110,110,110)),((tx-x0)*tile_px,(ty-y0)*tile_px))
    def px(lat,lon):
        lat=max(-85.05112878,min(85.05112878,lat)); xr=(lon+180)/360*n*tile_px; lr=math.radians(lat); yr=(1-math.asinh(math.tan(lr))/math.pi)/2*n*tile_px; return xr,yr
    lx,tn=px(north,west); rx,bs=px(south,east)
    crop=mosaic.crop((max(0,int(lx-x0*tile_px)),max(0,int(tn-y0*tile_px)),min(mosaic.width,int(rx-x0*tile_px)),min(mosaic.height,int(bs-y0*tile_px))))
    tw=1024; th=max(512,int(tw*terrain["size_y"]/terrain["size_x"]))
    crop.resize((tw,th),Image.Resampling.LANCZOS).save(aerial,"PNG",optimize=True)
    log(f"Ground satellite texture repaired: {aerial} {tw}x{th}, tiles={good}")
    return aerial

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

def _places_landmarks(places_source):
    """
    Parse Overture Places GeoJSON into named places that can be
    matched against nearby Overture buildings.

    Overture Places are point features. We retain useful metadata
    such as category, taxonomy, website and confidence.
    """
    from shapely.geometry import shape

    path = Path(places_source)

    if not path.exists():
        log(f"Overture Places file not found: {path}")
        return []

    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        log(f"Could not read Overture Places: {exc}")
        return []

    landmarks = []
    seen = set()

    for feature in data.get("features", []):
        geometry_data = feature.get("geometry")
        props = feature.get("properties") or {}

        if not geometry_data:
            continue

        try:
            geom = shape(geometry_data)
        except Exception:
            continue

        if geom.is_empty:
            continue

        # Overture Places should be points, but use centroid as a
        # safe fallback if a malformed/non-point geometry appears.
        point = geom if geom.geom_type == "Point" else geom.centroid

        lon = float(point.x)
        lat = float(point.y)

        # ----------------------------------------------
        # Name
        # ----------------------------------------------
        name = ""

        names = props.get("names")

        if isinstance(names, dict):
            primary = names.get("primary")

            if isinstance(primary, dict):
                name = (
                    primary.get("value")
                    or primary.get("name")
                    or ""
                )
            elif primary:
                name = str(primary)

        if not name:
            for key in (
                "name",
                "official_name",
                "common_name",
            ):
                value = props.get(key)
                if value:
                    name = str(value)
                    break

        if not name:
            continue

        # ----------------------------------------------
        # Category / taxonomy
        # ----------------------------------------------
        basic_category = props.get("basic_category") or ""

        taxonomy = props.get("taxonomy") or {}

        if isinstance(taxonomy, dict):
            taxonomy_primary = (
                taxonomy.get("primary") or ""
            )
            taxonomy_hierarchy = (
                taxonomy.get("hierarchy") or []
            )
        else:
            taxonomy_primary = ""
            taxonomy_hierarchy = []

        if isinstance(taxonomy_hierarchy, str):
            taxonomy_hierarchy = [taxonomy_hierarchy]

        # ----------------------------------------------
        # Website
        # ----------------------------------------------
        websites = props.get("websites") or []

        if isinstance(websites, str):
            websites = [websites]

        website = (
            websites[0]
            if isinstance(websites, list) and websites
            else ""
        )

        # ----------------------------------------------
        # Confidence
        # ----------------------------------------------
        confidence = props.get("confidence")

        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = None

        # ----------------------------------------------
        # Keep useful place
        # ----------------------------------------------
        landmark = {
            "name": name.strip(),
            "lat": lat,
            "lon": lon,
            "website": website,
            "basic_category": str(basic_category),
            "taxonomy_primary": str(taxonomy_primary),
            "taxonomy_hierarchy": [
                str(x) for x in taxonomy_hierarchy
            ],
            "confidence": confidence,
            "properties": props,
        }

        key = (
            landmark["name"].lower(),
            round(lat, 5),
            round(lon, 5),
        )

        if key in seen:
            continue

        seen.add(key)
        landmarks.append(landmark)

    log(
        f"Overture Places: "
        f"{len(landmarks)} named places"
    )

    for landmark in landmarks:
        category = (
            landmark["basic_category"]
            or landmark["taxonomy_primary"]
            or "unknown"
        )

        confidence = landmark["confidence"]

        if confidence is None:
            confidence_text = ""
        else:
            confidence_text = (
                f" confidence={confidence:.2f}"
            )

        log(
            f"  PLACE: {landmark['name']} "
            f"[{category}] "
            f"({landmark['lat']:.6f}, "
            f"{landmark['lon']:.6f})"
            f"{confidence_text}"
        )

    return landmarks


def _distance_m(lat1, lon1, lat2, lon2):
    """
    Approximate distance between two WGS84 coordinates in metres.
    Accurate enough for the small building/place matching radius.
    """
    return math.hypot(
        (lat2 - lat1) * 111320.0,
        (lon2 - lon1)
        * 111320.0
        * math.cos(
            math.radians(
                (lat1 + lat2) / 2.0
            )
        ),
    )
def _building_style(props, lat, lon, landmarks):
    """
    Determine the style of an Overture building without allowing one campus
    texture to leak into a neighboring campus.

    Priority:
      1. Nearest Overture Place within 80 m.
      2. Exact/strong building-name match, but only when it agrees with the
         nearest brand/campus anchor.
      3. Building category/class.
      4. Deterministic residential fallback.

    The important rule is that a generic Overture building name such as
    "KIET" is NOT allowed to override a closer ITS place. Overture Places
    are the authoritative source for campus/landmark assignment.
    """
    import hashlib
    import re

    props = props or {}

    def category_text(place):
        return " ".join([
            str(place.get("basic_category", "")),
            str(place.get("taxonomy_primary", "")),
            " ".join(place.get("taxonomy_hierarchy", [])),
            str(place.get("name", "")),
        ]).lower()

    def is_kiet(place):
        text = category_text(place)
        return (
            "kiet" in text
            or "krishna institute of engineering" in text
            or "kiet group" in text
        )

    def is_its(place):
        text = category_text(place)
        # Match ITS as an actual token, not the letters occurring inside
        # another word. Also accept the common dotted form I.T.S.
        return bool(
            re.search(r"\bi\.?t\.?s\.?\b", text, re.I)
            or "institute of technology and science" in text
        )

    # --------------------------------------------------
    # Building's own name -- used only as a secondary signal.
    # --------------------------------------------------
    values = []
    for key in (
        "name",
        "official_name",
        "common_name",
        "primary_name",
    ):
        value = props.get(key)
        if value:
            values.append(str(value))

    names = props.get("names")
    if isinstance(names, dict):
        for value in names.values():
            if isinstance(value, dict):
                value = value.get("value") or value.get("name") or ""
            if value:
                values.append(str(value))

    building_name_text = " ".join(values).lower()

    own_kiet = bool(
        "kiet" in building_name_text
        or "krishna institute of engineering" in building_name_text
        or "kiet group" in building_name_text
    )
    own_its = bool(
        re.search(r"\bi\.?t\.?s\.?\b", building_name_text, re.I)
        or "institute of technology and science" in building_name_text
    )

    # --------------------------------------------------
    # Find the nearest Overture Place.
    # --------------------------------------------------
    nearest = None
    nearest_distance = float("inf")

    for landmark in landmarks:
        distance = _distance_m(
            lat,
            lon,
            landmark["lat"],
            landmark["lon"],
        )
        if distance < nearest_distance:
            nearest = landmark
            nearest_distance = distance

    # --------------------------------------------------
    # Campus/brand assignment is authoritative.
    # --------------------------------------------------
    if nearest is not None and nearest_distance <= 80.0:
        name = nearest["name"]
        lower = name.lower()

        if is_kiet(nearest):
            return (
                "kiet",
                name,
                nearest.get("website", ""),
            )

        if is_its(nearest):
            return (
                "its",
                name,
                nearest.get("website", ""),
            )

        category = category_text(nearest)

        if any(word in category for word in (
            "college", "university", "institute", "school",
            "campus", "academy", "education",
        )):
            return (
                "education",
                name,
                nearest.get("website", ""),
            )

        if any(word in category for word in (
            "hospital", "medical", "clinic", "health",
        )):
            return (
                "hospital",
                name,
                nearest.get("website", ""),
            )

        if any(word in category for word in (
            "mall", "shopping", "retail", "commercial",
        )):
            return (
                "commercial",
                name,
                nearest.get("website", ""),
            )

        if any(word in category for word in (
            "industrial", "warehouse", "factory",
        )):
            return (
                "industrial",
                name,
                nearest.get("website", ""),
            )

    # --------------------------------------------------
    # If there is no nearby named place, allow an explicit building name.
    # This is deliberately AFTER Overture Places so it cannot make an ITS
    # building inherit KIET merely because Overture copied a campus name.
    # --------------------------------------------------
    if own_kiet:
        return (
            "kiet",
            "KIET",
            "https://www.kiet.edu/about/infrastructure/",
        )

    if own_its:
        return (
            "its",
            "I.T.S.",
            "https://ug.its.edu.in/photo-gallery-2025",
        )

    # --------------------------------------------------
    # Building class fallback.
    # --------------------------------------------------
    building_class = str(
        props.get("class") or props.get("subtype") or ""
    ).lower()

    if any(word in building_class for word in (
        "commercial", "retail", "office",
    )):
        return ("commercial", "commercial", "")

    if any(word in building_class for word in (
        "industrial", "warehouse",
    )):
        return ("industrial", "industrial", "")

    if any(word in building_class for word in (
        "education", "school", "college", "university",
    )):
        return ("education", "education", "")

    # --------------------------------------------------
    # Deterministic ordinary-building style.
    # --------------------------------------------------
    styles = ("res_a", "res_b", "res_c")
    seed = int(
        hashlib.sha256(
            f"{lat:.7f},{lon:.7f}".encode()
        ).hexdigest()[:8],
        16,
    )
    style = styles[seed % len(styles)]
    return (style, style, "")

def _safe_filename(text):
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text or "place"))
    return text.strip("._")[:80] or "place"


def _download_url_bytes(url, timeout=20):
    try:
        req = Request(
            url,
            headers={
                "User-Agent": "DroneRakshak/1.0 (+https://www.kiet.edu/)",
                "Accept": "image/avif,image/webp,image/apng,image/svg+xml,image/*,*/*;q=0.8",
            },
        )
        with urlopen(req, timeout=timeout) as r:
            return r.read(), r.headers.get_content_type()
    except Exception as exc:
        log(f"Image download failed for {url}: {exc}")
        return None, None


def _place_image_url(website):
    """
    Try to obtain a representative image from a place website.
    We intentionally keep this best-effort: many sites block automated
    requests or do not expose an image in their HTML.
    """
    if not website:
        return ""

    url = str(website).strip()
    if not url:
        return ""

    if not re.match(r"^https?://", url, re.I):
        url = "https://" + url

    try:
        req = Request(
            url,
            headers={
                "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) "
                              "AppleWebKit/537.36 Chrome/120 Safari/537.36"
            },
        )
        with urlopen(req, timeout=15) as r:
            content_type = r.headers.get_content_type()
            if content_type.startswith("image/"):
                return url
            html = r.read(2_000_000).decode("utf-8", errors="ignore")
    except Exception:
        return ""

    # Prefer OpenGraph, then Twitter card.
    patterns = [
        r'<meta[^>]+property=["\']og:image["\'][^>]+content=["\']([^"\']+)',
        r'<meta[^>]+content=["\']([^"\']+)["\'][^>]+property=["\']og:image["\']',
        r'<meta[^>]+name=["\']twitter:image["\'][^>]+content=["\']([^"\']+)',
        r'<meta[^>]+content=["\']([^"\']+)["\'][^>]+name=["\']twitter:image["\']',
    ]

    from urllib.parse import urljoin
    for pattern in patterns:
        match = re.search(pattern, html, flags=re.I)
        if match:
            return urljoin(url, match.group(1).strip())

    return ""


def _prepare_photo_texture(place, out_path, fallback_style):
    """
    Best-effort real-world image texture for a recognized named place.

    If the place website exposes og:image/twitter:image, download it and
    normalize it into a compact PNG texture. If not, return False and let
    the synthetic style texture be used.
    """
    out_path = Path(out_path)
    if out_path.exists():
        return True

    website = (place or {}).get("website", "")
    image_url = _place_image_url(website)
    if not image_url:
        return False

    data, content_type = _download_url_bytes(image_url)
    if not data:
        return False

    try:
        from io import BytesIO
        from PIL import Image, ImageOps

        img = Image.open(BytesIO(data)).convert("RGB")

        # A wide texture works better on building facades than an arbitrary
        # portrait crop. Keep the whole image and letterbox/crop deterministically.
        target_w, target_h = 512, 256
        img = ImageOps.fit(
            img,
            (target_w, target_h),
            method=Image.Resampling.LANCZOS,
            centering=(0.5, 0.5),
        )

        # Mild contrast normalization so web images remain visible in Gazebo.
        img.save(out_path, "PNG", optimize=True)

        log(
            f"Using real-world place image for "
            f"{(place or {}).get('name', fallback_style)}: {image_url}"
        )
        return True
    except Exception as exc:
        log(f"Could not convert place image {image_url}: {exc}")
        return False


def _make_style_texture(path, style, label="", place=None, roof=False):
    """
    Create one deterministic cached texture per style.

    For recognized named places, first try a real-world image from the
    Overture website. Roofs always use a dedicated roof texture because
    a facade photograph should not be projected onto roofs.
    """
    from PIL import Image, ImageDraw, ImageFont
    import hashlib
    import random

    path = Path(path)
    if path.exists():
        return

    if not roof and place:
        if _prepare_photo_texture(place, path, style):
            return

    facade_profiles = {
        "kiet": ((224,216,198),(153,54,42),(40,58,68)),
        "its": ((194,175,145),(101,62,43),(43,57,63)),
        "education": ((213,208,194),(133,78,53),(47,65,74)),
        "hospital": ((220,220,216),(71,126,133),(35,68,76)),
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
        "hospital": ((73,82,83),(101,128,132)),
        "commercial": ((69,77,80),(98,108,111)),
        "industrial": ((68,72,72),(92,99,98)),
        "res_a": ((82,75,68),(121,91,67)),
        "res_b": ((76,79,75),(106,110,100)),
        "res_c": ((82,71,67),(118,81,69)),
    }

    if roof:
        base, accent = roof_profiles.get(
            style, roof_profiles["res_a"]
        )
        window = tuple(
            max(0, min(255, c + 20)) for c in accent
        )
    else:
        base, accent, window = facade_profiles.get(
            style, facade_profiles["res_a"]
        )

    seed_text = f"{style}:{label}:{'roof' if roof else 'facade'}"
    rng = random.Random(
        int(hashlib.sha256(seed_text.encode()).hexdigest()[:8], 16)
    )

    img = Image.new("RGB", (256, 128), base)
    px = img.load()

    for y in range(128):
        for x in range(256):
            d = rng.randint(-4, 4)
            px[x, y] = tuple(
                max(0, min(255, c + d)) for c in base
            )

    draw = ImageDraw.Draw(img)

    if roof:
        # Roof planes get a separate, subtle tiled/metal/concrete pattern.
        tile_h = 28
        for y in range(0, 256, tile_h):
            draw.line(
                (0, y, 512, y),
                fill=accent,
                width=2,
            )
            for x in range((y // tile_h % 2) * 32, 512, 64):
                draw.line(
                    (x, y, x, min(256, y + tile_h)),
                    fill=window,
                    width=1,
                )
    else:
        floor = 72
        for y in range(0, 256, floor):
            draw.rectangle(
                (0, y, 512, y + 4),
                fill=(115, 110, 103),
            )
            draw.rectangle(
                (0, y + 5, 512, y + 9),
                fill=accent,
            )
            for x in range(18, 492, 58):
                draw.rectangle(
                    (x, y + 16, x + 34, y + 56),
                    fill=window,
                )
                draw.rectangle(
                    (x + 4, y + 20, x + 30, y + 24),
                    fill=(115, 139, 148),
                )

        if style in ("kiet", "its", "education"):
            for x in (70, 930):
                draw.rectangle(
                    (x, 0, x + 12, 256),
                    fill=accent,
                )

        if style in ("kiet", "its") and label:
            try:
                font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                    22,
                )
            except Exception:
                font = ImageFont.load_default()

            draw.rectangle(
                (8, 8, 260, 44),
                fill=(245, 241, 231),
            )
            draw.text(
                (14, 12),
                label[:28],
                fill=accent,
                font=font,
            )

    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, "PNG", optimize=True)


def _make_textured_building_meshes(geom, ground_z, height, style, facade_img, roof_img, facade_material_name=None, roof_material_name=None):
    """
    Convert one WGS84 Shapely Polygon into two textured meshes:
      - walls/facade
      - roof

    Coordinates returned are local east/north/up metres centered on the
    selected terrain. Separate meshes make facade and roof materials
    completely independent in Gazebo.
    """
    import numpy as np
    import trimesh
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import triangulate

    if geom.is_empty or geom.geom_type != "Polygon":
        return None, None

    # Overture polygon winding is not guaranteed to be consistent.
    # Normalize the exterior ring to counter-clockwise (and holes to the
    # opposite winding) so the wall normals point OUTWARD. Gazebo/OGRE
    # can cull back-faces, so inconsistent winding makes some exterior
    # walls disappear and makes the texture appear only on the inside.
    try:
        geom = orient(geom, sign=1.0)
    except Exception:
        pass

    exterior = list(geom.exterior.coords)
    if len(exterior) < 4:
        return None, None

    # Keep the supplied height sane.
    try:
        h = float(height)
    except Exception:
        h = 8.0
    h = max(2.5, min(h, 200.0))
    h *= BUILDING_SCALE_Z

    # The caller attaches the coordinate conversion attributes.
    cx = _TEXTURE_BUILD_CONTEXT["center_lon"]
    cy = _TEXTURE_BUILD_CONTEXT["center_lat"]
    lon_scale = _TEXTURE_BUILD_CONTEXT["lon_scale"]

    # Scale around this building's own centroid.  This increases the
    # footprint without changing the building's geographic center.
    building_center_lon = float(geom.centroid.x)
    building_center_lat = float(geom.centroid.y)

    def xy(pt):
        lon, lat = pt
        return (
            (building_center_lon - cx) * lon_scale
            + (lon - building_center_lon) * lon_scale * BUILDING_SCALE_X,
            (building_center_lat - cy) * 111320.0
            + (lat - building_center_lat) * 111320.0 * BUILDING_SCALE_Y,
        )

    # ---------------------------------------------------------
    # Walls
    # ---------------------------------------------------------
    wall_vertices = []
    wall_faces = []
    wall_uv = []

    rings = [geom.exterior] + list(geom.interiors)

    for ring in rings:
        coords = list(ring.coords)
        if len(coords) < 2:
            continue

        # Do not repeat the closing edge twice.
        for i in range(len(coords) - 1):
            p0 = xy(coords[i])
            p1 = xy(coords[i + 1])

            base_index = len(wall_vertices)

            wall_vertices.extend([
                [p0[0], p0[1], ground_z],
                [p1[0], p1[1], ground_z],
                [p1[0], p1[1], ground_z + h],
                [p0[0], p0[1], ground_z + h],
            ])

            edge_len = max(
                math.hypot(
                    p1[0] - p0[0],
                    p1[1] - p0[1],
                ),
                0.1,
            )

            # Tile approximately every 12 m horizontally and one texture
            # vertically per building floor/height.
            u_len = max(edge_len / 12.0, 0.25)
            v_len = max(h / 8.0, 0.5)

            wall_uv.extend([
                [0.0, 0.0],
                [u_len, 0.0],
                [u_len, v_len],
                [0.0, v_len],
            ])

            wall_faces.append(
                [base_index, base_index + 1,
                 base_index + 2, base_index + 3]
            )

    if wall_vertices:
        wall_mesh = trimesh.Trimesh(
            vertices=np.asarray(wall_vertices, dtype=float),
            faces=np.asarray(wall_faces, dtype=int),
            process=False,
        )
        wall_mesh.visual = trimesh.visual.texture.TextureVisuals(
            uv=np.asarray(wall_uv, dtype=float),
            material=trimesh.visual.material.SimpleMaterial(
                image=facade_img,
                name=facade_material_name or f"{style}_facade",
            ),
        )
    else:
        wall_mesh = None

    # ---------------------------------------------------------
    # Roof
    # ---------------------------------------------------------
    # Shapely triangulation handles concave polygons. Only keep triangles
    # whose representative point lies inside the polygon, which also
    # removes triangles crossing holes.
    roof_vertices = []
    roof_faces = []
    roof_uv = []

    minx, miny, maxx, maxy = geom.bounds
    span_x = max((maxx - minx) * lon_scale, 0.1)
    span_y = max((maxy - miny) * 111320.0, 0.1)

    for tri in triangulate(geom):
        try:
            if not geom.covers(tri.representative_point()):
                continue
        except Exception:
            continue

        coords = list(tri.exterior.coords)[:3]
        base_index = len(roof_vertices)

        for lon, lat in coords:
            x, y = xy((lon, lat))
            roof_vertices.append([x, y, ground_z + h])

            u = ((lon - minx) * lon_scale) / span_x * 2.0
            v = ((lat - miny) * 111320.0) / span_y * 2.0
            roof_uv.append([u, v])

        # Force the roof triangle normal to point upward (+Z).
        # Shapely triangulation does not promise a consistent winding.
        a = roof_vertices[base_index]
        b = roof_vertices[base_index + 1]
        c = roof_vertices[base_index + 2]
        signed_area = (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )
        if signed_area >= 0:
            roof_faces.append(
                [base_index, base_index + 1, base_index + 2]
            )
        else:
            roof_faces.append(
                [base_index, base_index + 2, base_index + 1]
            )

    if roof_vertices:
        roof_mesh = trimesh.Trimesh(
            vertices=np.asarray(roof_vertices, dtype=float),
            faces=np.asarray(roof_faces, dtype=int),
            process=False,
        )
        roof_mesh.visual = trimesh.visual.texture.TextureVisuals(
            uv=np.asarray(roof_uv, dtype=float),
            material=trimesh.visual.material.SimpleMaterial(
                image=roof_img,
                name=roof_material_name or f"{style}_roof",
            ),
        )
    else:
        roof_mesh = None

    return wall_mesh, roof_mesh


# Global context used only while the single threaded exporter is running.
_TEXTURE_BUILD_CONTEXT = {}



def _make_building_collision_mesh(geom, ground_z, height):
    """
    Create a lightweight, untextured collision mesh from the building
    footprint. The visual mesh keeps all facade / roof detail, while the
    collision mesh uses a simplified footprint extruded to the same 2x
    building height.

    The footprint is simplified before extrusion to keep Bullet / DART
    collision processing cheap and stable. Holes are preserved when possible.
    """
    import numpy as np
    import trimesh
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import triangulate

    if geom.is_empty or geom.geom_type != "Polygon":
        return None

    try:
        # Small simplification removes dense Overture boundary vertices while
        # preserving the actual building footprint at map scale.
        simplified = geom.simplify(0.35, preserve_topology=True)
        if not simplified.is_empty and simplified.geom_type == "Polygon":
            geom = simplified
    except Exception:
        pass

    try:
        geom = orient(geom, sign=1.0)
    except Exception:
        pass

    if geom.is_empty or geom.geom_type != "Polygon":
        return None

    h = max(2.5, min(float(height), 200.0)) * BUILDING_SCALE_Z

    cx = _TEXTURE_BUILD_CONTEXT["center_lon"]
    cy = _TEXTURE_BUILD_CONTEXT["center_lat"]
    lon_scale = _TEXTURE_BUILD_CONTEXT["lon_scale"]

    building_center_lon = float(geom.centroid.x)
    building_center_lat = float(geom.centroid.y)

    def xy(pt):
        lon, lat = pt
        return (
            (building_center_lon - cx) * lon_scale
            + (lon - building_center_lon) * lon_scale * BUILDING_SCALE_X,
            (building_center_lat - cy) * 111320.0
            + (lat - building_center_lat) * 111320.0 * BUILDING_SCALE_Y,
        )

    vertices = []
    faces = []

    # Side walls for exterior and interior rings.
    rings = [geom.exterior] + list(geom.interiors)
    for ring in rings:
        coords = list(ring.coords)
        for i in range(len(coords) - 1):
            p0 = xy(coords[i])
            p1 = xy(coords[i + 1])
            idx = len(vertices)
            vertices.extend([
                [p0[0], p0[1], ground_z],
                [p1[0], p1[1], ground_z],
                [p1[0], p1[1], ground_z + h],
                [p0[0], p0[1], ground_z + h],
            ])
            # Keep every collision face triangular. Mixing quad faces from
            # the side walls with triangle faces from the roof/base makes
            # NumPy reject the faces array as an inhomogeneous shape.
            faces.append([idx, idx + 1, idx + 2])
            faces.append([idx, idx + 2, idx + 3])

    # Top and bottom are triangulated from the simplified footprint.
    for tri in triangulate(geom):
        try:
            if not geom.covers(tri.representative_point()):
                continue
        except Exception:
            continue

        coords = list(tri.exterior.coords)[:3]
        top_idx = len(vertices)
        for lon, lat in coords:
            x, y = xy((lon, lat))
            vertices.append([x, y, ground_z + h])

        bottom_idx = len(vertices)
        for lon, lat in coords:
            x, y = xy((lon, lat))
            vertices.append([x, y, ground_z])

        # Top points upward, bottom points downward.
        a, b, c = [vertices[top_idx + i] for i in range(3)]
        signed_area = (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )
        if signed_area >= 0:
            faces.append([top_idx, top_idx + 1, top_idx + 2])
            faces.append([bottom_idx, bottom_idx + 2, bottom_idx + 1])
        else:
            faces.append([top_idx, top_idx + 2, top_idx + 1])
            faces.append([bottom_idx, bottom_idx + 1, bottom_idx + 2])

    if not vertices or not faces:
        return None

    return trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=float),
        faces=np.asarray(faces, dtype=int),
        process=False,
    )

def build_overture_textured_obj(geojson_in, out_obj, terrain, places_source=None):
    """
    Build the final Overture building layer as a textured OBJ/MTL asset.

    This replaces the terrain-generator DAE exporter for the building layer
    because OBJ/MTL gives us explicit per-mesh facade and roof materials.
    """
    import numpy as np
    import trimesh
    from PIL import Image
    from shapely.geometry import shape, box, Polygon, MultiPolygon

    out_obj = Path(out_obj)
    out_obj.parent.mkdir(parents=True, exist_ok=True)

    bbox = bbox_from_center_size(
        terrain["center_lat"],
        terrain["center_lon"],
        terrain["size_x"],
        terrain["size_y"],
        margin_m=0,
    )

    south, west, north, east = bbox
    terrain_clip = box(west, south, east, north)

    data = json.loads(Path(geojson_in).read_text())
    landmarks = (
        _places_landmarks(places_source)
        if places_source
        else []
    )

    # Coordinate conversion from WGS84 to the local terrain frame.
    lon_scale = 111320.0 * math.cos(
        math.radians(terrain["center_lat"])
    )

    _TEXTURE_BUILD_CONTEXT.clear()
    _TEXTURE_BUILD_CONTEXT.update({
        "center_lon": terrain["center_lon"],
        "center_lat": terrain["center_lat"],
        "lon_scale": lon_scale,
    })

    texture_dir = out_obj.parent / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)

    texture_cache = {}
    style_counts = {}
    # Accumulate buildings by material/style and export only a small number
    # of meshes. One OBJ object per building creates hundreds of Gazebo/OGRE
    # render objects and can consume several GB of RAM.
    style_wall_meshes = {}
    style_roof_meshes = {}
    style_collision_meshes = {}

    # Cache actual place records by style/name so a campus does not create
    # a new image for every building.
    place_cache = {}

    def get_textures(style, label, source):
        cache_key = (
            style,
            label,
            source,
        )
        if cache_key in texture_cache:
            return texture_cache[cache_key]

        place = None
        if style in ("kiet", "its", "education", "hospital",
                     "commercial", "industrial"):
            for lm in landmarks:
                if label and lm["name"] == label:
                    place = lm
                    break

        # For explicit KIET/ITS matches that do not have a corresponding
        # Overture Place record, retain the official source URL.
        if place is None and style == "kiet":
            place = {
                "name": label or "KIET",
                "website": source or "https://www.kiet.edu/about/infrastructure/",
            }
        elif place is None and style == "its":
            place = {
                "name": label or "I.T.S.",
                "website": source or "https://ug.its.edu.in/photo-gallery-2025",
            }

        safe = _safe_filename(label or style)

        facade_path = texture_dir / f"{safe}_{style}_facade.png"
        roof_path = texture_dir / f"{style}_roof.png"

        _make_style_texture(
            facade_path,
            style,
            label=label,
            place=place,
            roof=False,
        )

        # Roof texture is shared by style, not by building.
        _make_style_texture(
            roof_path,
            style,
            label=style,
            place=None,
            roof=True,
        )

        facade_img = Image.open(facade_path).convert("RGB")
        roof_img = Image.open(roof_path).convert("RGB")

        facade_material_name = _safe_filename(
            f"{safe}_{style}_facade"
        )
        roof_material_name = _safe_filename(
            f"{style}_roof"
        )

        texture_cache[cache_key] = (
            facade_img,
            roof_img,
            facade_material_name,
            roof_material_name,
        )
        return texture_cache[cache_key]

    terrain_image = Image.open(
        terrain["heightmap_path"]
    ).convert("I")

    processed = 0
    skipped = 0

    for feature_index, feature in enumerate(data.get("features", [])):
        geom_data = feature.get("geometry")
        props = feature.get("properties") or {}

        if not geom_data:
            skipped += 1
            continue

        try:
            geom = shape(geom_data)
            geom = geom.intersection(terrain_clip)
        except Exception:
            skipped += 1
            continue

        if geom.is_empty:
            skipped += 1
            continue

        if not geom.is_valid:
            try:
                geom = geom.buffer(0)
            except Exception:
                skipped += 1
                continue

        if geom.is_empty:
            skipped += 1
            continue

        polygons = []
        if isinstance(geom, Polygon):
            polygons = [geom]
        elif isinstance(geom, MultiPolygon):
            polygons = list(geom.geoms)
        else:
            # GeometryCollection may contain polygons after clipping.
            polygons = [
                g for g in getattr(geom, "geoms", [])
                if isinstance(g, Polygon)
            ]

        if not polygons:
            skipped += 1
            continue

        # A feature can become multiple polygons after clipping.
        for poly_index, poly in enumerate(polygons):
            if poly.area <= 1e-10:
                continue

            # Re-normalize after bbox clipping as a final guarantee that
            # every building's exterior wall winding is consistent.
            try:
                poly = orient(poly, sign=1.0)
            except Exception:
                pass

            c = poly.centroid
            lat = float(c.y)
            lon = float(c.x)

            style, label, source = _building_style(
                props,
                lat,
                lon,
                landmarks,
            )

            # Overture's height is preferred. Fall back to floors.
            raw_height = props.get("height")

            if raw_height in (None, "", 0):
                floors = (
                    props.get("num_floors")
                    or props.get("building:levels")
                    or 2
                )
                try:
                    raw_height = float(floors) * 3.2
                except Exception:
                    raw_height = 8.0

            try:
                height = float(raw_height)
            except (TypeError, ValueError):
                height = 8.0

            # Sample the terrain directly below the building centroid.
            ground_z = sample_height(
                lon,
                lat,
                terrain_image,
                bbox,
                terrain["size_z"],
                terrain["pos_z"],
            )

            (facade_img, roof_img, facade_material_name,
             roof_material_name) = get_textures(
                style,
                label,
                source,
            )

            wall_mesh, roof_mesh = _make_textured_building_meshes(
                poly,
                ground_z,
                height,
                style,
                facade_img,
                roof_img,
                facade_material_name,
                roof_material_name,
            )

            if wall_mesh is not None:
                style_wall_meshes.setdefault(style, []).append(wall_mesh)

            if roof_mesh is not None:
                style_roof_meshes.setdefault(style, []).append(roof_mesh)

            collision_mesh = _make_building_collision_mesh(
                poly,
                ground_z,
                height,
            )
            if collision_mesh is not None:
                style_collision_meshes.setdefault(style, []).append(collision_mesh)

            style_counts[style] = style_counts.get(style, 0) + 1
            processed += 1

    # Merge all buildings sharing the same material into one mesh. This is
    # the key memory optimization: hundreds of building objects become at
    # most one facade mesh + one roof mesh per style.
    scene = trimesh.Scene()
    for style in sorted(set(style_wall_meshes) | set(style_roof_meshes)):
        walls = style_wall_meshes.get(style, [])
        roofs = style_roof_meshes.get(style, [])
        if walls:
            merged_wall = trimesh.util.concatenate(walls)
            # Reuse the cached material/image for the style.
            for cache_key, cached in texture_cache.items():
                if cache_key[0] == style:
                    _, _, facade_material_name, _ = cached
                    merged_wall.visual = trimesh.visual.texture.TextureVisuals(
                        uv=merged_wall.visual.uv,
                        material=trimesh.visual.material.SimpleMaterial(
                            image=cached[0], name=facade_material_name
                        ),
                    )
                    break
            scene.add_geometry(merged_wall, geom_name=f"{style}_facades")
        if roofs:
            merged_roof = trimesh.util.concatenate(roofs)
            for cache_key, cached in texture_cache.items():
                if cache_key[0] == style:
                    _, _, _, roof_material_name = cached
                    merged_roof.visual = trimesh.visual.texture.TextureVisuals(
                        uv=merged_roof.visual.uv,
                        material=trimesh.visual.material.SimpleMaterial(
                            image=cached[1], name=roof_material_name
                        ),
                    )
                    break
            scene.add_geometry(merged_roof, geom_name=f"{style}_roofs")

    if not scene.geometry:
        raise RuntimeError(
            "No Overture building geometry could be converted "
            "to the textured OBJ."
        )

    # Export each style into its own OBJ/MTL asset set.
    # Gazebo/OGRE can otherwise collapse/resolve multi-material OBJ assets
    # inconsistently and make every building use the first material (often
    # KIET). Separate style assets make material ownership unambiguous while
    # keeping the number of render objects small.
    style_dir = out_obj.parent / "building_styles"
    style_dir.mkdir(parents=True, exist_ok=True)

    manifest = []
    all_styles = sorted(
        set(style_wall_meshes)
        | set(style_roof_meshes)
        | set(style_collision_meshes)
    )

    for style in all_styles:
        style_scene = trimesh.Scene()
        walls = style_wall_meshes.get(style, [])
        roofs = style_roof_meshes.get(style, [])

        if walls:
            merged_wall = trimesh.util.concatenate(walls)
            # Pick the exact texture cache entry used by this style.
            # Prefer the first entry for this style; KIET/ITS remain isolated
            # because they are exported to different OBJ files.
            cached = next(
                (v for k, v in texture_cache.items() if k[0] == style),
                None,
            )
            if cached is not None:
                merged_wall.visual = trimesh.visual.texture.TextureVisuals(
                    uv=merged_wall.visual.uv,
                    material=trimesh.visual.material.SimpleMaterial(
                        image=cached[0],
                        name=cached[2],
                    ),
                )
            style_scene.add_geometry(
                merged_wall,
                geom_name=f"{style}_facades",
            )

        if roofs:
            merged_roof = trimesh.util.concatenate(roofs)
            cached = next(
                (v for k, v in texture_cache.items() if k[0] == style),
                None,
            )
            if cached is not None:
                merged_roof.visual = trimesh.visual.texture.TextureVisuals(
                    uv=merged_roof.visual.uv,
                    material=trimesh.visual.material.SimpleMaterial(
                        image=cached[1],
                        name=cached[3],
                    ),
                )
            style_scene.add_geometry(
                merged_roof,
                geom_name=f"{style}_roofs",
            )

        if not style_scene.geometry:
            continue

        # Export a separate lightweight collision mesh. It contains no UVs,
        # textures or materials and is intentionally simplified from the
        # building footprint before extrusion.
        collision_meshes = style_collision_meshes.get(style, [])
        collision_obj = None
        if collision_meshes:
            merged_collision = trimesh.util.concatenate(collision_meshes)
            collision_dir = style_dir / style
            collision_dir.mkdir(parents=True, exist_ok=True)
            collision_obj = collision_dir / f"collision_{style}.obj"
            # DART / Bullet requires valid vertex normals when it imports
            # these OBJ collision meshes.  Exporting without normals causes
            # Gazebo 8.11 to create a zero-normal submesh and can segfault
            # inside the Bullet collision shape creation path.
            try:
                merged_collision.remove_duplicate_faces()
                merged_collision.remove_degenerate_faces()
            except Exception:
                pass
            try:
                merged_collision.fix_normals()
            except Exception:
                pass
            collision_export = trimesh.exchange.obj.export_obj(
                merged_collision,
                include_normals=True,
                include_texture=False,
                return_texture=False,
                write_texture=False,
            )
            collision_obj.write_text(collision_export)

        style_subdir = style_dir / style
        style_subdir.mkdir(parents=True, exist_ok=True)
        style_obj = style_subdir / f"overture_{style}.obj"
        style_mtl = style_obj.with_suffix(".mtl").name

        exported, assets = trimesh.exchange.obj.export_obj(
            style_scene,
            include_normals=True,
            include_texture=True,
            return_texture=True,
            write_texture=False,
            mtl_name=style_mtl,
        )
        style_obj.write_text(exported)

        for asset_name, asset_data in assets.items():
            target = style_subdir / Path(asset_name).name
            if isinstance(asset_data, str):
                target.write_text(asset_data)
            elif isinstance(asset_data, bytes):
                target.write_bytes(asset_data)
            else:
                target.write_bytes(bytes(asset_data))

        manifest.append({
            "style": style,
            "obj": str(style_obj),
            "collision_obj": str(collision_obj) if collision_obj else "",
            "count": style_counts.get(style, 0),
        })

    if not manifest:
        raise RuntimeError(
            "No Overture building geometry could be converted "
            "to textured OBJ assets."
        )

    manifest_path = out_obj.with_name(
        "overture_building_styles.json"
    )
    manifest_path.write_text(
        json.dumps(manifest, indent=2)
    )

    # Keep a tiny marker OBJ for backwards compatibility with existing
    # paths. The actual Gazebo world uses the per-style OBJ files above.
    out_obj.write_text(
        "# Drone Rakshak: textured buildings are exported per style.\n"
    )

    log(
        f"Textured Overture buildings: processed={processed}, "
        f"skipped={skipped}"
    )

    for style in sorted(style_counts):
        log(
            f"  STYLE {style}: "
            f"{style_counts[style]} building polygons"
        )

    log(
        f"Textured building style assets: {style_dir}"
    )
    log(
        "Exported one isolated OBJ/MTL material set per style "
        "to prevent Gazebo material bleed between KIET, ITS and other buildings."
    )
    log(
        "Exported lightweight footprint collision meshes separately from "
        "the detailed textured visual meshes."
    )
    log(
        f"Building scale: X={BUILDING_SCALE_X:.2f}x, "
        f"Y={BUILDING_SCALE_Y:.2f}x, Z={BUILDING_SCALE_Z:.2f}x"
    )
    log("Facade walls use normalized outward winding (single-sided for lower memory).")
    log("Building meshes merged by style/material; roof faces point upward.")

    return out_obj



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

        # Every OSM mesh must remain completely inside the
        # selected terrain BBOX.
        #
        # Do NOT allow roads to cross the terrain boundary.
        # Checking only the road center allows the actual road
        # geometry to extend outside the satellite plane.
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

def add_model(world_root, name, uri, pose=None, collision=False, material=False, collision_uri=None):
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
        ET.SubElement(cmesh, "uri").text = collision_uri or uri

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

    return "file://" + quote(
        resolved,
        safe="/:@-._~!$&'()*+,;=@"
    )


def ensure_px4_sensor_plugins(world_root, px4_root):
    world=world_root.find("world")
    if world is None: raise RuntimeError("Generated SDF has no <world> element")
    filenames={"gz-sim-physics-system","gz-sim-user-commands-system","gz-sim-scene-broadcaster-system","gz-sim-contact-system","gz-sim-imu-system","gz-sim-air-pressure-system","gz-sim-air-speed-system","gz-sim-navsat-system","gz-sim-magnetometer-system","gz-sim-sensors-system"}
    names={"gz::sim::systems::Physics","gz::sim::systems::UserCommands","gz::sim::systems::SceneBroadcaster","gz::sim::systems::Contact","gz::sim::systems::Imu","gz::sim::systems::AirPressure","gz::sim::systems::AirSpeed","gz::sim::systems::NavSat","gz::sim::systems::Magnetometer","gz::sim::systems::Sensors"}
    for p in list(world.findall("plugin")):
        if p.get("filename") in filenames or p.get("name") in names: world.remove(p)
    stack=[("gz-sim-physics-system","gz::sim::systems::Physics"),("gz-sim-user-commands-system","gz::sim::systems::UserCommands"),("gz-sim-scene-broadcaster-system","gz::sim::systems::SceneBroadcaster"),("gz-sim-contact-system","gz::sim::systems::Contact"),("gz-sim-imu-system","gz::sim::systems::Imu"),("gz-sim-air-pressure-system","gz::sim::systems::AirPressure"),("gz-sim-air-speed-system","gz::sim::systems::AirSpeed"),("gz-sim-navsat-system","gz::sim::systems::NavSat"),("gz-sim-magnetometer-system","gz::sim::systems::Magnetometer"),("gz-sim-sensors-system","gz::sim::systems::Sensors")]
    for filename,name in stack:
        p=ET.Element("plugin",{"entity_name":"*","entity_type":"world","filename":filename,"name":name})
        if filename=="gz-sim-sensors-system": ET.SubElement(p,"render_engine").text="ogre2"
        world.append(p)
    log("Embedded Gazebo sensor stack: IMU + AirPressure + Magnetometer + NavSat + Sensors")
    plugin_dir=px4_root/"build"/"px4_sitl_default"/"src"/"modules"/"simulation"/"gz_plugins"
    for filename,name in [("libOpticalFlowSystem.so","custom::OpticalFlowSystem"),("libGstCameraSystem.so","custom::GstCameraSystem")]:
        if (plugin_dir/filename).is_file(): world.append(ET.Element("plugin",{"entity_name":"*","entity_type":"world","filename":filename,"name":name}))

def build_final_world(terrain_world, osm_glb, overture_obj, final_world):
    root = ET.parse(terrain_world).getroot()
    world_element = root.find("world")
    if world_element is None:
        raise RuntimeError("Generated terrain SDF has no <world> element")
    # Keep the Gazebo world name stable across every generated map. The folder
    # and filename may be 31, 32, ... but PX4 always connects to drone_rakshak.
    world_element.set("name", "drone_rakshak")

    remove_generated_building_model(root)

    for parent in root.iter():
        for model in list(parent.findall("model")):
            name = model.get("name", "")
            if name.endswith("_buildings") and name != "drone_rakshak_overture_buildings":
                parent.remove(model)
                log(f"Removed legacy building model: {name}")

    terrain_dir = Path(terrain_world).resolve().parent
    mesh_dir = terrain_dir / "mesh"
    final_mesh_dir = final_world.parent / "mesh"

    if final_mesh_dir.exists() or final_mesh_dir.is_symlink():
        if final_mesh_dir.is_symlink() or final_mesh_dir.is_file():
            final_mesh_dir.unlink()
        else:
            shutil.rmtree(final_mesh_dir)

    if mesh_dir.exists():
        try:
            final_mesh_dir.symlink_to(mesh_dir, target_is_directory=True)
        except OSError:
            shutil.copytree(mesh_dir, final_mesh_dir)

    uri_map = {
        "mesh/height_map.png": gazebo_file_uri(mesh_dir / "height_map.png"),
        "mesh/aerial.png": gazebo_file_uri(mesh_dir / "aerial.png"),
        "mesh/normal_map.png": gazebo_file_uri(mesh_dir / "normal_map.png"),
        "mesh/buildings.dae": gazebo_file_uri(mesh_dir / "buildings.dae"),
    }
    heightmap=root.find(".//visual/geometry/heightmap")
    if heightmap is not None:
        tex=heightmap.find("texture")
        if tex is None: tex=ET.SubElement(heightmap,"texture")
        diffuse=tex.find("diffuse")
        if diffuse is None: diffuse=ET.SubElement(tex,"diffuse")
        diffuse.text=gazebo_file_uri(mesh_dir/"aerial.png")
        normal=tex.find("normal")
        if normal is not None and (mesh_dir/"normal_map.png").exists(): normal.text=gazebo_file_uri(mesh_dir/"normal_map.png")

    for uri in root.findall(".//uri"):
        value = (uri.text or "").strip()
        if value in uri_map:
            target = Path(uri_map[value].replace("file://", ""))
            uri.text = uri_map[value] if target.exists() else value

    terrain = parse_sdf(terrain_world)

    if os.environ.get("DRONE_RAKSHAK_LOAD_OSM","0")=="1":
        add_model(root,"drone_rakshak_osm_objects",gazebo_file_uri(osm_glb),pose=f"{terrain['pos_x']} {terrain['pos_y']} 0 1.57079632679 0 0",collision=False)
        log("OSM2World visual layer enabled.")
    else:
        log("OSM2World visual layer disabled for RAM; satellite ground + Overture buildings are used.")

    # IMPORTANT: do NOT put all styles into one multi-material OBJ.
    # Gazebo/OGRE can resolve the first material for the whole mesh on some
    # versions, which is why the previous build made every building look like
    # KIET. Load each style as its own mesh/model so its MTL is unambiguous.
    style_manifest = overture_obj.with_name(
        "overture_building_styles.json"
    )
    if not style_manifest.exists():
        raise RuntimeError(
            f"Missing textured building style manifest: {style_manifest}"
        )

    styles = json.loads(style_manifest.read_text())
    for entry in styles:
        style = str(entry["style"])
        style_obj = Path(entry["obj"])
        if not style_obj.exists():
            raise RuntimeError(
                f"Missing textured building OBJ for style {style}: {style_obj}"
            )

        collision_obj = Path(entry.get("collision_obj", ""))
        if not collision_obj.exists():
            raise RuntimeError(
                f"Missing lightweight collision OBJ for style {style}: {collision_obj}"
            )

        add_model(
            root,
            f"drone_rakshak_overture_{style}",
            gazebo_file_uri(style_obj),
            pose=f"{terrain['pos_x']} {terrain['pos_y']} 0 0 0 0",
            collision=True,
            material=False,
            collision_uri=gazebo_file_uri(collision_obj),
        )
        log(
            f"Gazebo building style loaded: {style} "
            f"({entry.get('count', 0)} polygons) "
            "with lightweight footprint collision"
        )

    for element in root.iter():
        if element.text:
            value = element.text.strip()
            if value.startswith("mesh/"):
                element.text = gazebo_file_uri(terrain_dir / value)

    ensure_px4_sensor_plugins(root, HOME / "PX4-Autopilot")

    ET.indent(root, space="  ")
    final_world.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(
        final_world,
        encoding="utf-8",
        xml_declaration=True,
    )
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
    env=os.environ.copy(); env["PYTHONUNBUFFERED"]="1"
    if shutil.which("MicroXRCEAgent"):
        probe=subprocess.run(["bash","-lc","ss -lun 2>/dev/null | grep -q ':8888'"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        if probe.returncode!=0:
            start_bg(["MicroXRCEAgent","udp4","-p","8888"],env=env,logfile=logdir/"microxrce.log"); time.sleep(1)
        else: log("MicroXRCEAgent already listening on UDP 8888.")
    if os.environ.get("DRONE_RAKSHAK_AUTO_ROS","0")=="1":
        for exe in ["radar_receiver","radar_fusion","obstacle_detector","avoidance_controller","offboard_controller"]:
            found=discover_ros_exe(exe)
            if found:
                pkg,ex=found; cmd=["bash","-lc",f"source /opt/ros/jazzy/setup.bash && source ~/drone_ws/install/setup.bash && exec ros2 run {pkg} {ex}"]
                start_bg(cmd,env=env,logfile=logdir/f"{exe}.log"); time.sleep(0.5)
    else: log("Heavy Drone Rakshak ROS nodes disabled by default for RAM.")
    if os.environ.get("DRONE_RAKSHAK_AUTO_QGC","0")=="1":
        qgc=HOME/"Downloads"/"QGroundControl-x86_64.AppImage"
        if qgc.exists() and subprocess.call(["bash","-lc","pgrep -f 'QGroundControl-x86_64.AppImage' >/dev/null 2>&1"])!=0: start_bg([str(qgc)],cwd=qgc.parent,logfile=logdir/"qgroundcontrol.log")
    else: log("QGroundControl auto-start disabled for RAM.")

def stop_sim():
    """Stop only processes started by this launcher; never pkill the shell."""
    log("Stopping previous Drone Rakshak simulation...")
    stop_tracked_processes(force=False)
    time.sleep(1)

def start_sim(final_world, world_workdir):
    logdir = world_workdir / "logs"
    logdir.mkdir(parents=True, exist_ok=True)

    # Every generated world is normalized to the stable PX4 world name.
    world_root = ET.parse(final_world).getroot()
    world_element = world_root.find("world")
    if world_element is None or not world_element.get("name"):
        raise RuntimeError(f"Could not determine Gazebo world name from {final_world}")
    gazebo_world_name = world_element.get("name")
    log(f"Gazebo world name: {gazebo_world_name}")

    # ---------------------------------------------------------
    # Gazebo SERVER
    # ---------------------------------------------------------
    server_env = os.environ.copy()
    user_site = str(HOME / ".local" / "lib" / "python3.12" / "site-packages")
    server_env["PYTHONPATH"] = user_site + (os.pathsep + server_env["PYTHONPATH"] if server_env.get("PYTHONPATH") else "")
    server_env["QT_OPENGL"] = "software"
    server_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    server_env["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"

    # Mirror the PX4-generated Gazebo environment from
    # build/px4_sitl_default/rootfs/gz_env.sh. The model path is needed to
    # spawn x500/model.sdf, while the system-plugin path is needed for the
    # PX4 Gazebo sensor / bridge plugins.
    px4_root = HOME / "PX4-Autopilot"
    px4_gz_models = px4_root / "Tools" / "simulation" / "gz" / "models"
    px4_gz_worlds = px4_root / "Tools" / "simulation" / "gz" / "worlds"
    px4_gz_plugins = px4_root / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_plugins"

    existing_gz_resources = server_env.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = [str(px4_gz_models), str(px4_gz_worlds)]
    if existing_gz_resources:
        resource_paths.append(existing_gz_resources)
    server_env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resource_paths)

    existing_gz_plugins = server_env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    plugin_paths = [str(px4_gz_plugins)]
    if existing_gz_plugins:
        plugin_paths.append(existing_gz_plugins)
    server_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(plugin_paths)

    # Use the official PX4 Gazebo server configuration. Gazebo's server_config
    # supplies the core sensor systems and avoids duplicated inline systems.
    log(f"Gazebo resource path: {server_env['GZ_SIM_RESOURCE_PATH']}")
    log(f"Gazebo system plugin path: {server_env['GZ_SIM_SYSTEM_PLUGIN_PATH']}")

    log("Starting Gazebo server...")

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

    # Give the server time to parse the world and load assets.
    time.sleep(8)

    # If the server died immediately, don't open a useless black GUI.
    if server is not None and server.poll() is not None:
        log(
            "Gazebo server exited before GUI startup. "
            f"See {logdir / 'gz_server.log'}"
        )
        return None

    # Verify the core sensor systems were actually loaded before starting the GUI.
    try:
        server_log_text = (logdir / "gz_server.log").read_text(errors="ignore")
        for label in ("Imu", "AirPressure", "Magnetometer", "NavSat", "Sensors"):
            if f"systems::{label}" not in server_log_text:
                log(f"WARNING: Gazebo has not reported systems::{label} yet; see gz_server.log")
    except Exception:
        pass

    # ---------------------------------------------------------
    # Gazebo GUI
    # ---------------------------------------------------------
    # Use the previously working WSLg/XCB path.
    # Do NOT force llvmpipe on the GUI.
    gui_env = os.environ.copy()
    gui_env.pop("WAYLAND_DISPLAY", None)
    gui_env["QT_QPA_PLATFORM"] = "xcb"
    gui_env["GZ_SIM_RESOURCE_PATH"] = server_env["GZ_SIM_RESOURCE_PATH"]
    gui_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = server_env["GZ_SIM_SYSTEM_PLUGIN_PATH"]
    log("Starting Gazebo GUI...")

    start_bg(
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

    # ---------------------------------------------------------
    # PX4
    # ---------------------------------------------------------
    px4_env = os.environ.copy()
    px4_env["PYTHONPATH"] = server_env["PYTHONPATH"]
    px4_env["GZ_SIM_RESOURCE_PATH"] = server_env["GZ_SIM_RESOURCE_PATH"]
    px4_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = server_env["GZ_SIM_SYSTEM_PLUGIN_PATH"]
    px4_env["PX4_GZ_MODELS"] = str(px4_gz_models)
    px4_env["PX4_GZ_WORLDS"] = str(px4_gz_worlds)
    px4_env["PX4_GZ_PLUGINS"] = str(px4_gz_plugins)
    px4_env["PX4_GZ_STANDALONE"] = "1"
    px4_env["PX4_GZ_WORLD"] = gazebo_world_name
    px4_env["PX4_SIM_MODEL"] = "gz_x500"
    px4_env["PX4_SYS_AUTOSTART"] = "4001"

    # Spawn high enough to be clearly visible above the generated buildings.
    # PX4 documents this variable as the x,y,z,roll,pitch,yaw spawn pose.
    px4_env["PX4_GZ_MODEL_POSE"] = "0,0,30,0,0,0"

    start_ros_and_qgc(logdir)

    log("Starting PX4...")
    log(
        "PX4 target: "
        f"world={gazebo_world_name}, model=gz_x500, pose=0,0,30,0,0,0"
    )

    # Launch the already-built PX4 executable directly.
    # Do NOT run "make px4_sitl gz_x500" here: the launcher itself is
    # already running under uv, which can hide the user's kconfiglib/menuconfig
    # installation and unnecessarily reconfigure/rebuild PX4 on every map.
    px4_bin = HOME / "PX4-Autopilot" / "build" / "px4_sitl_default" / "bin" / "px4"
    if not px4_bin.is_file():
        log(
            f"PX4 executable not found: {px4_bin}. "
            "Build it once with: cd ~/PX4-Autopilot && make px4_sitl gz_x500"
        )
        return None

    log(f"PX4 executable: {px4_bin}")
    log(f"PX4 Gazebo plugin path: {px4_env['GZ_SIM_SYSTEM_PLUGIN_PATH']}")

    start_bg(
        [
            "env",
            "PX4_GZ_STANDALONE=1",
            "PX4_GZ_WORLD=" + gazebo_world_name,
            "PX4_SIM_MODEL=gz_x500",
            "PX4_SYS_AUTOSTART=4001",
            "PX4_GZ_MODEL_POSE=0,0,30,0,0,0",
            str(px4_bin),
        ],
        cwd=HOME / "PX4-Autopilot",
        env=px4_env,
        logfile=logdir / "px4.log",
        interactive=True,
    )

    time.sleep(5)

    log(
        f"Simulation started. World: {final_world}"
    )
    log(
        f"Logs: {logdir}"
    )

    return server

def process_world(world_path):
    terrain = parse_sdf(world_path)
    bbox = bbox_from_center_size(
        terrain["center_lat"], terrain["center_lon"],
        terrain["size_x"], terrain["size_y"],
        margin_m=0,
    )
    try:
        _ensure_esri_satellite_texture(terrain)
    except Exception as exc:
        log(f"WARNING: satellite ground repair failed: {exc}")

    WORK_ROOT.mkdir(parents=True, exist_ok=True)
    map_work = WORK_ROOT / world_path.parent.name
    map_work.mkdir(parents=True, exist_ok=True)

    osm_file = map_work / "osm.osm"
    clipped_osm_file = map_work / "osm_clipped.osm"
    roads_osm_file = map_work / "osm_roads_clipped.osm"
    raw_osm_glb = map_work / "osm_raw.glb"
    clean_osm_glb = map_work / "osm_objects_draped.glb"
    overture_raw = map_work / "overture_buildings.geojson"
    overture_norm = map_work / "overture_buildings_normalized.geojson"
    overture_places = map_work / "overture_places.geojson"
    overture_obj = map_work / "overture_buildings.obj"
    overture_manifest = map_work / "overture_building_styles.json"
    final_world = map_work / f"{world_path.stem}_drone_rakshak.world"

    log(f"Center: {terrain['center_lat']:.8f}, {terrain['center_lon']:.8f}")
    log(f"Terrain size: {terrain['size_x']:.2f} x {terrain['size_y']:.2f} m")
    log(f"BBOX: {bbox_str(bbox)}")
    log("Overture Buildings and Overture Places use the same selected BBOX.")

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
        gc.collect()

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
        log(f"Overture Places: {overture_places}")

    # Parse Overture Places now so the selected BBOX is automatically
    # associated with named places/campuses. The parsed landmarks are
    # consumed by the textured building exporter in the next stage.
    landmarks = _places_landmarks(overture_places)
    log(
        "Overture named places available for building matching: "
        f"{len(landmarks)}"
    )

    if not overture_norm.exists():
        count = write_overture_properties(overture_raw, overture_norm)
        log(f"Overture buildings: {count}")

    if not overture_obj.exists() or not overture_manifest.exists():
        build_overture_textured_obj(
            overture_norm,
            overture_obj,
            terrain,
            overture_places,
        )
        gc.collect()

    # Rebuild the final world if the textured building manifest is newer.
    if (
        not final_world.exists()
        or final_world.stat().st_mtime_ns
        < overture_manifest.stat().st_mtime_ns
    ):
        build_final_world(
            world_path,
            clean_osm_glb,
            overture_obj,
            final_world,
        )

    return final_world

def _handle_sigint(signum, frame):
    if SHUTDOWN_EVENT.is_set(): os._exit(130)
    SHUTDOWN_EVENT.set()
    log("Ctrl+C received: clean shutdown requested.")

def main():
    signal.signal(signal.SIGINT, _handle_sigint)
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-px4", action="store_true")
    ap.add_argument("--restart", action="store_true")
    args = ap.parse_args()

    WORK_ROOT.mkdir(parents=True, exist_ok=True)
    if args.restart:
        stop_sim()

    ensure_terrain_server()
    open_browser()

    # Do not use old worlds as triggers.
    before = snapshot_worlds()
    current = None
    proc_server = None

    try:
        while True:
            world = wait_for_new_world(before)
            if world is None:
                break
            before = snapshot_worlds()
            if current and world.resolve() == current.resolve() and not args.restart:
                continue

            log(f"Building Drone Rakshak environment for {world.parent.name}")
            try:
                final_world = process_world(world)
            except Exception as e:
                log(f"PIPELINE FAILED: {e}")
                log("The terrain UI remains available. Fix the reported issue and generate the world again.")
                continue

            stop_sim()
            final_work = WORK_ROOT / world.parent.name
            if args.no_px4:
                log("Launching Gazebo only (--no-px4).")
                env = os.environ.copy()
                env["QT_OPENGL"] = "software"
                env["LIBGL_ALWAYS_SOFTWARE"] = "1"
                env["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"
                start_bg(
                    ["gz", "sim", "-s", "-r", "-v", "4", str(final_world)],
                    logfile=final_work / "logs/gz_server.log",
                    env=env,
                )
                time.sleep(4)
                start_bg(
                    ["env", "-u", "WAYLAND_DISPLAY", "QT_QPA_PLATFORM=xcb", "gz", "sim", "-g", "-v", "4"],
                    logfile=final_work / "logs/gz_gui.log",
                    env=env,
                )
            else:
                start_sim(final_world, final_work)

            current = world

            # IMPORTANT:
            # Snapshot AFTER the current world has been processed.
            # The outer loop will then return to wait_for_new_world()
            # and detect either:
            #   - a new terrain folder, or
            #   - a changed existing terrain world.
            before = snapshot_worlds()

            log(
                "Ready. Generate another terrain in the browser. "
                "The current simulation will be stopped automatically "
                "and the new terrain will be loaded."
            )
    except KeyboardInterrupt:
        log("Stopping Drone Rakshak launcher.")
    finally:
        stop_tracked_processes(force=False)
        if proc_server:
            try:
                proc_server.terminate()
            except Exception:
                pass

if __name__ == "__main__":
    main()
