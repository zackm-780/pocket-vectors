"""Source data access: naming, availability lists, cached downloads.

Verified layouts (see README.md, "Verified source layouts"):

* GLO-30:  https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_{N|S}{lat:02d}_00_{E|W}{lon:03d}_00_DEM/<same name>.tif
* GLO-90:  https://copernicus-dem-90m.s3.amazonaws.com/Copernicus_DSM_COG_30_{N|S}{lat:02d}_00_{E|W}{lon:03d}_00_DEM/<same name>.tif
  lat/lon in the name are the SOUTH-WEST integer corner of the 1 degree cell. A cell that is not
  in the bucket's tileList.txt is open ocean (no tile published).
* WorldCover: https://esa-worldcover.s3.amazonaws.com/v200/2021/map/ESA_WorldCover_10m_2021_v200_{N|S}{lat:02d}{E|W}{lon:03d}_Map.tif
  3 x 3 degree tiles named by their SW corner. Tiles with no land are not published.
"""
from __future__ import annotations

import os
import re
import sys
import time
from dataclasses import dataclass

import requests

DEM_BUCKETS = {
    "glo30": ("copernicus-dem-30m", "10"),
    "glo90": ("copernicus-dem-90m", "30"),
}
WC_BUCKET = "esa-worldcover"
WC_PREFIX = "v200/2021/map/"
WC_SPAN_DEG = 3

# Rough average object sizes, used only by --dry-run estimates (measured on the Bay Area:
# GLO-30 20-49 MB per land cell, GLO-90 about 2.4 MB, WorldCover 4-115 MB per 3-degree tile).
EST_MB_PER_CELL = {"glo30": 30.0, "glo90": 2.0}
EST_MB_PER_WC_TILE = 60.0

USER_AGENT = "pocket-vectors-terrain/1.0"


def cell_name(product: str, lat0: int, lon0: int) -> str:
    code = DEM_BUCKETS[product][1]
    ns = "N" if lat0 >= 0 else "S"
    ew = "E" if lon0 >= 0 else "W"
    return f"Copernicus_DSM_COG_{code}_{ns}{abs(lat0):02d}_00_{ew}{abs(lon0):03d}_00_DEM"


_CELL_RE = re.compile(r"_([NS])(\d{2})_00_([EW])(\d{3})_00_DEM")
_WC_RE = re.compile(r"_([NS])(\d{2})([EW])(\d{3})_Map\.tif")


def parse_cell_name(name: str):
    m = _CELL_RE.search(name)
    if not m:
        return None
    lat0 = int(m.group(2)) * (1 if m.group(1) == "N" else -1)
    lon0 = int(m.group(4)) * (1 if m.group(3) == "E" else -1)
    return lat0, lon0


def wc_tile_name(lat0: int, lon0: int) -> str:
    ns = "N" if lat0 >= 0 else "S"
    ew = "E" if lon0 >= 0 else "W"
    return f"ESA_WorldCover_10m_2021_v200_{ns}{abs(lat0):02d}{ew}{abs(lon0):03d}_Map.tif"


def wc_tile_origin(lat_deg: float, lon_deg: float):
    """SW corner (lat0, lon0) of the 3-degree WorldCover tile containing the point."""
    import math
    return (math.floor(lat_deg / WC_SPAN_DEG) * WC_SPAN_DEG,
            math.floor(lon_deg / WC_SPAN_DEG) * WC_SPAN_DEG)


class OfflineMiss(RuntimeError):
    """A needed file is not in the cache and --offline was given."""


@dataclass
class Sources:
    cache_dir: str
    offline: bool = False
    verbose: bool = True

    def __post_init__(self):
        self._lists: dict[str, set] = {}
        self.session = None

    # ------------------------------------------------------------------ paths
    def dem_path(self, product: str, lat0: int, lon0: int) -> str:
        return os.path.join(self.cache_dir, product, cell_name(product, lat0, lon0) + ".tif")

    def dem_url(self, product: str, lat0: int, lon0: int) -> str:
        n = cell_name(product, lat0, lon0)
        return f"https://{DEM_BUCKETS[product][0]}.s3.amazonaws.com/{n}/{n}.tif"

    def wc_path(self, lat0: int, lon0: int) -> str:
        return os.path.join(self.cache_dir, "worldcover", wc_tile_name(lat0, lon0))

    def wc_url(self, lat0: int, lon0: int) -> str:
        return f"https://{WC_BUCKET}.s3.amazonaws.com/{WC_PREFIX}{wc_tile_name(lat0, lon0)}"

    # ------------------------------------------------------------------ http
    def _session(self) -> requests.Session:
        if self.session is None:
            self.session = requests.Session()
            self.session.headers["User-Agent"] = USER_AGENT
        return self.session

    def _get(self, url: str, **kw) -> requests.Response:
        last = None
        for attempt in range(5):
            try:
                r = self._session().get(url, timeout=(15, 120), **kw)
                if r.status_code in (500, 502, 503, 504):
                    raise requests.HTTPError(f"HTTP {r.status_code}")
                return r
            except (requests.RequestException, OSError) as e:  # noqa: PERF203
                last = e
                time.sleep(min(2 ** attempt, 20))
        raise RuntimeError(f"GET {url} failed after retries: {last}")

    def download(self, url: str, dest: str) -> bool:
        """Download url to dest atomically. Returns False on HTTP 404."""
        if os.path.exists(dest):
            return True
        if self.offline:
            raise OfflineMiss(f"{dest} is not cached and --offline was given")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        part = f"{dest}.{os.getpid()}.part"
        for attempt in range(4):
            try:
                with self._get(url, stream=True) as r:
                    if r.status_code == 404:
                        return False
                    r.raise_for_status()
                    want = int(r.headers.get("Content-Length", -1))
                    got = 0
                    with open(part, "wb") as f:
                        for chunk in r.iter_content(1 << 20):
                            f.write(chunk)
                            got += len(chunk)
                    if want >= 0 and got != want:
                        raise OSError(f"short read {got}/{want}")
                os.replace(part, dest)
                return True
            except (requests.RequestException, OSError) as e:
                if os.path.exists(part):
                    os.remove(part)
                if attempt == 3:
                    raise RuntimeError(f"download of {url} failed: {e}") from e
                time.sleep(2 ** attempt)
        return False

    # ------------------------------------------------------------------ availability lists
    def dem_cells(self, product: str) -> set:
        """Set of (lat0, lon0) cells published for the product (ocean cells are absent)."""
        if product in self._lists:
            return self._lists[product]
        path = os.path.join(self.cache_dir, "lists", f"{product}_tileList.txt")
        if not os.path.exists(path):
            if self.offline:
                raise OfflineMiss(f"{path} not cached; run once online to fetch the tile list")
            bucket = DEM_BUCKETS[product][0]
            os.makedirs(os.path.dirname(path), exist_ok=True)
            if not self.download(f"https://{bucket}.s3.amazonaws.com/tileList.txt", path):
                raise RuntimeError(f"tileList.txt not found in bucket {bucket}")
        cells = set()
        with open(path) as f:
            for line in f:
                c = parse_cell_name(line)
                if c:
                    cells.add(c)
        self._lists[product] = cells
        return cells

    def wc_tiles(self) -> set:
        """Set of (lat0, lon0) 3-degree WorldCover tiles published (ocean-only tiles absent)."""
        if "wc" in self._lists:
            return self._lists["wc"]
        path = os.path.join(self.cache_dir, "lists", "worldcover_tiles.txt")
        if not os.path.exists(path):
            if self.offline:
                raise OfflineMiss(f"{path} not cached; run once online to fetch the tile list")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            keys, token = [], None
            while True:
                params = {"list-type": "2", "prefix": WC_PREFIX}
                if token:
                    params["continuation-token"] = token
                r = self._get(f"https://{WC_BUCKET}.s3.amazonaws.com/", params=params)
                r.raise_for_status()
                keys += re.findall(r"<Key>([^<]+)</Key>", r.text)
                m = re.search(r"<NextContinuationToken>([^<]+)</NextContinuationToken>", r.text)
                if "<IsTruncated>true</IsTruncated>" in r.text and m:
                    token = m.group(1)
                else:
                    break
            with open(path, "w") as f:
                f.write("\n".join(k for k in keys if k.endswith("_Map.tif")) + "\n")
        tiles = set()
        with open(path) as f:
            for line in f:
                m = _WC_RE.search(line)
                if m:
                    lat0 = int(m.group(2)) * (1 if m.group(1) == "N" else -1)
                    lon0 = int(m.group(4)) * (1 if m.group(3) == "E" else -1)
                    tiles.add((lat0, lon0))
        self._lists["wc"] = tiles
        return tiles

    # ------------------------------------------------------------------ cached fetch
    def get_dem(self, product: str, lat0: int, lon0: int):
        """Local path of the DEM cell, downloading if needed. None if the cell is not published."""
        if (lat0, lon0) not in self.dem_cells(product):
            return None
        path = self.dem_path(product, lat0, lon0)
        if not os.path.exists(path):
            if self.verbose:
                print(f"  download {os.path.basename(path)}", file=sys.stderr, flush=True)
            if not self.download(self.dem_url(product, lat0, lon0), path):
                return None
        return path

    def get_wc(self, lat0: int, lon0: int):
        if (lat0, lon0) not in self.wc_tiles():
            return None
        path = self.wc_path(lat0, lon0)
        if not os.path.exists(path):
            if self.verbose:
                print(f"  download {os.path.basename(path)}", file=sys.stderr, flush=True)
            if not self.download(self.wc_url(lat0, lon0), path):
                return None
        return path
