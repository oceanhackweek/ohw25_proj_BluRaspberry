from typing import List, Tuple
import math

DEG_LAT_M = 111_111.0

def seg_len_m(a: Tuple[float,float], b: Tuple[float,float]) -> float:
    lat_m = (b[0] - a[0]) * DEG_LAT_M
    lon_m = (b[1] - a[1]) * DEG_LAT_M * math.cos(math.radians(a[0]))
    return math.hypot(lat_m, lon_m)

def lerp(a: Tuple[float,float], b: Tuple[float,float], f: float) -> Tuple[float,float]:
    return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))

def build_stations(polyline: List[Tuple[float,float]], spacing_m: float, min_leg_m: float = 5.0) -> List[Tuple[float,float]]:
    """
    Vertices are always stations. Within each segment, place stations every `spacing_m`.
    Spacing resets at each vertex (no leftover carried across vertices).
    """
    # De-dupe consecutive identical vertices
    verts: List[Tuple[float,float]] = []
    for p in polyline:
        if not verts or (abs(p[0]-verts[-1][0]) > 1e-12 or abs(p[1]-verts[-1][1]) > 1e-12):
            verts.append(p)
    if len(verts) < 2:
        return verts

    stations: List[Tuple[float,float]] = [verts[0]]
    for i in range(len(verts)-1):
        a, b = verts[i], verts[i+1]
        L = seg_len_m(a, b)
        if L <= 1e-9:
            if seg_len_m(stations[-1], b) >= 1e-6:
                stations.append(b)
            continue

        # place evenly spaced points strictly inside the segment
        if spacing_m > 0:
            n = int(L // spacing_m)
            for k in range(1, n+1):
                d = k * spacing_m
                if d >= L - 1e-6:
                    break
                f = d / L
                pt = lerp(a, b, f)
                if seg_len_m(stations[-1], pt) >= min_leg_m:
                    stations.append(pt)

        # always include the segment endpoint vertex
        if seg_len_m(stations[-1], b) >= 1e-6:
            stations.append(b)

    return stations

def generate_vertical_lawnmower(
    polyline: List[Tuple[float, float]],
    depth_min: float,
    depth_max: float,
    horizontal_leg_spacing: float,
    vertical_point_spacing: float,
    start_down: bool = True,
) -> List[Tuple[float, float, float]]:
    """
    Pattern:
      - Start at polyline[0].
      - Vertical sweep (down if start_down else up) between [depth_min, depth_max].
      - Horizontal transit to next station at the *ended* depth.
      - Stations are: every vertex + evenly spaced points each `horizontal_leg_spacing`
        along each segment, with spacing RESET at each vertex (no leftover).
      - Alternate sweep direction at each station. No duplicate triplets.
    """
    if len(polyline) < 2:
        return []

    # --- stations (this is the key change) ---
    stations = build_stations(polyline, float(max(1e-6, horizontal_leg_spacing)), min_leg_m=5.0)

    # --- depth ladder (inclusive) ---
    lo, hi = (depth_min, depth_max) if depth_min <= depth_max else (depth_max, depth_min)
    vstep = float(max(1e-6, vertical_point_spacing))

    ladder = []
    d = lo
    while d < hi - 1e-12:
        ladder.append(d)
        d += vstep
    if not ladder or ladder[-1] != hi:
        ladder.append(hi)

    ladder_down = ladder                  # lo -> hi
    ladder_up   = list(reversed(ladder))  # hi -> lo

    waypoints: List[Tuple[float, float, float]] = []
    going_down = start_down

    def append_wp(lat: float, lon: float, dep: float):
        if waypoints and abs(waypoints[-1][0]-lat) < 1e-12 and \
                        abs(waypoints[-1][1]-lon) < 1e-12 and \
                        abs(waypoints[-1][2]-dep) < 1e-9:
            return
        waypoints.append((lat, lon, dep))

    for i, (lat, lon) in enumerate(stations):
        seq = ladder_down if going_down else ladder_up

        # If we just transited to (lat,lon,seq[0]), skip duplicate first depth
        if waypoints and abs(waypoints[-1][0]-lat) < 1e-12 and \
                        abs(waypoints[-1][1]-lon) < 1e-12 and \
                        abs(waypoints[-1][2]-seq[0]) < 1e-9:
            sweep = seq[1:]
        else:
            sweep = seq

        for dep in sweep:
            append_wp(lat, lon, dep)

        ended_depth = sweep[-1] if sweep else (hi if going_down else lo)

        # Horizontal transit to next station holding ended_depth
        if i < len(stations) - 1:
            nlat, nlon = stations[i + 1]
            append_wp(nlat, nlon, ended_depth)

        going_down = not going_down

    return waypoints
