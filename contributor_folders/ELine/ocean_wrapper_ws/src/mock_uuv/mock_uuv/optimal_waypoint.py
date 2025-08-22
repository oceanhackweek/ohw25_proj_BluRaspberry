import math
from typing import Tuple, Dict, Optional, List
import rclpy
import threading

DEG_LAT_M = 111_320.0  # meters per degree latitude (coarse WGS84 approximation)

from messages_88.srv import GetOceanScore 

# Internal singletons (hidden; your external API stays functional)
__node = None
__cli = None
__lock = threading.Lock()
__TIMEOUT = 3.0  # seconds

def select_best_target(
    target_lat: float,
    target_lon: float,
    target_depth_m: float,
    my_lat: float,
    my_lon: float,
    my_depth_m: float,
    my_heading_deg: float,
    *,
    radius_m: float = 50.0,
    num_points: int = 10,
    weights: Optional[Dict[str, float]] = None,
) -> Tuple[float, float, float]:
    """
    Choose the best nearby point around `target_(lat,lon,depth)` by sampling `num_points`
    equally spaced azimuths on a circle of `radius_m` (fixed depth), then scoring each
    candidate by a weighted average of:
      - minimize distance from my current position (3D meters),
      - maximize algal bloom probability (via ROS2 service),
      - minimize vector difference to currents (i.e., maximize heading alignment).

    Args:
        target_lat, target_lon: degrees
        target_depth_m: meters (positive down)
        my_lat, my_lon: degrees
        my_depth_m: meters (positive down)
        my_heading_deg: degrees, 0 = North, 90 = East (navigation convention)
        radius_m: sampling circle radius around the target
        num_points: number of samples on the circle
        weights: dict with keys {"distance","bloom","alignment"} summing to ~1.0
                 defaults to equal weights if not provided

    Returns:
        (best_lat, best_lon, best_depth_m)
    """
    if weights is None:
        weights = {"distance": 1/3, "algae": 1/3, "currents": 1/3}

    # Helper: meters per degree longitude varies with latitude
    def meters_per_deg_lon(lat_deg: float) -> float:
        return DEG_LAT_M * math.cos(math.radians(lat_deg))

    # Generate candidate points on the circumference of the circle around target
    ring_points: List[Tuple[float, float, float]] = []
    lon_m = meters_per_deg_lon(target_lat)
    for i in range(num_points):
        theta = 2.0 * math.pi * (i / num_points)  # 0..2π
        dx_east = radius_m * math.sin(theta)      # ENU: x=east, y=north
        dy_north = radius_m * math.cos(theta)

        dlat = dy_north / DEG_LAT_M
        dlon = dx_east / lon_m
        ring_points.append((target_lat + dlat, target_lon + dlon, target_depth_m))

    # Score each candidate
    distances: List[float] = []
    blooms: List[float] = []
    aligns: List[float] = []

    my_lon_m = meters_per_deg_lon(my_lat)

    for (lat, lon, dep) in ring_points:
        # 3D distance (meters) using local tangent plane approximation
        dx = (lon - my_lon) * my_lon_m
        dy = (lat - my_lat) * DEG_LAT_M
        dz = dep - my_depth_m
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        distances.append(dist)

        # Algal bloom probability (0..1 expected; treat unknown as neutral 0.5)
        try:
            p_bloom = get_algal_bloom_probability(lat, lon, dep)
            if p_bloom is None:
                p_bloom = 0.5
            p_bloom = max(0.0, min(1.0, float(p_bloom)))
        except NotImplementedError:
            p_bloom = 0.5
        blooms.append(p_bloom)

        # Current alignment score: 1.0 when perfectly aligned, 0.0 when opposite
        try:
            u_east, v_north = get_current_vector(lat, lon, dep)  # m/s
            curr_heading_deg = heading_from_east_north(u_east, v_north)
            angle_diff = angular_difference_deg(my_heading_deg, curr_heading_deg)  # 0..180
            align = 1.0 - (angle_diff / 180.0)
        except NotImplementedError:
            align = 0.5  # neutral if unavailable
        aligns.append(align)

    # Normalize distance so that smaller distance -> higher score in [0,1]
    d_min, d_max = min(distances), max(distances)
    if d_max == d_min:
        dist_scores = [1.0 for _ in distances]
    else:
        dist_scores = [(d_max - d) / (d_max - d_min) for d in distances]

    # Bloom and alignment already in [0,1]
    bloom_scores = blooms
    align_scores = aligns

    # Weighted sum
    total_scores = [
        weights.get("distance", 0.0) * ds
        + weights.get("bloom", 0.0) * bs
        + weights.get("alignment", 0.0) * ascore
        for ds, bs, ascore in zip(dist_scores, bloom_scores, align_scores)
    ]

    # Pick best
    best_idx = max(range(num_points), key=lambda i: total_scores[i])
    return ring_points[best_idx]


# ---------------------------
# Utility helpers (pure math)
# ---------------------------

def heading_from_east_north(u_east: float, v_north: float) -> float:
    """
    Convert a vector expressed in ENU components (east, north) to a navigation heading in degrees:
    0° = North, 90° = East, 180° = South, 270° = West.
    """
    ang_rad = math.atan2(u_east, v_north)  # note order for nav convention
    ang_deg = math.degrees(ang_rad)
    if ang_deg < 0:
        ang_deg += 360.0
    return ang_deg

def angular_difference_deg(a_deg: float, b_deg: float) -> float:
    """
    Smallest absolute difference between two headings (degrees), in [0, 180].
    """
    diff = abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)
    return diff


# ---------------------------
# ROS2 service stubs
# ---------------------------

def __ensure_client():
    """Lazily init rclpy, a tiny client node, and the /ocean_decision client."""
    global __node, __cli
    with __lock:
        if not rclpy.ok():
            rclpy.init(args=None)
        if __node is None:
            __node = rclpy.create_node('ocean_decision_client')
        if __cli is None:
            __cli = __node.create_client(GetOceanScore, 'ocean_decision')
            __cli.wait_for_service(timeout_sec=__TIMEOUT)

def __score(metric: str, lat: float, lon: float, depth_m: float, stamp: float = 0.0) -> Optional[float]:
    """Call /ocean_decision for a single scalar metric. Return float or None on failure."""
    __ensure_client()
    req = GetOceanScore.Request()
    req.metric = metric
    req.lat = float(lat)
    req.lon = float(lon)
    req.depth = float(depth_m)
    req.stamp = float(stamp)

    fut = __cli.call_async(req)
    ok = rclpy.spin_until_future_complete(__node, fut, timeout_sec=__TIMEOUT)
    if not ok or fut.result() is None:
        return None
    return float(fut.result().value)

def get_algal_bloom_probability(lat: float, lon: float, depth_m: float) -> Optional[float]:
    """
    Calls /ocean_decision with metric='algae'.
    Returns float in [0,1] or None if unavailable (your caller already treats None as neutral 0.5).
    """
    return __score('algae', lat, lon, depth_m, 0.0)

def get_current_vector(lat: float, lon: float, depth_m: float) -> Tuple[float, float]:
    """
    Calls /ocean_decision twice ('current_u' and 'current_v') and returns (u_east, v_north) in m/s.
    On failure, raises NotImplementedError so your existing try/except path stays the same.
    """
    u = __score('current_u', lat, lon, depth_m, 0.0)
    v = __score('current_v', lat, lon, depth_m, 0.0)
    if u is None or v is None:
        raise NotImplementedError("currents unavailable")
    return (u, v)