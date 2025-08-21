import math
from typing import Tuple, Dict, Optional, List

DEG_LAT_M = 111_320.0  # meters per degree latitude (coarse WGS84 approximation)

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

def get_algal_bloom_probability(lat: float, lon: float, depth_m: float) -> Optional[float]:
    """
    STUB: Replace with a ROS2 service call to obtain algal bloom probability at (lat,lon,depth).
    Expected return: float in [0,1]. Return None if unavailable.
    Example integration sketch (pseudo-code):
        import rclpy
        from my_msgs.srv import BloomProb
        ...
        client.call_async(BloomProb.Request(lat=lat, lon=lon, depth=depth_m))
    """
    raise NotImplementedError("Connect to ROS2 service for algal bloom probability")

def get_current_vector(lat: float, lon: float, depth_m: float) -> Tuple[float, float]:
    """
    STUB: Replace with a ROS2 service call to obtain local water current vector at (lat,lon,depth).
    Expected return: (u_east, v_north) in m/s.
    Example integration sketch (pseudo-code):
        import rclpy
        from my_msgs.srv import Currents
        ...
        client.call_async(Currents.Request(lat=lat, lon=lon, depth=depth_m))
    """
    raise NotImplementedError("Connect to ROS2 service for currents (east/north)")
