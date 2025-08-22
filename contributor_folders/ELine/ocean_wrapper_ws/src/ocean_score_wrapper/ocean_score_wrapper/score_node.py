#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# ⬇️ Update this import to your real package name
# e.g. from ocean_interfaces.srv import GetOceanScore
from messages_88.srv import GetOceanScore

import ocean_score_wrapper.o2score as o2score

class _SimpleCache:
    """In-memory cache keyed by (metric, lat, lon, depth, stamp) -> float value."""
    def __init__(self):
        self._store = {}

    @staticmethod
    def _key(metric: str, lat: float, lon: float, depth: float, stamp: float):
        # Normalize floats for safer hashing (optional)
        return (str(metric), float(lat), float(lon), float(depth), float(stamp))

    def get(self, metric, lat, lon, depth, stamp):
        return self._store.get(self._key(metric, lat, lon, depth, stamp))

    def put(self, metric, lat, lon, depth, stamp, value: float):
        self._store[self._key(metric, lat, lon, depth, stamp)] = float(value)


class OceanDecisionNode(Node):
    """
    Publishes a service `/ocean_decision` with type GetOceanScore.
    Request:
        string metric
        float64 lat, lon, depth, stamp
    Response:
        float64 value
        bool from_cache
    """
    def __init__(self):
        super().__init__('ocean_decision_node')
        self.cache = _SimpleCache()

        # Advertise service
        self._srv = self.create_service(
            GetOceanScore,
            'ocean_decision',
            self.handle_get_ocean_score
        )
        self.get_logger().info("OceanDecisionNode up! Service [/ocean_decision] ready.")

    def handle_get_ocean_score(self, req: GetOceanScore.Request, resp: GetOceanScore.Response):
        # 1) Cache lookup
        cached = self.cache.get(req.metric, req.lat, req.lon, req.depth, req.stamp)
        if cached is not None:
            resp.value = float(cached)
            resp.from_cache = True
            return resp

        # 2) Cache miss → expensive compute / backend call
        value = self.query_backend(req.metric, req.lat, req.lon, req.depth, req.stamp)

        # 3) Store & return
        self.cache.put(req.metric, req.lat, req.lon, req.depth, req.stamp, value)
        resp.value = float(value)
        resp.from_cache = False
        return resp

    def query_backend(self, metric: str, lat: float, lon: float, depth: float, stamp: float) -> float:
        """
        Placeholder for real data source (DB/model/another ROS service).
        Return a single float score for the requested metric.
        """
        self.get_logger().info(
            f"[ocean_decision] Querying backend for metric='{metric}' "
            f"at (lat={lat:.6f}, lon={lon:.6f}, depth={depth}) @ t={stamp:.3f}s"
        )
        # Example heuristic: pick from known metrics or fallback to 0.0
        scores = {
            "current": 0.8,
            "algae":   0.5,
            "oxygen":  0.3,
        }
        return float(scores.get(metric, 0.0))


def main(args=None):
    rclpy.init(args=args)
    node = OceanDecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
