import rclpy
from rclpy.node import Node

class ScoreNode(Node):
    def __init__(self):
        super().__init__('score_node')
        self.get_logger().info("ScoreNode up!")

    def handle_get_score(self, req, resp):
        v = self.cache.get(req.metric, req.lat, req.lon, req.depth, req.stamp)
        if v is not None:
            resp.value = v
            resp.from_cache = True
            return resp

        # MISS: do the expensive dataset query
        v = self.query_backend(req.metric, req.lat, req.lon, req.depth, req.stamp)

        # store it
        self.cache.put(req.metric, req.lat, req.lon, req.depth, req.stamp, v)

        resp.value = v
        resp.from_cache = False
        return resp
    
    def query_backend(self, metric, lat, lon, depth, stamp):
        # Placeholder for actual backend query logic
        self.get_logger().info(f"Querying backend for {metric} at ({lat}, {lon}, {depth}) at {stamp}")
        # Simulate returning an array of 3 scores
        return {
            "current": 0.8,
            "algae": 0.5,
            "oxygen": 0.3
        }


def main(args=None):
    rclpy.init(args=args)
    node = ScoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
