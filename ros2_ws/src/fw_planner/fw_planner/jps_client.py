"""Use a nav_msgs/GetPlan service (cf_drone's jps3d node) as a leg planner for fw_planner.planners."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

RETRIES = 30


class GetPlanClient:
    """leg(a, b) -> corner points [(x, y), ...] at flight altitude. Sums the service time in plan_seconds."""

    def __init__(self, node, altitude, obstacles, service='/plan', frame_id='px4_local_ned'):
        self.node, self.z, self.frame_id = node, -altitude, frame_id
        self.obstacles = obstacles  # raw no-fly circles, to spot a planner that has no map yet
        self.client = node.create_client(GetPlan, service)
        self.plan_seconds = 0.0

    def leg(self, a, b):
        if not self.client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'{self.client.srv_name} is not available')
        for _ in range(RETRIES):
            request = GetPlan.Request(start=self._pose(a), goal=self._pose(b))
            started = time.monotonic()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
            seconds = time.monotonic() - started
            points = [(p.pose.position.x, p.pose.position.y) for p in future.result().plan.poses]
            # The planner treats unknown space as free: a path through a no-fly circle means no map yet.
            if points and self._clear(points):
                self.plan_seconds += seconds
                return points
            self.node.get_logger().info('jps3d gave no path or one through a no-fly zone (no map yet?), retrying')
            time.sleep(1.0)
        raise RuntimeError(f'jps3d found no path from {a} to {b}')

    def _pose(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[0], pose[1], self.z
        return msg

    def _clear(self, points):
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            steps = max(1, int(math.hypot(x1 - x0, y1 - y0)))
            for i in range(steps + 1):
                x, y = x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps
                if any(math.hypot(x - ox, y - oy) < r for ox, oy, r in self.obstacles):
                    return False
        return True
