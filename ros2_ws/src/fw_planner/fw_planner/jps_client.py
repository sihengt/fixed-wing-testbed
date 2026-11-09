"""Use a nav_msgs/GetPlan service (cf_drone's jps3d node) as a leg planner for fw_planner.planners."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

RETRIES = 30


class GetPlanClient:
    """leg(a, b) -> corner points [(x, y), ...] at flight altitude, or [(x, y, h), ...] for 3D poses (x, y, h, psi).

    plan_seconds sums the service time of the answers used; wait_seconds the rest (waiting for the map).
    """

    def __init__(self, node, altitude, obstacles, service='/plan', frame_id='px4_local_ned'):
        self.node, self.z, self.frame_id = node, -altitude, frame_id
        # Raw no-fly zones, to spot a planner that has no map yet: circles (x, y, r) or cylinders (x, y, r, bottom, top).
        self.obstacles = obstacles
        self.client = node.create_client(GetPlan, service)
        self.plan_seconds = self.wait_seconds = 0.0

    def leg(self, a, b):
        entered = time.monotonic()
        points = self._leg(a, b)
        self.wait_seconds += time.monotonic() - entered - self.last_seconds
        return points

    def _leg(self, a, b):
        if not self.client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'{self.client.srv_name} is not available')
        for _ in range(RETRIES):
            request = GetPlan.Request(start=self._pose(a), goal=self._pose(b))
            started = time.monotonic()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
            seconds = time.monotonic() - started
            poses = future.result().plan.poses
            points = [(p.pose.position.x, p.pose.position.y) + ((-p.pose.position.z,) if len(a) == 4 else ())
                      for p in poses]
            # The planner treats unknown space as free: a path through a no-fly circle means no map yet.
            if points and self._clear(points):
                self.plan_seconds += seconds
                self.last_seconds = seconds
                return points
            self.node.get_logger().info('jps3d gave no path or one through a no-fly zone (no map yet?), retrying')
            time.sleep(1.0)
        raise RuntimeError(f'jps3d found no path from {a} to {b}')

    def _pose(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.position.z = -pose[2] if len(pose) == 4 else self.z  # NED
        return msg

    def _clear(self, points):
        for a, b in zip(points, points[1:]):
            steps = max(1, int(math.dist(a, b)))
            for i in range(steps + 1):
                p = [u + (v - u) * i / steps for u, v in zip(a, b)]
                if any(math.hypot(p[0] - o[0], p[1] - o[1]) < o[2] and (len(p) == 2 or len(o) == 3 or o[3] <= p[2] <= o[4])
                       for o in self.obstacles):
                    return False
        return True
