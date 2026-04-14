# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry
# from geometry_msgs.msg import PoseStamped
# import math
# from collections import deque
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from std_msgs.msg import Bool

# BLACKLIST_RADIUS = 0.3  # reduced from 0.5

# class SimpleExplorer(Node):
#     def __init__(self):
#         super().__init__('simple_explorer')

#         self.active_sub = self.create_subscription(Bool, '/explorer_active', self.active_callback, 10)
#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

#         self.is_active = True
#         self.map_msg = None
#         self.pos = (0.0, 0.0)
#         self.last_pos = (0.0, 0.0)
#         self.timer = self.create_timer(3.0, self.explore)
#         self.current_goal = None
#         self.current_score = 0.0

#         self.stuck_counter = 0
#         self.blacklist = deque(maxlen=20)
#         self.goal_in_progress = False
#         self._goal_handle = None

#     def active_callback(self, msg):
#         self.is_active = msg.data
#         if not self.is_active and self._goal_handle:
#             self._goal_handle.cancel_goal_async()
#             self.goal_in_progress = False

#     def map_callback(self, msg):
#         self.map_msg = msg
#         self.res = self.map_msg.info.resolution
#         self.origin_x = self.map_msg.info.origin.position.x
#         self.origin_y = self.map_msg.info.origin.position.y
#         self.w = self.map_msg.info.width

#     def odom_callback(self, msg):
#         self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

#     def get_world_coords(self, index):
#         return (self.origin_x + (index % self.w) * self.res,
#                 self.origin_y + (index // self.w) * self.res)

#     def cluster_frontiers(self, frontiers_coords):
#         CLUSTER_RADIUS = 0.6
#         bucket_size = CLUSTER_RADIUS

#         buckets = {}
#         for point in frontiers_coords:
#             key = (int(point[0] / bucket_size), int(point[1] / bucket_size))
#             buckets.setdefault(key, []).append(point)

#         visited = set()
#         clusters = []

#         for start in frontiers_coords:
#             if start in visited:
#                 continue

#             cluster = []
#             queue = deque([start])
#             visited.add(start)

#             while queue:
#                 point = queue.popleft()
#                 cluster.append(point)

#                 bx = int(point[0] / bucket_size)
#                 by = int(point[1] / bucket_size)

#                 for dx in (-1, 0, 1):
#                     for dy in (-1, 0, 1):
#                         for neighbour in buckets.get((bx + dx, by + dy), []):
#                             if neighbour not in visited and math.dist(point, neighbour) < CLUSTER_RADIUS:
#                                 visited.add(neighbour)
#                                 queue.append(neighbour)

#             clusters.append(cluster)

#         return clusters

#     def explore(self):
#         if not self.is_active:
#             return

#         # Stuck detection
#         if self.current_goal is not None:
#             moved = math.dist(self.pos, self.last_pos)
#             if moved < 0.1:
#                 self.stuck_counter += 1
#             else:
#                 self.stuck_counter = 0
#         self.last_pos = self.pos

#         if self.stuck_counter >= 10:
#             self.get_logger().error("STUCK — blacklisting goal and cancelling navigation")
#             if self.current_goal:
#                 self.blacklist.append(self.current_goal)
#             self.current_goal = None
#             self.current_score = 0.0
#             self.stuck_counter = 0
#             self.goal_in_progress = False
#             self.last_pos = self.pos
#             if self._goal_handle:
#                 self._goal_handle.cancel_goal_async()
#             self._goal_handle = None
#             return

#         if not self.map_msg or self.goal_in_progress:
#             return

#         # Find frontiers
#         grid = self.map_msg.data
#         w, h = self.map_msg.info.width, self.map_msg.info.height
#         frontiers_coords = []

#         for y in range(1, h - 1):
#             for x in range(1, w - 1):
#                 i = y * w + x
#                 if 0 < grid[i] < 45:
#                     continue
#                 neighbors = [
#                     grid[y * w + (x - 1)],
#                     grid[y * w + (x + 1)],
#                     grid[(y - 1) * w + x],
#                     grid[(y + 1) * w + x],
#                 ]
#                 if -1 in neighbors:
#                     frontiers_coords.append(self.get_world_coords(i))

#         if not frontiers_coords:
#             self.get_logger().info("Exploration Complete!")
#             return

#         clusters = self.cluster_frontiers(frontiers_coords)

#         # Debug logging
#         self.get_logger().info(f"Frontiers: {len(frontiers_coords)} | Clusters: {len(clusters)} | Blacklist: {len(self.blacklist)}")

#         scored_goals = []
#         for cluster in clusters:
#             avg_x = sum(p[0] for p in cluster) / len(cluster)
#             avg_y = sum(p[1] for p in cluster) / len(cluster)
#             centre = (avg_x, avg_y)
#             dist = math.dist(centre, self.pos)
#             size = len(cluster)

#             if size < 2: continue
#             if dist < 0.05: continue  # reduced from 0.1
#             if any(math.dist(centre, b) < BLACKLIST_RADIUS for b in self.blacklist): continue

#             utility = size / (dist + 0.1)
#             scored_goals.append((utility, centre))

#         if not scored_goals:
#             self.get_logger().warn("No goals after filtering — clearing blacklist!")
#             self.blacklist.clear()  # FIX: don't get stuck forever
#             return

#         # Clear current goal if we've arrived
#         if self.current_goal is not None and math.dist(self.pos, self.current_goal) < 0.4:
#             self.current_goal = None
#             self.current_score = 0.0

#         best_score, best_goal = max(scored_goals, key=lambda x: x[0])

#         # Don't resend if goal hasn't changed much
#         if self.current_goal is not None and math.dist(best_goal, self.current_goal) < 0.1:  # reduced from 0.2
#             return

#         self.current_goal = best_goal
#         self.current_score = best_score
#         self.send_navigation_goal(best_goal)

#     def send_navigation_goal(self, coords):
#         if not self.nav_client.wait_for_server(timeout_sec=30.0):  # increased from 5.0
#             self.get_logger().error("NavigateToPose action server not available!")
#             return

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = coords[0]
#         goal_msg.pose.pose.position.y = coords[1]
#         goal_msg.pose.pose.orientation.w = 1.0

#         self.get_logger().info(f"Sending goal: {coords}")
#         self.goal_in_progress = True
#         send_goal_future = self.nav_client.send_goal_async(goal_msg)
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by Nav2!")
#             self.goal_in_progress = False
#             self.current_goal = None
#             return  # FIX: was missing, caused crash on rejection
#         self._goal_handle = goal_handle
#         self.get_result_future = goal_handle.get_result_async()
#         self.get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result()
#         self.get_logger().info(f"Goal result: {result.status}")
#         self.current_goal = None
#         self.goal_in_progress = False
#         self._goal_handle = None
#         self.get_logger().info("Goal finished, finding next frontier.")

# def main():
#     rclpy.init()
#     rclpy.spin(SimpleExplorer())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import math
from collections import deque
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from enum import Enum

# ── Tunables ──────────────────────────────────────────────────────────────────
# Increased to 65 since your map's "free" space is 30-50.
OCC_THRESHOLD     = 65
MIN_FRONTIER_SIZE = 5
BLACKLIST_RADIUS  = 0.35
STUCK_TICKS       = 8      # × 3s = 24s before declaring stuck
GOAL_ARRIVED_DIST = 0.4
SCAN_SPEED        = 0.6    # rad/s for 360 scan
SCAN_DURATION     = 11.0   # seconds for full 360
# ─────────────────────────────────────────────────────────────────────────────

class PointClassification(Enum):
    MapOpen        = 1
    MapClosed      = 2
    FrontierOpen   = 4
    FrontierClosed = 8

class FrontierPoint:
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

class FrontierCache:
    def __init__(self):
        self.cache = {}

    def getPoint(self, x, y):
        # Cantor pairing function for unique indexing
        idx = ((x + y) * (x + y + 1) // 2) + y
        if idx not in self.cache:
            self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def clear(self):
        self.cache = {}

class OccupancyGrid2d:
    def __init__(self, msg):
        self.msg = msg

    def getCost(self, mx, my):
        return self.msg.data[my * self.msg.info.width + mx]

    def getSizeX(self):
        return self.msg.info.width

    def getSizeY(self):
        return self.msg.info.height

    def mapToWorld(self, mx, my):
        res = self.msg.info.resolution
        ox  = self.msg.info.origin.position.x
        oy  = self.msg.info.origin.position.y
        return (ox + (mx + 0.5) * res,
                oy + (my + 0.5) * res)

    def worldToMap(self, wx, wy):
        res = self.msg.info.resolution
        ox  = self.msg.info.origin.position.x
        oy  = self.msg.info.origin.position.y
        return (int((wx - ox) / res),
                int((wy - oy) / res))

def _neighbors(point, grid2d, cache):
    pts = []
    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if 0 <= x < grid2d.getSizeX() and 0 <= y < grid2d.getSizeY():
                if x == point.mapX and y == point.mapY:
                    continue
                pts.append(cache.getPoint(x, y))
    return pts

def _is_frontier(point, grid2d, cache):
    # A frontier cell must be Unknown (-1)
    if grid2d.getCost(point.mapX, point.mapY) != -1:
        return False
    
    has_free_neighbor = False
    for n in _neighbors(point, grid2d, cache):
        c = grid2d.getCost(n.mapX, n.mapY)
        # If any neighbor is a wall (> threshold), this is a dangerous frontier
        if c > OCC_THRESHOLD:
            return False
        # If neighbor is "Known Passable" (0 to Threshold)
        if 0 <= c <= OCC_THRESHOLD:
            has_free_neighbor = True
            
    return has_free_neighbor

def _find_free(mx, my, grid2d, cache):
    """Finds the nearest 'passable' cell to start the WFD search."""
    bfs = [cache.getPoint(mx, my)]
    visited = {((mx + my) * (mx + my + 1) // 2) + my}
    
    while bfs:
        loc = bfs.pop(0)
        cost = grid2d.getCost(loc.mapX, loc.mapY)
        # Change: Accept 0-OCC_THRESHOLD instead of just 0
        if 0 <= cost <= OCC_THRESHOLD:
            return (loc.mapX, loc.mapY)
            
        for n in _neighbors(loc, grid2d, cache):
            idx = ((n.mapX + n.mapY) * (n.mapX + n.mapY + 1) // 2) + n.mapY
            if idx not in visited:
                visited.add(idx)
                bfs.append(n)
    return (mx, my)

def get_frontiers(robot_pos, grid2d):
    cache = FrontierCache()
    mx, my = grid2d.worldToMap(robot_pos[0], robot_pos[1])
    fx, fy = _find_free(mx, my, grid2d, cache)

    start = cache.getPoint(fx, fy)
    start.classification = PointClassification.MapOpen.value
    map_queue = [start]
    frontiers = []

    while map_queue:
        p = map_queue.pop(0)
        if p.classification & PointClassification.MapClosed.value:
            continue

        if _is_frontier(p, grid2d, cache):
            p.classification |= PointClassification.FrontierOpen.value
            f_queue   = [p]
            new_front = []

            while f_queue:
                q = f_queue.pop(0)
                if q.classification & (PointClassification.MapClosed.value |
                                       PointClassification.FrontierClosed.value):
                    continue
                if _is_frontier(q, grid2d, cache):
                    new_front.append(q)
                    for w in _neighbors(q, grid2d, cache):
                        if not (w.classification & (
                                PointClassification.FrontierOpen.value |
                                PointClassification.FrontierClosed.value |
                                PointClassification.MapClosed.value)):
                            w.classification |= PointClassification.FrontierOpen.value
                            f_queue.append(w)
                q.classification |= PointClassification.FrontierClosed.value

            if len(new_front) >= MIN_FRONTIER_SIZE:
                coords = [grid2d.mapToWorld(fp.mapX, fp.mapY) for fp in new_front]
                cx = sum(c[0] for c in coords) / len(coords)
                cy = sum(c[1] for c in coords) / len(coords)
                frontiers.append((cx, cy, len(new_front)))

            for fp in new_front:
                fp.classification |= PointClassification.MapClosed.value

        for v in _neighbors(p, grid2d, cache):
            if not (v.classification & (PointClassification.MapOpen.value |
                                        PointClassification.MapClosed.value)):
                # Change: check for passable neighbors (0-OCC_THRESHOLD)
                if any(0 <= grid2d.getCost(n.mapX, n.mapY) <= OCC_THRESHOLD
                       for n in _neighbors(v, grid2d, cache)):
                    v.classification |= PointClassification.MapOpen.value
                    map_queue.append(v)

        p.classification |= PointClassification.MapClosed.value

    # Farthest first to encourage exploration
    frontiers.sort(key=lambda f: math.dist((f[0], f[1]), robot_pos), reverse=True)
    return frontiers

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        self.create_subscription(Bool,          '/explorer_active', self.active_cb, 10)
        self.create_subscription(OccupancyGrid, '/map',             self.map_cb,    10)
        self.create_subscription(Odometry,      '/odom',            self.odom_cb,   10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.is_active        = True
        self.grid2d           = None
        self.pos              = (0.0, 0.0)
        self.last_pos         = (0.0, 0.0)
        self.current_goal     = None
        self.goal_in_progress = False
        self._goal_handle     = None
        self.stuck_counter    = 0
        self.blacklist        = deque(maxlen=30)

        self.scanning       = False
        self.scan_start_time = None

        self.timer = self.create_timer(3.0, self.explore)
        self.get_logger().info('SimpleExplorer ready with adjusted thresholds')

    def active_cb(self, msg):
        self.is_active = msg.data
        if not self.is_active:
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self.goal_in_progress = False
            self.scanning = False
            self.stop()

    def map_cb(self, msg):
        self.grid2d = OccupancyGrid2d(msg)

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def start_scan(self):
        self.scanning        = True
        self.scan_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('Starting 360° ArUco scan...')

    def tick_scan(self):
        now     = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.scan_start_time
        if elapsed < SCAN_DURATION:
            twist = Twist()
            twist.angular.z = SCAN_SPEED
            self.cmd_vel_pub.publish(twist)
            return False
        else:
            self.stop()
            self.scanning = False
            return True

    def force_rotate(self):
        self.get_logger().warn('Forcing rotation to recover...')
        twist = Twist()
        twist.angular.z = 0.8
        for _ in range(15):
            self.cmd_vel_pub.publish(twist)
            # Use small sleep to allow message to go out
            import time
            time.sleep(0.1)
        self.stop()

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    def explore(self):
        if not self.is_active or self.grid2d is None:
            return

        if self.scanning:
            if not self.tick_scan():
                return

        if self.current_goal is not None and not self.scanning:
            moved = math.dist(self.pos, self.last_pos)
            self.stuck_counter = 0 if moved > 0.05 else self.stuck_counter + 1
        self.last_pos = self.pos

        if self.stuck_counter >= STUCK_TICKS:
            self.get_logger().error('STUCK — blacklisting and recovering')
            if self.current_goal:
                self.blacklist.append(self.current_goal)
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self.goal_in_progress = False
            self.stuck_counter    = 0
            self.force_rotate()
            return

        if self.goal_in_progress:
            return

        try:
            frontiers = get_frontiers(self.pos, self.grid2d)
        except Exception as e:
            self.get_logger().warn(f'WFD error: {e}')
            return

        if not frontiers:
            self.get_logger().info('No frontiers found.')
            return

        best_goal = None
        for (fx, fy, size) in frontiers:
            centre = (fx, fy)
            if math.dist(centre, self.pos) < 0.2:
                continue
            if any(math.dist(centre, b) < BLACKLIST_RADIUS for b in self.blacklist):
                continue
            best_goal = centre
            break

        if best_goal:
            self.send_navigation_goal(best_goal)

    def send_navigation_goal(self, coords):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = coords[0]
        goal_msg.pose.pose.position.y    = coords[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to: ({coords[0]:.2f}, {coords[1]:.2f})')
        self.goal_in_progress = True
        self.current_goal = coords
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_in_progress = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        self.goal_in_progress = False
        self.current_goal     = None
        self.start_scan()

def main():
    rclpy.init()
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()