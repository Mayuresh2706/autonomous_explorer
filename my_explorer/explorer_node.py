import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import math
from collections import deque
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient                  
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


#Prevent oscillation aaround same goal
BLACKLIST_RADIUS = 0.5

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        self.active_sub = self.create_subscription(Bool, '/explorer_active', self.active_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.is_active = True
        self.map_msg = None
        self.pos = (0.0, 0.0)
        self.last_pos = (0.0,0.0)
        self.timer = self.create_timer(3.0, self.explore)
        self.current_goal = None
        self.current_score = 0.0

        #Prevent oscillation aaround same goal
        self.stuck_counter = 0
        self.blacklist     = []
        self.goal_in_progress = False

    def active_callback(self, msg):
        self.is_active = msg.data
        if not self.is_active and hasattr(self, '_goal_handle') and self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self.goal_in_progress = False

    def map_callback(self,msg):
        self.map_msg = msg
        self.res = self.map_msg.info.resolution
        self.origin_x = self.map_msg.info.origin.position.x
        self.origin_y = self.map_msg.info.origin.position.y
        self.w = self.map_msg.info.width
    
    def odom_callback(self,msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def get_world_coords(self, index):
        return (self.origin_x + (index % self.w) * self.res, 
        self.origin_y + (index // self.w) * self.res)
    
    def cluster_frontiers(self, frontiers_coords):
        CLUSTER_RADIUS = 0.6
        bucket_size = CLUSTER_RADIUS

        buckets = {}
        for point in frontiers_coords:
            key = (int(point[0] / bucket_size), int(point[1] / bucket_size))
            buckets.setdefault(key, []).append(point)

        visited = set()
        clusters = []

        for start in frontiers_coords:
            if start in visited:
                continue

            cluster = []
            queue = deque([start])
            visited.add(start)

            while queue:
                point = queue.popleft()
                cluster.append(point)

                bx = int(point[0] / bucket_size)
                by = int(point[1] / bucket_size)

                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for neighbour in buckets.get((bx + dx, by + dy), []):
                            if neighbour not in visited and math.dist(point, neighbour) < CLUSTER_RADIUS:
                                visited.add(neighbour)
                                queue.append(neighbour)

            clusters.append(cluster)

        return clusters


    def explore(self):
        if not self.is_active: return
        if not self.map_msg or self.goal_in_progress: 
            return

        if self.current_goal is not None:
            moved = math.dist(self.pos, self.last_pos)
            if moved < 0.1:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
        self.last_pos = self.pos

        if self.stuck_counter >= 10:
            self.get_logger().error("STUCK")
            if self.current_goal:
                self.blacklist.append(self.current_goal)
                if len(self.blacklist) > 20:
                     self.blacklist.pop(0)

            self.current_goal  = None
            self.current_score = 0.0
            self.stuck_counter = 0
            self.goal_in_progress = False
            self.last_pos = self.pos

            #Reset everything
            stop_msg = PoseStamped()
            stop_msg.header.frame_id = 'map'
            stop_msg.pose.position.x, stop_msg.pose.position.y = self.pos
            stop_msg.pose.orientation.w = 1.0
            return

        
        grid = self.map_msg.data
        w, h = self.map_msg.info.width, self.map_msg.info.height
        frontiers_coords = []


        for y in range(1, h - 1):
            for x in range(1, w - 1):
                i = y * w + x

                if not (0 <= grid[i] <= 45):
                    continue

                neighbors = [
                    grid[y * w + (x - 1)],  # left
                    grid[y * w + (x + 1)],  # right
                    grid[(y - 1) * w + x],  # up
                    grid[(y + 1) * w + x],  # down
                    ]

                if -1 in neighbors:
                    frontiers_coords.append(self.get_world_coords(i))

        if not frontiers_coords:
            self.get_logger().info("Exploration Complete!")
            return

        clusters = self.cluster_frontiers(frontiers_coords)

        scored_goals = []
        for cluster in clusters:
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            centre = (avg_x, avg_y)
            dist = math.dist(centre, self.pos)
            size = len(cluster)

            if size <2:continue
            if dist < 0.1:continue
            if any(math.dist(centre, b) < BLACKLIST_RADIUS for b in self.blacklist): continue
            utility = (size) / (dist + 0.1)

            scored_goals.append((utility, centre))
        
        
        if not scored_goals:
            self.get_logger().info("No high-quality goals. Waiting...")
            return 
        
        if self.current_goal is not None and math.dist(self.pos, self.current_goal) < 0.4:
            self.current_goal  = None
            self.current_score = 0.0

        best_score, best_goal = max(scored_goals, key=lambda x: x[0])
        
        # if self.current_goal is not None and best_score < self.current_score * SWITCH_THRESHOLD:
        #     best_goal = self.current_goal
        #     best_score = self.current_score
        
        self.current_goal = best_goal
        self.current_score = best_score
        self.send_navigation_goal(best_goal)
        

        # msg = PoseStamped()
        # msg.header.frame_id = 'map'
        # msg.pose.position.x =  best_goal[0]
        # msg.pose.position.y = best_goal[1]
        # msg.pose.orientation.w = 1.0
        # msg.header.stamp = self.get_clock().now().to_msg()
        # self.goal_pub.publish(msg)
        # self.get_logger().info(f"Moving to: {best_goal}")
    def send_navigation_goal(self, coords):                   
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords[0]
        goal_msg.pose.pose.position.y = coords[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: {coords}")
        self.goal_in_progress = True                         
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):                 
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.goal_in_progress = False
            return
        self._goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):                    
        self.current_goal = None
        self.goal_in_progress = False
        self.get_logger().info("Goal reached, finding next frontier.")

def main():
    rclpy.init()
    rclpy.spin(SimpleExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()