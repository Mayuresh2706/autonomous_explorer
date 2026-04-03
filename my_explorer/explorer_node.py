import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import math
from collections import deque


#To shift goals away according to launcher offset
LAUNCHER_OFFSET_Y   = 0.3
#To commit to goals for longer
SWITCH_THRESHOLD  = 2.0
#Prevent oscillation aaround same goal

BLACKLIST_RADIUS = 0.6

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map_msg = None
        self.pos = (0.0, 0.0)
        self.last_pos = (0.0,0.0)
        self.timer = self.create_timer(3.0, self.explore)
        self.current_goal = None
        self.current_score = 0.0
        #Prevent oscillation aaround same goal
        self.stuck_counter = 0
        self.blacklist     = []
    
    def map_callback(self,msg):
        self.map_msg = msg
    
    def odom_callback(self,msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def get_world_coords(self, index):
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin.position
        w = self.map_msg.info.width
        return (origin.x + (index % w) * res, origin.y + (index // w) * res)
    
    def explore(self):
        if not self.map_msg: 
            return

        if self.current_goal is not None:
            moved = math.dist(self.pos, self.last_pos)
            if moved < 0.05:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
        self.last_pos = self.pos

        if self.stuck_counter >= 5:
            if self.current_goal:
                self.blacklist.append(self.current_goal)
            self.current_goal  = None
            self.current_score = 0.0
            self.stuck_counter = 0
            self.last_pos = self.pos

            #Reset everything
            stop_msg = PoseStamped()
            stop_msg.header.frame_id = 'map'
            stop_msg.pose.position.x, stop_msg.pose.position.y = self.pos
            stop_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(stop_msg)
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
                for neighbour in frontiers_coords:
                    if neighbour not in visited and math.dist(point, neighbour) < 0.6:
                                                visited.add(neighbour)
                                                queue.append(neighbour)
            clusters.append(cluster)


        # for f in frontiers_coords:
        #     found_cluster = False
        #     for cluster in clusters:
        #         if math.dist(f, cluster[0]) < 0.6:
        #             cluster.append(f)
        #             found_cluster = True
        #             break

        #     if not found_cluster:
        #         clusters.append([f])
            


        scored_goals = []
        for cluster in clusters:
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            centre = (avg_x, avg_y)
            dist = math.dist(centre, self.pos)
            size = len(cluster)

            if size < 5:continue
            if dist < 0.1: continue
            if any(math.dist(centre, b) < BLACKLIST_RADIUS for b in self.blacklist): continue
            utility = (size**1.2) / (dist + 0.5)

            scored_goals.append((utility, centre))
        
        
        if not scored_goals:
            self.get_logger().info("No high-quality goals. Waiting...")
            return 
        
        if self.current_goal is not None and math.dist(self.pos, self.current_goal) < 0.4:
            self.current_goal  = None
            self.current_score = 0.0

        best_score, best_goal = max(scored_goals, key=lambda x: x[0])
        
        if self.current_goal is not None and best_score < self.current_score * SWITCH_THRESHOLD:
            best_goal = self.current_goal
            best_score = self.current_score
        
        self.current_goal = best_goal
        self.current_score = best_score
        
        #Math for launcher not hitting the wall
        dx = self.pos[0] - best_goal[0]
        dy = self.pos[1] - best_goal[1]
        d = math.sqrt(dx**2 + dy**2)


        final_x = best_goal[0] + (dx / (d+0.0000001)) * LAUNCHER_OFFSET_Y
        final_y = best_goal[1] + (dy / (d+0.0000001)) * LAUNCHER_OFFSET_Y

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x =  final_x
        msg.pose.position.y = final_y
        msg.pose.orientation.w = 1.0
        msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Moving to: {best_goal}")

def main():
    rclpy.init()
    rclpy.spin(SimpleExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()