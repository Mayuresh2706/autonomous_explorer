import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # IDs 1 and 2 are valid docking targets; everything else is ignored
        self.target_ids = {1, 2}
        self.state = 'SEARCHING'
        self.detected_marker_id = None

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.exp_active_pub  = self.create_publisher(Bool, '/explorer_active', 10)
        self.task_a_pub      = self.create_publisher(Bool, '/task_a_active', 10)
        self.task_b_pub      = self.create_publisher(Bool, '/task_b_active', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        self.create_subscription(String, 'task_status', self.task_status_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.target_x_map = None
        self.target_y_map = None
        self._goal_handle = None
        self.approach_in_progress = False

        self.initial_pose_published = False
        self.create_timer(1.0, self.publish_initial_pose_once)

        self.get_logger().info('Mission Manager Initialized - SEARCHING')

    def publish_initial_pose_once(self):
        if self.initial_pose_published:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initial_pose_pub.publish(msg)
        self.initial_pose_published = True
        self.get_logger().info('Initial pose published')

    def _publish_task_active(self, value: bool):
        msg = Bool(data=value)
        if self.detected_marker_id == 1:
            self.task_a_pub.publish(msg)
        elif self.detected_marker_id == 2:
            self.task_b_pub.publish(msg)

    def aruco_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)

        if marker_id not in self.target_ids:
            return

        if self.state != 'SEARCHING':
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            p_map = do_transform_pose(msg.pose, t)
            new_x = p_map.position.x
            new_y = p_map.position.y

            if self.target_x_map is None:
                self.get_logger().info(
                    f'First detection! Marker ID {marker_id} at ({new_x:.2f}, {new_y:.2f}). Moving in...'
                )
                self.detected_marker_id = marker_id
                self.target_x_map = new_x
                self.target_y_map = new_y
                self.exp_active_pub.publish(Bool(data=False))
                self.state = 'APPROACHING'
                self.start_approach(new_x, new_y)

            else:
                dist_change = math.sqrt(
                    (new_x - self.target_x_map) ** 2 +
                    (new_y - self.target_y_map) ** 2
                )
                if dist_change > 0.15:
                    self.get_logger().info(f'Target updated ({dist_change:.2f}m shift) → re-routing')
                    self.target_x_map = new_x
                    self.target_y_map = new_y

                    if self._goal_handle is not None:
                        self._goal_handle.cancel_goal_async()
                        self._goal_handle = None
                    self.approach_in_progress = False
                    self.start_approach(new_x, new_y)

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

    def start_approach(self, target_x, target_y):
        if self.approach_in_progress:
            return

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y

            dx = robot_x - target_x
            dy = robot_y - target_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            self.get_logger().info(f'Robot: ({robot_x:.2f}, {robot_y:.2f})')
            self.get_logger().info(f'Target: ({target_x:.2f}, {target_y:.2f}), dist: {dist:.2f}m')

            if dist < 0.1:
                self.get_logger().warn('Already close enough, skipping approach nav — starting task directly')
                self.state = 'DOCKING'
                self._publish_task_active(True)
                return

            goal_x = target_x + (0.6 * dx / dist)
            goal_y = target_y + (0.6 * dy / dist)
            angle = math.atan2(target_y - goal_y, target_x - goal_x)

            self.get_logger().info(
                f'Approach goal: ({goal_x:.2f}, {goal_y:.2f}), heading: {math.degrees(angle):.1f}°'
            )

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

            self.approach_in_progress = True
            self.nav_client.wait_for_server()
            self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)

        except Exception as e:
            self.get_logger().error(f'Approach setup failed: {e}')
            self.reset_to_explore()

    def nav_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected — resetting to explore')
            self.approach_in_progress = False
            self.reset_to_explore()
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

    def nav_finished_cb(self, future):
        self.approach_in_progress = False
        self._goal_handle = None
        result = future.result()
        status = result.status

        if status != 4:  # 4 = STATUS_SUCCEEDED
            self.get_logger().warn(f'Approach failed (status {status}), resetting...')
            self.reset_to_explore()
            return

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            dist_to_target = math.sqrt(
                (robot_x - self.target_x_map) ** 2 +
                (robot_y - self.target_y_map) ** 2
            )
            self.get_logger().info(f'Distance to target after approach: {dist_to_target:.2f}m')

            if dist_to_target > 1.0:
                self.get_logger().warn(f'Nav2 succeeded but robot is {dist_to_target:.2f}m away — retrying')
                self.start_approach(self.target_x_map, self.target_y_map)
                return

        except Exception as e:
            self.get_logger().warn(f'Could not verify position after approach: {e}')

        task_label = 'A' if self.detected_marker_id == 1 else 'B'
        self.get_logger().info(f'Approach complete. Starting Task {task_label} docking...')
        self.state = 'DOCKING'
        self._publish_task_active(True)

    def task_status_cb(self, msg):
        if msg.data == 'SUCCESS' and self.state == 'DOCKING':
            task_label = 'A' if self.detected_marker_id == 1 else 'B'
            self.get_logger().info(f'Task {task_label} complete. Resuming exploration...')
            self._publish_task_active(False)
            self.reset_to_explore()

    def reset_to_explore(self):
        self.state = 'SEARCHING'
        self.target_x_map = None
        self.target_y_map = None
        self.detected_marker_id = None
        self._goal_handle = None
        self.approach_in_progress = False
        self.exp_active_pub.publish(Bool(data=True))
        self.get_logger().info('Back to SEARCHING')


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
