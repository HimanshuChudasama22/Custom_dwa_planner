import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math

class LocalNavigator(Node):
    def __init__(self):
        super().__init__('local_navigator')
        # self.declare_parameter('look_ahead_dist', 0.05)
        # self.declare_parameter('linear_speed', 0.5)
        # self.declare_parameter('kp_angular', 1.2)
        # self.declare_parameter('max_angular_speed', 0.4)
        # self.declare_parameter('obstacle_stop_dist', 0.3)
        # self.declare_parameter('yaw_tolerance', 0.05)
        # self.declare_parameter('pos_tolerance', 0.02)


        self.lin_speed = 0.1
        self.look_ahead = 0.3
        self.kp_ang = 1.2
        self.max_ang_speed = 0.4
        self.obs_stop = 0.3
        self.yaw_tol = 0.05
        self.pos_tol = 0.05


        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_scan = None
        self.current_odom = None
        self.path = []
        self.goal_yaw = None
        self.triggered = False

        self.state = 'IDLE'
        self.speed_factor = 1

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('LocalNavigator with FSM and Pure Pursuit started.')

    def goal_callback(self, msg):
        self.triggered = True  # just a flag to rotate towards path once

    def path_callback(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if new_path == self.path:
            return

        self.path = new_path

        # Determine goal yaw
        if len(self.path) >= 2:
            if msg.poses:
                q = msg.poses[-1].pose.orientation
                yaw_from_quat = self.quat_to_yaw(q)
                if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 0.01:  # ignore zero-orientation
                    self.goal_yaw = yaw_from_quat
                else:
                    # Fallback: compute from last two path points
                    x1, y1 = self.path[-2]
                    x2, y2 = self.path[-1]
                    self.goal_yaw = math.atan2(y2 - y1, x2 - x1)
            else:
                x1, y1 = self.path[-2]
                x2, y2 = self.path[-1]
                self.goal_yaw = math.atan2(y2 - y1, x2 - x1)
        else:
            self.goal_yaw = None

        if len(self.path) >= 2:
            if self.triggered:
                self.state = 'ROTATE_TO_PATH'
                self.get_logger().info(f'New goal received. Switching to {self.state}')
                self.triggered = False
        else:
            self.state = 'IDLE'
    def scan_callback(self, msg: LaserScan):
        self.current_scan = msg

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def control_loop(self):
        if self.current_odom is None:
            return

        if self.state == 'IDLE':
            self.publish_stop()
            return

        if self.state == 'ROTATE_TO_PATH':
            done = self.rotate_towards_first_segment()
            if done:
                self.state = 'FOLLOW_PATH'
            return

        if self.state == 'FOLLOW_PATH':
            done = self.follow_path_step()
            if done:
                if self.goal_yaw is not None:
                    self.state = 'ROTATE_TO_GOAL'
                else:
                    self.state = 'IDLE'
            return

        if self.state == 'ROTATE_TO_GOAL':
            done = self.rotate_to_yaw(self.goal_yaw)
            if done:
                self.state = 'IDLE'
            return

    def rotate_towards_first_segment(self):
        if len(self.path) < 2:
            return True

        rx, ry, yaw = self.get_robot_pose()
        target = self.path[1]
        desired = math.atan2(target[1] - ry, target[0] - rx)
        err = self.normalize_angle(desired - yaw)

        self.get_logger().info(f"[ROTATE_TO_PATH] yaw={yaw:.2f}, desired={desired:.2f}, err={err:.2f}")

        if abs(err) < self.yaw_tol:
            self.publish_stop()
            return True

        twist = Twist()
        twist.angular.z = max(-0.3, min(0.3, self.kp_ang * err))
        self.cmd_pub.publish(twist)
        return False

    def follow_path_step(self):
        speed_factor = 1
        if self.is_obstacle_ahead():
            # self.get_logger().info("\U0001F6AB Obstacle detected ahead. Stopping.")
            # self.publish_stop()
            # return False
            speed_factor = self.speed_factor

        rx, ry, yaw = self.get_robot_pose()
        target = None
        min_d = 999.0
        for (px, py) in self.path:
            d = math.hypot(px - rx, py - ry)
            if d > self.look_ahead and d < min_d:
                min_d = d
                target = (px, py)

        if target is None:
            gx, gy = self.path[-1]
            goal_dist = math.hypot(gx - rx, gy - ry)
            self.get_logger().info(f"✅ At final point. Distance to goal: {goal_dist:.2f}")
            if goal_dist < self.pos_tol:
                self.publish_stop()
                return True
            else:
                self.get_logger().info("⚠️ No target ahead. Approaching final goal directly.")
                target = (gx, gy)

        angle_to_target = math.atan2(target[1] - ry, target[0] - rx)
        heading_error = self.normalize_angle(angle_to_target - yaw)

        self.get_logger().info(f"[FOLLOW_PATH] target=({target[0]:.2f}, {target[1]:.2f}), heading_error={heading_error:.2f}")

        twist = Twist()
        twist.linear.x = self.lin_speed*speed_factor
        self.speed_factor = 1

        if abs(heading_error) < math.radians(45):
            twist.angular.z = max(-self.max_ang_speed, min(self.max_ang_speed, self.kp_ang * heading_error))
        else:
            twist.angular.z = self.max_ang_speed * (1 if heading_error > 0 else -1)

        self.cmd_pub.publish(twist)
        return False

    def rotate_to_yaw(self, desired_yaw):
        rx, ry, yaw = self.get_robot_pose()
        err = self.normalize_angle(desired_yaw - yaw)
        if abs(err) < self.yaw_tol:
            self.publish_stop()
            return True
        twist = Twist()
        twist.angular.z = max(-0.3, min(0.3, self.kp_ang * err))
        self.cmd_pub.publish(twist)
        return False

    def get_robot_pose(self):
        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        yaw = self.quat_to_yaw(q)
        return p.x, p.y, yaw

    def is_obstacle_ahead(self):
        if self.current_scan is None:
            self.speed_factor = 1.0
            return False

        scan = self.current_scan
        angle_min = scan.angle_min
        inc = scan.angle_increment
        center = int((0.0 - angle_min) / inc)
        window = int(math.radians(20) / inc)
        start = max(0, center - window)
        end = min(len(scan.ranges) - 1, center + window)

        min_range = float('inf')
        for i in range(start, end):
            r = scan.ranges[i]
            if scan.range_min < r < min_range:
                min_range = r

        # Define zones
        full_speed_dist = self.obs_stop + 0.6
        slow_down_dist = self.obs_stop + 0.2

        if min_range < self.obs_stop:
            self.speed_factor = 0.0
            return True
        elif min_range < slow_down_dist:
            # Linearly interpolate speed between 0 and 1 based on distance
            self.speed_factor = max(0.1, (min_range - self.obs_stop) / (slow_down_dist - self.obs_stop))
        elif min_range < full_speed_dist:
            self.speed_factor = 0.9
        else:
            self.speed_factor = 1.0

        return False
    

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    