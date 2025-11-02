import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # === Configurable Parameters ===
        self.max_speed = 0.1
        self.min_speed = 0.0
        self.max_yaw_rate = 2.0
        self.max_accel = 0.2
        self.max_yaw_accel = 2.5
        self.dt = 0.1  # prediction timestep
        self.predict_time = 5.0  # seconds
        self.v_reso = 0.01
        self.w_reso = 0.1

        self.goal = None  # will be updated dynamically
        self.goal_tol = 0.2
        self.robot_radius = 0.3

        # === ROS Interfaces ===
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === State Variables ===
        self.odom = None
        self.scan = None

        # === Timer ===
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("🧭 DWA Local Planner Node started.")

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"🎯 New goal received: {self.goal}")

    def timer_callback(self):
        if self.odom is None or self.scan is None:
            return
        if self.goal is None:
            if not hasattr(self, 'last_goal_warn') or self.get_clock().now().nanoseconds - self.last_goal_warn > 5e9:
                self.get_logger().warn("⚠️ Waiting for /goal_pose...")
                self.last_goal_warn = self.get_clock().now().nanoseconds
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)
        v = self.odom.twist.twist.linear.x
        w = self.odom.twist.twist.angular.z

        state = np.array([x, y, yaw, v, w])
        dw = self.calc_dynamic_window(state)
        u, traj = self.dwa_control(state, dw)

        if self.distance_to_goal(x, y) < self.goal_tol:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🚩 Goal reached.")
            return

        twist = Twist()
        twist.linear.x = u[0]
        twist.angular.z = u[1]
        self.cmd_pub.publish(twist)

    def calc_dynamic_window(self, state):
        vs = [self.min_speed, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        v = state[3]
        w = state[4]
        vd = [v - self.max_accel * self.dt, v + self.max_accel * self.dt,
              w - self.max_yaw_accel * self.dt, w + self.max_yaw_accel * self.dt]
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        dw[0] = max(0.01, dw[0])
        return dw

    def dwa_control(self, state, dw):
        min_cost = float('inf')
        best_u = [0.0, 0.0]
        best_traj = []

        for v in np.arange(dw[0], dw[1], self.v_reso):
            for w in np.arange(dw[2], dw[3], self.w_reso):
                traj = self.predict_trajectory(state, v, w)
                to_goal = self.calc_to_goal_cost(traj)
                speed_cost = (self.max_speed - traj[-1, 3])
                if traj[-1, 3] < 0.05:
                    speed_cost += 5.0
                obs_cost = self.calc_obstacle_cost(traj)
                if obs_cost == float('inf'):
                    continue
                final_cost = 1* to_goal + 1.5 * speed_cost + 150 * obs_cost
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj

        return best_u, best_traj

    def predict_trajectory(self, state, v, w):
        traj = []
        x = np.array(state)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, w])
            traj.append(x.copy())
            time += self.dt
        return np.array(traj)

    def motion(self, x, u):
        x[2] += u[1] * self.dt
        x[0] += u[0] * math.cos(x[2]) * self.dt
        x[1] += u[0] * math.sin(x[2]) * self.dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_to_goal_cost(self, traj):
        dx = self.goal[0] - traj[-1, 0]
        dy = self.goal[1] - traj[-1, 1]
        angle_to_goal = math.atan2(dy, dx)
        traj_yaw = traj[-1, 2]
        heading_error = abs(self.normalize_angle(angle_to_goal - traj_yaw))
        return heading_error

    def calc_obstacle_cost(self, traj):
        if self.scan is None or self.goal is None:
            return 0.0

        ranges = np.array(self.scan.ranges, dtype=np.float32)
        n = len(ranges)
        angles = self.scan.angle_min + np.arange(n) * self.scan.angle_increment

        # Filter out invalid scan points
        valid = np.isfinite(ranges) & (ranges > 0.05)  # ignore zero/inf/nan
        if not np.any(valid):
            return 0.0

        ranges = ranges[valid]
        angles = angles[valid]

        # === Parameters for shaping the cost field ===
        distance_decay = 0.85    # smaller = more aggressive near robot
        angle_spread = 0.5      # radians, defines front "danger cone"

        # Use the final pose of the trajectory
        traj_heading = traj[-1, 2]  # yaw

        cost = 0.0
        for r, a in zip(ranges, angles):
            angle_error = abs(self.normalize_angle(a - traj_heading))
            if angle_error > math.pi / 3:
                continue  # ignore obstacles behind or too sideways

            dist_cost = math.exp(-r / distance_decay)
            angle_cost = math.exp(-angle_error  / (2 * angle_spread ** 2))

            cost += dist_cost * angle_cost

        return cost / len(ranges)  # normalize to keep cost magnitude balanced



    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def distance_to_goal(self, x, y):
        return math.hypot(self.goal[0] - x, self.goal[1] - y)

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
