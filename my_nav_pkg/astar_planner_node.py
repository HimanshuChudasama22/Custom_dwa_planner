import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import numpy as np
import cv2
import yaml
from cv_bridge import CvBridge
import pathlib
from heapq import heappush, heappop
import math
from ament_index_python.packages import get_package_share_directory
import os




class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.get_logger().info("Initializing AStarPlanner node...")

        pkg_share = get_package_share_directory('my_nav_pkg')
        self.yaml_path = os.path.join(pkg_share, 'maps', 'map.yaml')

        self.load_map()

        self.bridge = CvBridge()
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.debug_costmap_pub = self.create_publisher(Image, '/astar_costmap', 1)
        self.debug_path_img_pub = self.create_publisher(Image, '/astar_path_image', 1)
        self.odom_debug_pub = self.create_publisher(PoseStamped, '/current_odom_pose', 1)

        self.current_pose = None
        self.goal_pose = None

        self.timer_1hz = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Set up 1Hz timer for publishing debug images and odom pose.")

    def load_map(self):
        self.get_logger().info(f"Loading map from: {self.yaml_path}")
        with open(self.yaml_path, 'r') as f:
            y = yaml.safe_load(f)

        pgm_path = str((pathlib.Path(self.yaml_path).parent / y['image']).resolve())
        self.get_logger().info(f"Loaded image: {pgm_path}")

        self.resolution = float(y['resolution'])
        self.origin = y['origin'][:2]  # ignore yaw
        self.occ_thresh = float(y['occupied_thresh'])
        self.free_thresh = float(y['free_thresh'])
        negate = int(y.get('negate', 0))

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE).astype(np.float32) / 255.0
        if negate == 1:
            img = 1.0 - img
        self.img = img

        self.costmap = self.generate_costmap(img)
        self.get_logger().info("Costmap generated.")

    def generate_costmap(self, img):
        GAUSS_SIGMA = 5
        BASE_FREE_COST = 1.0
        OBSTACLE_COST = 500
        INFLATION_WEIGHT = 500
        DARKNESS_WEIGHT = 20.0

        occupied = img <= self.occ_thresh
        free = img >= self.free_thresh
        unknown = ~(occupied | free)

        raw_cost = np.full_like(img, OBSTACLE_COST, dtype=np.float32)
        raw_cost[free] = BASE_FREE_COST
        raw_cost[unknown] = OBSTACLE_COST

        obstacle_field = occupied.astype(np.float32)
        inflated = cv2.GaussianBlur(obstacle_field, (0, 0), GAUSS_SIGMA)

        costmap = raw_cost.copy()
        mask = raw_cost < OBSTACLE_COST
        costmap[mask] += INFLATION_WEIGHT * inflated[mask]
        costmap[mask] += DARKNESS_WEIGHT * (1.0 - img[mask])

        self.obstacle_field = obstacle_field
        self.inflated = inflated

        return costmap

    def timer_callback(self):
        if self.current_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose = self.current_pose
            self.odom_debug_pub.publish(msg)

        self.publish_costmap_debug()

    def publish_costmap_debug(self):
        norm = np.log1p(self.costmap)
        norm = (255 * norm / norm.max()).astype(np.uint8)
        rgb = cv2.applyColorMap(norm, cv2.COLORMAP_VIRIDIS)
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_costmap_pub.publish(msg)
        self.get_logger().info("Costmap debug image published.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"Received odometry. Current pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
        self.try_plan()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        x = self.goal_pose.position.x
        y = self.goal_pose.position.y
        self.get_logger().info(f"✅ Received goal pose: ({x:.2f}, {y:.2f})")
        self.try_plan()

    def pose_to_pixel(self, pose):
        x, y = pose.position.x, pose.position.y
        origin_x = (2.0 - self.origin[0]) 
        origin_y = (0.5 - self.origin[1])
        map_x = (origin_x + x)/self.resolution
        map_y = (origin_y + y)/self.resolution
        r = int(self.img.shape[0] - map_y)
        c = int(map_x)
        return (r, c)

    def pixel_to_pose(self, rc):
        r, c = rc
        origin_x = (2.0 - self.origin[0])
        origin_y = (0.5 - self.origin[1])

        x = (c * self.resolution) - origin_x
        y = ((self.img.shape[0] - r) * self.resolution) - origin_y

        return x, y


    def try_plan(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        self.get_logger().info("Planning path with A*...")
        start_rc = self.pose_to_pixel(self.current_pose)
        goal_rc = self.pose_to_pixel(self.goal_pose)
        self.get_logger().info(f"Start pixel: {start_rc}, Goal pixel: {goal_rc}")

        path_pixels = self.astar(self.costmap, start_rc, goal_rc)
        if not path_pixels:
            self.get_logger().warn('A* failed to find a path')
            return

        self.get_logger().info(f"A* path found with {len(path_pixels)} points.")

        rgb = np.dstack([self.img]*3)
        for r, c in path_pixels:
            if 0 <= r < rgb.shape[0] and 0 <= c < rgb.shape[1]:
                rgb[r, c] = [0.0, 0.0, 1.0]
        r0, c0 = start_rc
        r1, c1 = goal_rc
        rgb[r0, c0] = [1.0, 0.0, 0.0]
        rgb[r1, c1] = [0.0, 1.0, 0.0]

        img_msg = self.bridge.cv2_to_imgmsg((rgb*255).astype(np.uint8), encoding='rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_path_img_pub.publish(img_msg)
        self.get_logger().info("Published debug path image.")

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for r, c in path_pixels:
            x, y = self.pixel_to_pose((r, c))
            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published nav_msgs/Path message.")

    def astar(self, costmap, start, goal, max_iter=200000):
        OBSTACLE_COST = 1e6
        rows, cols = costmap.shape
        open_set = []
        heappush(open_set, (self.heuristic(start, goal), 0.0, start, None))
        visited = {}
        came_from = {}

        def is_valid(r, c):
            return 0 <= r < rows and 0 <= c < cols and costmap[r, c] < OBSTACLE_COST

        while open_set and max_iter > 0:
            max_iter -= 1
            f, g, (r, c), parent = heappop(open_set)
            if (r, c) in visited:
                continue
            visited[(r, c)] = g
            came_from[(r, c)] = parent
            if (r, c) == goal:
                path = []
                cur = (r, c)
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                return path[::-1]

            for dr, dc, mc in [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]:
                nr, nc = r + dr, c + dc
                if not is_valid(nr, nc):
                    continue
                if dr != 0 and dc != 0:
                    if not is_valid(r + dr, c) or not is_valid(r, c + dc):
                        continue
                new_g = g + costmap[nr, nc] * mc
                if (nr, nc) not in visited or new_g < visited[(nr, nc)]:
                    new_f = new_g + self.heuristic((nr, nc), goal)
                    heappush(open_set, (new_f, new_g, (nr, nc), (r, c)))
        return None

    def heuristic(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()