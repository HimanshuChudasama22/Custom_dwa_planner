import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class DebugViz(Node):
    def __init__(self):
        super().__init__('debug_viz')
        self.sub = self.create_subscription(Path, 'planned_path', self.path_cb, 10)
        self.pub = self.create_publisher(Marker, 'planned_path_marker', 10)

    def path_cb(self, msg: Path):
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'planned_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for p in msg.poses:
            pt = Point()
            pt.x = p.pose.position.x
            pt.y = p.pose.position.y
            pt.z = 0.01
            marker.points.append(pt)
        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DebugViz()
    rclpy.spin(node)
    rclpy.shutdown()
