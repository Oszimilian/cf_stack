import rclpy
from rclpy.node import Node

from cf_messages.msg import SegmentMsg
from cf_messages.msg import SegmentListMsg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from typing import List, Dict

class Visualizer(Node):
    def __init__(self):
        super().__init__('Visualizer')
        self.get_logger().info('Started Visualizer')

        #Parameter
        self.declare_parameter(name='x_segment_size', value=0.2)
        self.declare_parameter(name='y_segment_size', value=0.2)

        self.x_segment_size: float = self.get_parameter('x_segment_size').get_parameter_value().double_value
        self.y_segment_size: float = self.get_parameter('y_segment_size').get_parameter_value().double_value

        self.get_logger().info(f"X Segment Size: {self.x_segment_size}")
        self.get_logger().info(f"Y Segment Size: {self.y_segment_size}")



        self.segment_subscriber = self.create_subscription( SegmentListMsg, 
                                                            '/segments', 
                                                            self.segment_subscriber_callback,
                                                            10)
        
        self.segment_vis_publisher = self.create_publisher( MarkerArray,
                                                            'visualization_marker',
                                                            10)
        

        self.path_subscriber = self.create_subscription(    SegmentListMsg,
                                                            '/path',
                                                            self.path_subscriber_callback,
                                                            10)
        
        self.path_vis_publisher = self.create_publisher(    Path,
                                                            'path_vis',
                                                            10)
        
        self.path_opt_subscriber = self.create_subscription(    SegmentListMsg,
                                                            '/path/opt',
                                                            self.opt_path_subscriber_callback,
                                                            10)
        
        self.path_opt_vis_publisher = self.create_publisher(    Path,
                                                            'path_vis/opt',
                                                            10)
        
        self.ppc_pose_subscriber = self.create_subscription(    Point,
                                                                '/ppc',
                                                                self.ppc_pose_subscriber_callback,
                                                                10)

        self.ppc_pose_vis_publisher = self.create_publisher(    Marker,
                                                                '/ppc/vis',
                                                                10)
        
        self.drone_pos_vis_publisher = self.create_publisher(   Marker,
                                                                '/drone/vis',
                                                                10)
        
        self.drone_pos_subscriber = self.create_subscription(   Point,
                                                                '/drone/pos',
                                                                self.drone_pos_callback,
                                                                10)

    def drone_pos_callback(self, msg : Point):
        marker = self.get_segment_marker(segment=self.segment_out_of_point(msg), id=50, size_scale=0.2, rgb=[1.0, 0.5, 1.0])
        self.drone_pos_vis_publisher.publish(marker)

    def segment_out_of_point(self, point : Point) -> SegmentMsg:
        segment = SegmentMsg()
        segment.x = point.x
        segment.y = point.y
        segment.z = point.z
        return segment

    def ppc_pose_subscriber_callback(self, msg : Point):
        marker = self.get_segment_marker(segment=self.segment_out_of_point(msg), id=50, size_scale=0.2, rgb=[1.0, 0.0, 0.0])
        self.ppc_pose_vis_publisher.publish(marker)

    def segment_subscriber_callback(self, msg : SegmentListMsg):

        marker_array = MarkerArray()
        for id, segment in enumerate(msg.segments):
            marker_array.markers.append(self.get_segment_marker(segment, id, size_scale=0.9))

        self.segment_vis_publisher.publish(marker_array)

    def opt_path_subscriber_callback(self, msg : SegmentListMsg):
        path = Path()

        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()

        offset : float = 0
        inc_step : float = 0.3 / len(msg.segments)
        inc : float = 0
        for seg in msg.segments:
            path.poses.append(self.get_path_pose(seg, z_offset=1.0))
            inc += inc_step

        self.path_opt_vis_publisher.publish(path)

    def path_subscriber_callback(self, msg : SegmentListMsg):
        path = Path()

        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()

        for seg in msg.segments:
            path.poses.append(self.get_path_pose(seg))

        self.path_vis_publisher.publish(path)

    def get_path_pose(self, segment : SegmentMsg, z_offset : float = 1.0) -> PoseStamped:
        pose = PoseStamped()

        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = segment.x - (self.x_segment_size / 2)
        pose.pose.position.y = segment.y - (self.y_segment_size / 2)
        pose.pose.position.z = segment.z + z_offset
        pose.pose.orientation.w = 1.0

        return pose

    def get_segment_marker(self, segment : SegmentMsg, id : int, size_scale : float = 1.0, rgb : List[float] = []) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'world'

        marker.ns = 'cube'
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = segment.x - (self.x_segment_size / 2)
        marker.pose.position.y = segment.y - (self.y_segment_size / 2)
        marker.pose.position.z = segment.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        if len(rgb) == 0:
            marker.color.r = 1.0 if segment.obstacle else 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0 if segment.start else 0.0
            marker.color.a = 1.0
        else:
            marker.color.r = rgb[0]
            marker.color.g = rgb[1]
            marker.color.b = rgb[2]
            marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        marker.scale.x = self.x_segment_size * size_scale
        marker.scale.y = self.y_segment_size * size_scale
        marker.scale.z = 0.05 * size_scale
        marker.header.stamp = self.get_clock().now().to_msg()

        return marker

def main():
    rclpy.init()
    pathPlanning = Visualizer()

    try:
        rclpy.spin(pathPlanning)
        pathPlanning.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
