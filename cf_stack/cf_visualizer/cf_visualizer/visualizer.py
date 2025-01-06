import rclpy
from rclpy.node import Node

from cf_messages.msg import SegmentMsg
from cf_messages.msg import SegmentListMsg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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


    def segment_subscriber_callback(self, msg : SegmentListMsg):

        marker_array = MarkerArray()
        for id, segment in enumerate(msg.segments):
            marker_array.markers.append(self.get_segment_marker(segment, id))

        self.segment_vis_publisher.publish(marker_array)

    def opt_path_subscriber_callback(self, msg : SegmentListMsg):
        path = Path()

        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        offset : float = 0
        inc_step : float = 0.3 / len(msg.segments)
        inc : float = 0
        for seg in msg.segments:
            path.poses.append(self.get_path_pose(seg, z_offset=inc))
            inc += inc_step

        self.path_opt_vis_publisher.publish(path)

    def path_subscriber_callback(self, msg : SegmentListMsg):
        path = Path()

        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for seg in msg.segments:
            path.poses.append(self.get_path_pose(seg))

        self.path_vis_publisher.publish(path)

    def get_path_pose(self, segment : SegmentMsg, z_offset : float = 0) -> PoseStamped:
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = segment.x
        pose.pose.position.y = segment.y
        pose.pose.position.z = segment.z + z_offset
        pose.pose.orientation.w = 1.0

        return pose

    def get_segment_marker(self, segment : SegmentMsg, id : int) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'

        marker.ns = 'cube'
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = segment.x
        marker.pose.position.y = segment.y
        marker.pose.position.z = segment.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0 if segment.obstacle else 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0 if segment.start else 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        marker.scale.x = self.x_segment_size * 0.5
        marker.scale.y = self.y_segment_size * 0.5
        marker.scale.z = 0.1
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
