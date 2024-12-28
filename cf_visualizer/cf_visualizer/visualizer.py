import rclpy
from rclpy.node import Node

from cf_messages.msg import SegmentMsg
from cf_messages.msg import SegmentListMsg
from visualization_msgs.msg import Marker

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
        
        self.segment_vis_publisher = self.create_publisher( Marker,
                                                            'visualization_marker',
                                                            1000)
        
        self.last_segments : List[float] = []


    def segment_subscriber_callback(self, msg : SegmentListMsg):
        first_step : bool = True if len(self.last_segments) == 0 else False
        for id, segment in enumerate(msg.segments):
            if first_step:
                self.last_segments.append(segment.z)
                segment_marker = self.get_segment_marker(segment, id)
                self.segment_vis_publisher.publish(segment_marker)
            
            if self.last_segments[id] != segment.z:
                segment_marker = self.get_segment_marker(segment, id)
                self.segment_vis_publisher.publish(segment_marker)
                self.last_segments[id] = segment.z


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
        marker.color.b = 0.0
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
