import rclpy
from rclpy.node import Node
from enum import Enum
from crazyflie_interfaces.msg import LogBlock
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from typing import List
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class State(Enum):
    CREAT_LOG_BLOGS = 1
    WAIT1 = 2
    WAIT2 = 3
    START_LOG_BLOGS = 4
    MEASURE = 5

class Sampling(Node):
    def __init__(self):
        super().__init__("sampling")
        self.get_logger().info("Started Sampling")

        self.timer = self.create_timer( 0.5,
                                        self.timer_callback)
        
        self.id = 'cf2'
        
        self.create_z_logblock_publisher = self.create_publisher(   LogBlock,
                                                                    f'/{self.id}/create_log_block',
                                                                    10)
        
        self.start_z_logblock_publisher = self.create_publisher(    Int16,
                                                                    f'/{self.id}/log/z_range/start',
                                                                    10)

        self.state : State = State.CREAT_LOG_BLOGS

        self.z_range_subscriber = self.create_subscription(    LogBlock,
                                                               f'/{self.id}/log/z_range/data',
                                                               self.z_range_callback,
                                                               10)
        self.z_publisher = self.create_publisher(   Point,
                                                    '/grid/z',
                                                    10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.z_publisher = self.create_publisher(Point, '/grid/z', 10)

    def z_range_callback(self, msg):
        z_pos = Point()
        pos = self.get_drone_position()
        if len(pos) > 0:
            z_pos.x = pos[0]
            z_pos.y = pos[1]
            z_pos.z = msg.values[0]
            self.z_publisher.publish(z_pos)

    def get_drone_position(self) -> List[float]:
        try:
            t = self.tf_buffer.lookup_transform('world', 
                                                self.tf_name,
                                                rclpy.time.Time())
            return [t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z]
        except Exception as ex:
            self.state = State.ERROR
            return []

    def timer_callback(self):
        if self.state == State.CREAT_LOG_BLOGS:
            log_block_msg = LogBlock()
            log_block_msg.variables = ['range.zrange']
            log_block_msg.name = 'z_range'
            self.create_z_logblock_publisher.publish(log_block_msg)
            self.state = State.WAIT1
        elif self.state == State.WAIT1:
            self.state = State.WAIT2
        elif self.state == State.WAIT2:
            self.state = State.START_LOG_BLOGS
        elif self.state == State.START_LOG_BLOGS:
            sample_rate = Int16()
            sample_rate.data = 10
            self.start_z_logblock_publisher.publish(sample_rate)
            self.state = State.MEASURE
        

def main():
    rclpy.init()
    sampling = Sampling()

    try:
        rclpy.spin(sampling)
        sampling.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()