import rclpy
from rclpy.node import Node
from enum import Enum
from crazyflie_interfaces.msg import LogBlock
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from typing import List
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from crazyflie_interfaces.msg import GenericLogData

class State(Enum):
    CREAT_LOG_BLOGS = 1
    WAIT1 = 2
    WAIT2 = 3
    START_LOG_BLOGS = 4
    MEASURE = 5
    ERROR = 6

class Sampling(Node):
    def __init__(self):
        super().__init__("sampling")
        self.get_logger().info("Started Sampling")

        self.timer = self.create_timer( 0.5,
                                        self.timer_callback)
        
        self.declare_parameter(name='id', value=0)
        self.id_param : int = self.get_parameter('id').get_parameter_value().integer_value
        
        
        self.id = f'cf{self.id_param}'
        self.get_logger().info(f"ID: {self.id}")

        
        self.create_z_logblock_publisher = self.create_publisher(   LogBlock,
                                                                    f'/{self.id}/create_log_block',
                                                                    10)
        
        self.start_z_logblock_publisher = self.create_publisher(    Int16,
                                                                    f'/{self.id}/log/z/start',
                                                                    QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.state : State = State.CREAT_LOG_BLOGS

        self.z_range_subscriber = self.create_subscription(    GenericLogData,
                                                               f'/{self.id}/log/z/data',
                                                               self.z_range_callback,
                                                               10)

        self.z_publisher = self.create_publisher(   Point,
                                                    '/grid/z',
                                                    10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def z_range_callback(self,msg):
        z_pos = Point()
        if len(msg.values) < 3: return
        pos = self.get_drone_position()
        if len(pos) > 0 and abs(msg.values[1]) < 3.0 and abs(msg.values[2]) < 3.0:
            z_pos.x = pos[0]
            z_pos.y = pos[1]
            z_pos.z = (pos[2] * 1000.0 - msg.values[0] - 35.0) / 1000.0
            self.z_publisher.publish(z_pos)

    def get_drone_position(self) -> List[float]:
        try:
            t = self.tf_buffer.lookup_transform('world', 
                                                self.id,
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
            log_block_msg.variables = ['range.zrange', 'gyro.x', 'gyro.y', 'acc.z']
            log_block_msg.name = 'z'
            self.create_z_logblock_publisher.publish(log_block_msg)
            self.state = State.WAIT1
        elif self.state == State.WAIT1:
            self.state = State.WAIT2
        elif self.state == State.WAIT2:
            self.state = State.START_LOG_BLOGS
        elif self.state == State.START_LOG_BLOGS:
            sample_rate = Int16()
            sample_rate.data = 15
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