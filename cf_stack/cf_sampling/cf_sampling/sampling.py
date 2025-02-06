import rclpy
from rclpy.node import Node
from enum import Enum
from crazyflie_interfaces.msg import LogBlock
from std_msgs.msg import Int16

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