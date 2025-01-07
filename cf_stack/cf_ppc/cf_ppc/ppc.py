import rclpy
from dataclasses import dataclass
from typing import List
from rclpy.node import Node
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg

@dataclass
class Point:
    x : float
    y : float


class PPC(Node):
    def __init__(self):
        super().__init__("PPC")
        self.get_logger().info("Started PPC")

        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                            '/path/opt',
                                                            self.segment_list_callback,
                                                            10)
        
        self.segments : List[SegmentMsg] = []
        self.pos : int = 0
        self.ppc_pos : Point = None


    def segment_list_callback(self, msg: SegmentListMsg):
        self.segments.clear()
        self.segments = msg.segments


    def set_ppc(self):
        
        for i in self.segments:

            self.ppc_pos = Point(x = i.x, y = i.y)



        self.timer = self.create_timer(1.0, self.)


        
    


def main():
    rclpy.init()
    ppc = PPC()

    try:
        rclpy.spin(ppc)
        ppc.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()