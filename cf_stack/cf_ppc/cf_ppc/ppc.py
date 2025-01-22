import rclpy
from dataclasses import dataclass
from typing import List, Tuple
from rclpy.node import Node
import math
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from geometry_msgs.msg import Point

@dataclass
class Pose:
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
        
        self.timer = self.create_timer( 0.001,
                                        self.handle_ppc)
        
        self.ppc_publisher = self.create_publisher( Point,
                                                    '/ppc',
                                                    10)
        
        self.delta_dist : float = 0.001
        
        self.segments : List[SegmentMsg] = []
        self.pos : int = -1
        self.ppc_pos : Pose = None
        self.act_dir : Pose = None

    def get_act_dir(self) -> Pose:
        x_diff : float = 0
        y_diff : float = 0
        if len(self.segments) > self.pos + 1:
            x_diff : float = self.segments[self.pos + 1].x - self.segments[self.pos].x
            y_diff : float = self.segments[self.pos + 1].y - self.segments[self.pos].y
        return Pose(x=x_diff, y=y_diff)
    
    def is_ppc_finish(self) -> bool:
            return True if len(self.segments) < self.pos + 1 else False
    
    def get_act_angle(self) -> float:
        if self.act_dir.x != 0.0 and self.act_dir.y != 0.0:
            return math.atan2(self.act_dir.y, self.act_dir.x)
        elif self.act_dir.x == 0.0 and self.act_dir.y > 0.0:
            return  math.pi / 2
        elif self.act_dir.x == 0.0 and self.act_dir.y < 0.0:
            return  math.pi / 2 * 3
        elif self.act_dir.x > 0.0 and self.act_dir.y == 0.0:
            return  0
        elif self.act_dir.x < 0.0 and self.act_dir.y == 0.0:
            return  math.pi
        else:
            self.get_logger().log("Error")
            return 0
    


    def get_future_Pose(self, dist : float) -> Pose:
        alpha : float = self.get_act_angle()
        delta_x : float = math.cos(alpha) * dist
        delta_y : float = math.sin(alpha) * dist
        return Pose(x=delta_x, y=delta_y)
    
    def is_ppc_pos_next_pos(self) -> bool:
        delta_x : float = self.segments[self.pos + 1].x - self.ppc_pos.x
        delta_y : float = self.segments[self.pos + 1].y - self.ppc_pos.y
        dist : float = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
        
        return True if dist <= self.delta_dist * 3 else False


    def segment_list_callback(self, msg: SegmentListMsg):
        self.segments.clear()
        self.segments = msg.segments
        if self.pos == -1:
            self.ppc_pos = Pose(x=msg.segments[0].x, y=msg.segments[0].y)
            self.pos = 0
            self.act_dir = self.get_act_dir()

    def handle_ppc(self):
        if len(self.segments) > (self.pos + 1):
            new_pos = self.get_future_Pose(dist=self.delta_dist)
            self.ppc_pos.x += new_pos.x
            self.ppc_pos.y += new_pos.y
            if self.is_ppc_pos_next_pos() == True:
                self.pos += 1
                self.act_dir = self.get_act_dir()



            point = Point()
            point.x = self.ppc_pos.x
            point.y = self.ppc_pos.y
            point.z = 0.1
            self.ppc_publisher.publish(point)




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