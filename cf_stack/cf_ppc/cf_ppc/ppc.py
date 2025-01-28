import rclpy
from dataclasses import dataclass
from typing import List, Tuple
from rclpy.node import Node
import math
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

@dataclass
class Pose:
    x : float
    y : float
    is_edge_pose : bool


class PPC(Node):
    def __init__(self):
        super().__init__("PPC")
        self.get_logger().info("Started PPC")

        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                            '/path/opt',
                                                            self.segment_list_callback,
                                                            10)
        

        
        self.ppc_publisher = self.create_publisher( Point,
                                                    '/ppc',
                                                    10)
        
        self.future_points = self.create_publisher( SegmentListMsg,
                                                    '/path/future',
                                                    10)
        

        self.ppc_velocity_subscriber = self.create_subscription(    Float32,
                                                                    '/ppc/velocity',
                                                                    self.ppc_velocity_callback,
                                                                    10)
        
        self.ppc_points : List[Pose] = []
        self.ppc_points_pose : int = 0


        self.main_path_points : List[Pose] = []
        self.main_path_pose : int = 0

        self.local_path_points : List[Pose] = []
        self.local_path_pose : int = 0

        self.ppc_path_done : bool = False
        self.delta_dist : float = 0.001

        self.ppc_delta_time : float = 0.003
        self.timer = self.create_timer( self.ppc_delta_time,
                                        self.handle_ppc)




    def ppc_velocity_callback(self, msg : Float32):
        self.delta_dist = msg.data * self.ppc_delta_time


    def get_distance(self, a : SegmentMsg, b : SegmentMsg) -> float:
        delta_x : float = b.x - a.x
        delta_y : float = b.y - a.y
        return math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
    
    def get_angle(self, a : SegmentMsg, b : SegmentMsg) -> float:
        delta_x : float = b.x - a.x
        delta_y : float = b.y - a.y
        if delta_x != 0.0 and delta_y!= 0.0:
            return math.atan2(delta_y, delta_x)
        elif delta_x == 0.0 and delta_y > 0.0:
            return  math.pi / 2
        elif delta_x == 0.0 and delta_y < 0.0:
            return  math.pi / 2 * 3
        elif delta_x > 0.0 and delta_y == 0.0:
            return  0
        elif delta_x < 0.0 and delta_y == 0.0:
            return  math.pi
        else:
            self.get_logger().log("Error")
            return 0
        
    def get_new_point(self, start_point : Pose, angle : float, distance : float) -> Pose:
        delta_x : float = math.cos(angle) * distance
        delta_y : float = math.sin(angle) * distance
        return Pose(x=start_point.x + delta_x, y=start_point.y + delta_y, is_edge_pose=False)
    
    def get_local_path_points(self, a : SegmentMsg, b : SegmentMsg) -> List[Pose]:
        path_points : List[Pose] = []
        dist : float = self.get_distance(a=a, b=b)
        steps : int = (int)(dist / self.delta_dist) - 1
        angle : float = self.get_angle(a=a, b=b)
        start_pose : Pose = Pose(a.x, a.y, is_edge_pose=True)
        path_points.append(start_pose)
        for _ in range(steps):
            new_pose : Pose = self.get_new_point(start_point=start_pose, angle=angle, distance=self.delta_dist)
            path_points.append(new_pose)
            start_pose = new_pose
        return path_points

    def segment_list_callback(self, msg: SegmentListMsg):
        if self.ppc_path_done == False:
            for a in msg.segments:
                self.main_path_points.append(Pose(x=a.x, y=a.y, is_edge_pose=True))
            self.ppc_path_done = True



    def publish_ppc_point(self, pose : Pose):
        point = Point()
        point.x = pose.x
        point.y = pose.y
        point.z = 0.1
        self.ppc_publisher.publish(point)

    def publish_future_points(self):
        future_poses : List[Pose] = [   self.main_path_points[self.main_path_pose],
                                        self.main_path_points[self.main_path_pose + 1],
                                        self.main_path_points[self.main_path_pose + 2],]

        segments = SegmentListMsg()
        for pose in future_poses:
            segment = SegmentMsg()
            segment.x = pose.x
            segment.y = pose.y
            segments.segments.append(segment)
            
        self.future_points.publish(segments)

    def handle_ppc(self):
        if len(self.local_path_points) == 0 and len(self.main_path_points) >= self.main_path_pose + 1:
            self.local_path_points = self.get_local_path_points(    self.main_path_points[self.main_path_pose], 
                                                                    self.main_path_points[self.main_path_pose + 1])
            
            self.main_path_pose += 1
            self.local_path_pose = 0

            if self.main_path_pose + 2 <= len(self.main_path_points):
                self.publish_future_points()

        if self.local_path_pose < len(self.local_path_points):
            self.publish_ppc_point(pose=self.local_path_points[self.local_path_pose])
            self.local_path_pose += 1

        if self.local_path_pose == len(self.local_path_points):
            self.local_path_points = []



        
    
        

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