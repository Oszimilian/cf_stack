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
    is_edge_pose : bool


class PPC(Node):
    def __init__(self):
        super().__init__("PPC")
        self.get_logger().info("Started PPC")

        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                            '/path/opt',
                                                            self.segment_list_callback,
                                                            10)
        
        self.timer = self.create_timer( 0.0035,
                                        self.handle_ppc)
        
        self.ppc_publisher = self.create_publisher( Point,
                                                    '/ppc',
                                                    10)
        
        self.future_points = self.create_publisher( SegmentListMsg,
                                                    '/path/future',
                                                    10)
        
        self.ppc_points : List[Pose] = []
        self.ppc_points_pose : int = 0
        self.ppc_path_done : bool = False
        self.delta_dist : float = 0.001


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
    
    def segment_list_callback(self, msg: SegmentListMsg):
        if self.ppc_path_done == False:
            for a, b in zip(msg.segments, msg.segments[1:]):
                dist : float = self.get_distance(a=a, b=b)
                steps : int = (int)(dist / self.delta_dist) - 1
                angle : float = self.get_angle(a=a, b=b)
                start_pose : Pose = Pose(a.x, a.y, is_edge_pose=True)
                self.ppc_points.append(start_pose)
                for i in range(steps):
                    new_pose : Pose = self.get_new_point(start_point=start_pose, angle=angle, distance=self.delta_dist)
                    self.ppc_points.append(new_pose)
                    start_pose = new_pose
            self.ppc_path_done = True

    def get_future_edge_poses(self, start_ppc_pose : int, count : int) -> List[Pose]:
        edge_poses : List[Pose] = []
        for pose in self.ppc_points[start_ppc_pose:]:
            if pose.is_edge_pose == True:
                edge_poses.append(pose)
                count -= 1
                if count == 0:
                    return edge_poses
        return edge_poses

    def handle_ppc(self):
        if self.ppc_points_pose < len(self.ppc_points):
            point = Point()
            point.x = self.ppc_points[self.ppc_points_pose].x
            point.y = self.ppc_points[self.ppc_points_pose].y
            point.z = 0.1
            self.ppc_publisher.publish(point)

            if self.ppc_points[self.ppc_points_pose].is_edge_pose == True:
                future_poses : List[Pose] = self.get_future_edge_poses(self.ppc_points_pose, 3)

                segments = SegmentListMsg()
                for pose in future_poses:
                    segment = SegmentMsg()
                    segment.x = pose.x
                    segment.y = pose.y
                    segments.segments.append(segment)
                    
                self.future_points.publish(segments)

            self.ppc_points_pose += 1


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