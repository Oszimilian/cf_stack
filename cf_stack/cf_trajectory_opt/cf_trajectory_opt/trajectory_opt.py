import rclpy
from rclpy.node import Node
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from dataclasses import dataclass
from typing import List, Tuple
import math

from enum import Enum



class TrajectoryOpt(Node):
    def __init__(self):
        super().__init__("TrajectoryOpt")
        self.get_logger().info("Started TrajectoryOpt")

        self.path_subscriber = self.create_subscription(    SegmentListMsg,
                                                            '/path', 
                                                            self.path_callback,
                                                            10)
        
        self.trajectory_opt_publisher = self.create_publisher(  SegmentListMsg,
                                                              '/path/opt',
                                                              10)
    

    def get_sub_path(self, points : List[SegmentMsg], start_pos : int) -> List[SegmentMsg]:
        if len(points) < start_pos + 3: return None
        return points[start_pos:start_pos + 3]
    
    def get_movement(self, x_diff : float, y_diff : float) -> int:
        if x_diff == 0 and y_diff > 0:
            return 2
        elif x_diff == 0 and y_diff < 0:
            return 0
        elif x_diff > 0 and y_diff == 0:
            return 3
        elif x_diff < 0 and y_diff == 0:
            return 1
        else:
            return 4
    
    # 0=>UP 1=> RIGHT 2=>DOWN 3=>LEFT 4=>ERROR 
    def get_sub_path_movements(self, points : List[SegmentMsg]) -> List[int]:
        if len(points) > 3: return []
        movements : List[int] = []

        x_diff = points[0].x - points[1].x
        y_diff = points[0].y - points[1].y

        movement = self.get_movement(x_diff=x_diff, y_diff=y_diff)
        if movement == 4: return []
        movements.append(movement)

        x_diff = points[1].x - points[2].x
        y_diff = points[1].y - points[2].y

        movement = self.get_movement(x_diff=x_diff, y_diff=y_diff)
        if movement == 4: return []
        movements.append(movement)

        return movements


    def get_opt_points(self, points : List[SegmentMsg]) -> List[Tuple[int, List[int]]]:
        opt_points : List[Tuple[int, List[int]]] = []
        for id, point in enumerate(points):
            sub_path = self.get_sub_path(points=points, start_pos=id)
            
            if sub_path == None: continue
            
            sub_path_movements = self.get_sub_path_movements(points=sub_path)
            
            if len(sub_path_movements) == 0: continue
            if sub_path_movements[0] == sub_path_movements[1]: continue
            opt_points.append((id + 1, sub_path_movements))
        return opt_points
    
    def get_adapted_segment(self, old_segment : SegmentMsg, pos : int, scale : float) -> SegmentMsg:
        segment = SegmentMsg()
        
        segment.x = old_segment.x
        segment.y = old_segment.y
        segment.z = old_segment.z

        match pos:
            case 0:
                segment.y += scale
            case 1:
                segment.x += scale
            case 2:
                segment.y -= scale
            case 3:
                segment.x -= scale

        return segment

    
    # 0=>UP 1=> RIGHT 2=>DOWN 3=>LEFT 4=>ERROR 
    def get_opti_points(self, points : List[SegmentMsg], opt_points : List[Tuple[int, List[int]]]) -> SegmentListMsg:
        opt1 : float = 0.1
        opt2 : float = 0.1
        count : int = 0
        segmentList = SegmentListMsg()
        for id, point in enumerate(points):
            if count < len(opt_points):
                if opt_points[count][0] == id:
                    match opt_points[count][1]:
                        
                        case [0, 3]: # UP - LEFT => DOWN - RIGHT - UP - LEFT
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                        case [0, 2]: # UP - DOWN => DOWN - RIGHT - UP - LEFT - DOWN
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                        case [0, 1]: # UP - RIGHT => DOWN - LEFT - UP - RIGHT
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt2))

                        case [1, 2]: # RIGHT - DOWN => LEFT - UP - RIGHT - DOWN
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                        case [1, 3]: # RIGTH - LEFT => LEFT - UP - RIGHT - DOWN - LEFT
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                        case [1, 0]: # RIGHT - DOWN => LEFT - DOWN - RIGHT - UP
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))

                        case [2, 3]: # DOWN - LEFT => UP - RIGHT - DOWN - LEFT
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                        case [2, 0]: # DOWN - UP => UP - RIGHT - DOWN - LEFT - UP
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                        case [2, 1]: # DOWN - RIGHT => UP - LEFT - DOWN - RIGHT
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt2))

                        case [3, 2]: # LEFT - DOWN => RIGHT - UP - LEFT - DOWN
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                        case [3, 1]: # LEFT - RIGHT => RIGHT - DOWN - LEFT - UP - RIGHT
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt1))
                        case [3, 0]: # LEFT - UP => RIGHT - DOWN - LEFT - UP
                            segmentList.segments.append(self.get_adapted_segment(point, 1, opt2))
                            segmentList.segments.append(self.get_adapted_segment(point, 2, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 3, opt1))
                            segmentList.segments.append(self.get_adapted_segment(point, 0, opt2))
                    count += 1
                else:
                    segmentList.segments.append(point)
            else:
                segmentList.segments.append(point)
        return segmentList
    
    def get_distance(self, point_a : SegmentMsg, point_b : SegmentMsg) -> float:
        x_diff : float = point_b.x - point_a.x
        y_diff : float = point_b.y - point_a.y
        return math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))
                    
    def remove_near_points(self, segments: SegmentListMsg) -> SegmentListMsg:
        rmv_point_ids: List[int] = []
        new_segments = SegmentListMsg()
        for idx, point in enumerate(segments.segments):
            if idx + 1 < len(segments.segments):
                next_point = segments.segments[idx + 1]
                if self.get_distance(point, next_point) < 0.001:
                    rmv_point_ids.append(idx)
        for idx, segment in enumerate(segments.segments):
            if idx not in rmv_point_ids:
                new_segments.segments.append(segment)
        
        return new_segments

    def path_callback(self, msg : SegmentListMsg):
        opt_points : List[Tuple[int, List[int]]] = self.get_opt_points(points=msg.segments)
        new_points : SegmentListMsg = self.get_opti_points(points=msg.segments, opt_points=opt_points)
        new_points = self.remove_near_points(new_points)
        self.trajectory_opt_publisher.publish(new_points)

                

        

        

def main():
    rclpy.init()
    trajectoryOpt = TrajectoryOpt()

    try:
        rclpy.spin(trajectoryOpt)
        trajectoryOpt.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()