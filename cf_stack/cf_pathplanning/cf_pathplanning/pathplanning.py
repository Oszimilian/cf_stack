import rclpy
from rclpy.node import Node
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from dataclasses import dataclass
from typing import List, Dict
from cf_pathplanning.coverage_planner import CoveragePlanner, HeuristicType
import copy

@dataclass
class PathPlanningSegment:
    x : float
    y : float
    z : float
    x_id : int
    y_id : int
    obstacle : bool
    start : bool
    

class PathPlanner(Node):
    def __init__(self):
        super().__init__("PathPlanner")
        self.get_logger().info("Started PathPlanner")

        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                           '/segments',
                                                           self.segment_list_callback,
                                                           10)
        
        self.path_publisher = self.create_publisher(    SegmentListMsg,
                                                        '/path',
                                                        10)
        
        self.grid : List[List[int]] = []
        self.segments : List[PathPlanningSegment] = []

        self.cp_heuristics = [HeuristicType.HORIZONTAL]
        self.orientations = [0, 1, 2, 3]
        


    def segment_list_callback(self, msg: SegmentListMsg):
        self.grid.clear()
        self.segments.clear()

        a = {}

        for segment in msg.segments:
            if segment.y in a:
                a[segment.y].append([segment.x, segment.obstacle, segment.start, segment.z])
            else:
                a[segment.y] = [[segment.x, segment.obstacle, segment.start, segment.z]]

        for key in a:
            a[key].sort(key=lambda item: item[0])
        sorted_keys = sorted(a.keys())

        for y_id, y in enumerate(sorted_keys):
            y_line : List[int] = []
            for x_id, value in enumerate(a[y]):
                x, obstacle,start, z = value
                pps = PathPlanningSegment(x=x, y=y, z=z, x_id=x_id, y_id=y_id, obstacle=obstacle, start=start)
                self.segments.append(pps)

                if obstacle == True and start == False:
                    y_line.append(1)
                elif obstacle == False and start == True:
                    y_line.append(2)
                else:
                    y_line.append(0)
            self.grid.append(y_line)

        self.publish_path()

    def get_path(self, heuristic, grid) -> List:
        compare_tb = []

        cp = CoveragePlanner(grid)

        for orientation in self.orientations:
            cp.start(initial_orientation=orientation, cp_heuristic=heuristic)
            cp.compute()

            res = [heuristic.name, orientation]
            res.extend(cp.result())
            compare_tb.append(res)

        compare_tb.sort(key=lambda x: (x[3], x[4]))

        return compare_tb[0][6]
    
    def is_close_to_zero(self, value : float, epsilon : float = 0.0001) -> bool:
        return value < epsilon and value > -epsilon
    
    def get_act_angle(self, act_pos : List, next_pos : List) -> float:
        #self.get_logger().info(f'{act_pos} ==> {next_pos}')
        x_diff : float = next_pos[0] - act_pos[0]
        y_diff : float = next_pos[1] - act_pos[1]
        if x_diff > 0 and self.is_close_to_zero(y_diff):
            return 0
        elif x_diff < 0 and self.is_close_to_zero(y_diff):
            return 180
        elif self.is_close_to_zero(x_diff) and y_diff > 0:
            return 90
        elif self.is_close_to_zero(x_diff) and y_diff < 0:
            return 270
        else:
            self.get_logger().info(f"Error: {x_diff} - {y_diff}")
            return -1

    
    def get_path_sections(self, path : List) -> List[List]:
        sections : List[List] = []
        if len(path) < 2: return [[]]
        angle : float = self.get_act_angle(path[0], path[1])

        section : List = []
        for a, b in zip(path[1:], path[2:]):
            new_angle : float = self.get_act_angle(a, b)
            if new_angle != angle:
                sections.append(section)
                section = []
            section.append(a)

        return sections
    
    def get_score_of_path(self, path) -> int:
        if len(path) == 0: return -1000000
        sections : List[List] = self.get_path_sections(path)
        sections.sort(key=len)
        sections.reverse()
        score = 225 - len(sections)
        for i in range(1, 225):
            for section in sections:
                if len(section) == i and i > 2:
                    score += i
        self.get_logger().info(f'Score: {score}')

        return score
    
    def get_best_path(self, grid) -> List:
        best_path : List = []
        for heuristic in [HeuristicType.HORIZONTAL, HeuristicType.VERTICAL]:
            path = self.get_path(heuristic=heuristic, grid=grid)
            if self.get_score_of_path(path) > self.get_score_of_path(best_path):
                best_path = path
        return best_path
    
    def insert_new_start_pose_in_grid(self, grid : List[List[int]], start_pose : List[int]) -> List[List[int]]:
        found_start : bool = False
        for y_id, y in enumerate(grid):
            for x_id, x in enumerate(y):
                if x == 2:
                    grid[y_id][x_id] = 1
                    found_start = True
        if found_start == False: return []
        grid[start_pose[1]][start_pose[0]] = 2
        return grid
        

    def get_opt_path(self) -> List:
        opt_path : List = []
        local_grid : List[List[int]] = copy.copy(self.grid)

        start_path : List = self.get_best_path(local_grid)
        opt_path.append(start_path[0])
        start_path = start_path[1:]

        while True:
            local_grid = self.insert_new_start_pose_in_grid(local_grid, opt_path[-1])
            if len(local_grid) == 0:
                return start_path
            path = self.get_best_path(local_grid)
            if len(path) == 0:
                break
            if self.get_score_of_path(opt_path + start_path) < self.get_score_of_path(opt_path + path):
                start_path = path
            if len(start_path) > 0:
                opt_path.append(start_path[0])
                start_path = start_path[1:]

        return opt_path
            
            
            




    def publish_path(self):

        segment_list = SegmentListMsg()
        for i in self.get_opt_path():
            segment = SegmentMsg()
            for seg in self.segments:
                if i[0] == seg.y_id and i[1] == seg.x_id:
                    segment.x = seg.x
                    segment.y = seg.y
                    segment.z = 0.1
                    segment.obstacle = False
                    segment.start = seg.start
                    break
            segment_list.segments.append(segment)

        self.path_publisher.publish(segment_list)


def main():
    rclpy.init()
    pathPlanning = PathPlanner()

    try:
        rclpy.spin(pathPlanning)
        pathPlanning.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()