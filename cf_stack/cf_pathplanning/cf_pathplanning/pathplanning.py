import rclpy
from rclpy.node import Node
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from dataclasses import dataclass
from typing import List, Dict
from cf_pathplanning.coverage_planner import CoveragePlanner, HeuristicType
from collections import deque
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

    def shortest_path(self, grid):
        rows, cols = len(grid), len(grid[0])
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        start, end = None, None
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == 2:
                    start = (r, c)
                elif grid[r][c] == 3:
                    end = (r, c)
        
        if not start or not end:
            return None
        
        queue = deque([(start[0], start[1], [])])
        visited = set()
        visited.add(start)
        
        while queue:
            r, c, path = queue.popleft()
            path = path + [(r, c)]
            
            if (r, c) == end:
                return path
            
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1 and (nr, nc) not in visited:
                    queue.append((nr, nc, path))
                    visited.add((nr, nc))
        
        return []
        
    def transform_grid_msg_to_grid(self, msg: SegmentListMsg):
        a = {}

        for segment in msg.segments:
            if segment.y in a:
                a[segment.y].append([segment.x, segment.obstacle, segment.start, segment.z])
            else:
                a[segment.y] = [[segment.x, segment.obstacle, segment.start, segment.z]]

        for key in a:
            a[key].sort(key=lambda item: item[0])
        sorted_keys = sorted(a.keys())
        
        self.create_id_grid(grid=sorted_keys, sorted_segments=a)

    def create_id_grid(self, grid : Dict, sorted_segments : Dict):
        for y_id, y in enumerate(grid):
            y_line : List[int] = []
            for x_id, value in enumerate(sorted_segments[y]):
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

    def segment_list_callback(self, msg: SegmentListMsg):
        self.grid.clear()
        self.segments.clear()

        self.transform_grid_msg_to_grid(msg=msg)

        coverage_path = self.get_opt_path()

        full_coverage_path = self.get_opt_full_path(path_coverage=coverage_path)

        self.publish_path(path=full_coverage_path)

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
            self.get_logger().info("Error")
            return -1

    
    def get_path_sections(self, path : List) -> List[List]:
        sections : List[List] = []
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
        if len(path) == 0: return -100000000
        sections : List[List] = self.get_path_sections(path)
        sections.sort(key=len)
        sections.reverse()
        score = 225 - len(sections)
        for i in range(1, 225):
            for section in sections:
                if len(section) == i and i > 2:
                    score += i
        for section in sections:
            if len(section) > 2:
                pass
                #self.get_logger().info(f'{section}')
        return score

    def get_opt_path(self) -> List:
        best_path : List = []
        heuristic = HeuristicType.HORIZONTAL
        for h in [HeuristicType.HORIZONTAL, HeuristicType.VERTICAL, HeuristicType.MANHATTAN, HeuristicType.CHEBYSHEV]:
            path = self.get_path(h, self.grid)
            if self.get_score_of_path(path) > self.get_score_of_path(best_path):
                best_path = path
                heuristic = h
            

        return path
    
    def get_opt_full_path(self, path_coverage : List) -> List:
        start = path_coverage[0]
        end = path_coverage[-1]

        tmp_grid = copy.copy(self.grid)

        tmp_grid[start[1]][start[0]] = 3
        tmp_grid[end[1]][end[0]] = 2

        back_path = self.shortest_path(grid=tmp_grid)
        if len(back_path) > 2:
            return path_coverage + back_path[1:]
        else:
            return path_coverage


    def publish_path(self, path : List = []):

        segment_list = SegmentListMsg()
        for i in path:
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