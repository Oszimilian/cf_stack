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

        # subscriber for listening to the segements
        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                           '/segments',
                                                           self.segment_list_callback,
                                                           10)
        
        # publisher for publishing the positions of the path
        self.path_publisher = self.create_publisher(    SegmentListMsg,
                                                        '/path',
                                                        10)
        
        # 2d-grid for the coverage path planning
        self.grid : List[List[int]] = []
        
        # list of segments of the grid
        self.segments : List[PathPlanningSegment] = []

        # possible orientations for movement in the pathplanning
        self.orientations = [0, 1, 2, 3]

    def shortest_path(self, grid) -> List:
        """!
        Algorithem for finding the shortes path in a grid from a to b.
        A is represented by a 2 in the grid
        B is represented by a 3 in the grid
        A obstical is a 1
        @param grid grid were the path have to be calculated
        @return path
        """
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
        """!
        This function is responsible for transforming the segementlist which represents the grid,
        to a 2-d grid which contains the informations for coverage path planning
        @param msg segementlist
        """
        a = {}

        # Create a dictionary with the y-koordinates as the keys 
        for segment in msg.segments:
            if segment.y in a:
                a[segment.y].append([segment.x, segment.obstacle, segment.start, segment.z])
            else:
                a[segment.y] = [[segment.x, segment.obstacle, segment.start, segment.z]]

        # Sort the x-koordinates inside the key and sort the keys
        for key in a:
            a[key].sort(key=lambda item: item[0])
        sorted_keys = sorted(a.keys())
        
        # create a grid out of the dictionary
        self.create_id_grid(grid=sorted_keys, sorted_segments=a)

    def create_id_grid(self, grid : Dict, sorted_segments : Dict):
        """!
        This function creates a 2-d list-object out of a dictionary whith positions and other informations
        """
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
        """!
        This function gets a segement-list and calculates and publishes the best path in the grid
        @param msg segementlist
        """
        self.grid.clear()
        self.segments.clear()

        # Transform the segmentlist into a grid
        self.transform_grid_msg_to_grid(msg=msg)

        # get best path in the grid
        coverage_path = self.get_opt_path()

        # append the back-path to the best path
        full_coverage_path = self.get_opt_full_path(path_coverage=coverage_path)

        # publish the path 
        self.publish_path(path=full_coverage_path)

    def get_path(self, heuristic, grid) -> List:
        """!
        This function calculates the opt path based on the coverage-path-planner algorithem 
        and returns the path
        @param heuristic heuristic for calculating the path
        @grid grid 
        @return list of path-points
        """
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
        """!
        Function to avoid rounding error around the zero
        @param value value which has to be checked if it is zero
        @param epsilon accepted blur around zero
        @return is it zero or not
        """
        return value < epsilon and value > -epsilon
    
    def get_act_angle(self, act_pos : List, next_pos : List) -> float:
        """!
        This function is responsible to calculate the angle in the worl-frame between two points.
        Only the angles 0, 90, 180 and 270 are possible because the path has only 4 directions
        @param act_pos pos a
        @param next_pos pos b 
        @return angle 
        """
        x_diff : float = next_pos[0] - act_pos[0]
        y_diff : float = next_pos[1] - act_pos[1]
        
        # select angles based on the x and y diffs
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
        """!
        This function returns all connected sections of a path with the same direction
        @param path 
        @return list of connected path sections
        """
        sections : List[List] = []
        
        #calculate the first angle between the path-points
        angle : float = self.get_act_angle(path[0], path[1])

        section : List = []
        # iterate over all positions with one position offset and do the same as above
        for a, b in zip(path[1:], path[2:]):
            new_angle : float = self.get_act_angle(a, b)
            if new_angle != angle:
                sections.append(section)
                section = []
            section.append(a)

        return sections
    
    def get_score_of_path(self, path) -> int:
        """!
        This function returns a score for a path
        @param path
        @return score of the path
        """
        # if no path is given the score is very low
        if len(path) == 0: return -100000000
        
        sections : List[List] = self.get_path_sections(path)
        sections.sort(key=len)
        sections.reverse()
        #calculate a score based on the amount of sections with different orientations which are not connected
        score = 225 - len(sections)
        #append a score based on the len of such sub-sections
        for i in range(1, 225):
            for section in sections:
                if len(section) == i and i > 2:
                    score += i
        for section in sections:
            if len(section) > 2:
                pass
        return score

    def get_opt_path(self) -> List:
        """!
        This function returns the fasted path for the drone based on the possible heuristics
        The score of each path is compered to return the fastest option
        @return path
        """
        best_path : List = []
        heuristic = HeuristicType.HORIZONTAL
        for h in [HeuristicType.HORIZONTAL, HeuristicType.VERTICAL, HeuristicType.MANHATTAN, HeuristicType.CHEBYSHEV]:
            path = self.get_path(h, self.grid)
            if self.get_score_of_path(path) > self.get_score_of_path(best_path):
                best_path = path
                heuristic = h
        return path
    
    def get_opt_full_path(self, path_coverage : List) -> List:
        """!
        This function appends a backpath to the start-point of the path to ensure a save lending
        @param path_coverage path which only covers the shortest path trough all sections
        @return full path
        """
        start = path_coverage[0]
        end = path_coverage[-1]

        tmp_grid = copy.copy(self.grid)

        tmp_grid[start[0]][start[1]] = 3
        tmp_grid[end[0]][end[1]] = 2

        back_path = self.shortest_path(grid=tmp_grid)

        if len(back_path) > 2:
            return path_coverage + back_path[1:]
        else:
            return path_coverage


    def publish_path(self, path : List = []):
        """!
        This function publishes the full path
        @param path
        """
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