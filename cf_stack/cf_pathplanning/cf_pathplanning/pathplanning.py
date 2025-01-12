import rclpy
from rclpy.node import Node
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from dataclasses import dataclass
from typing import List, Dict
from cf_pathplanning.coverage_planner import CoveragePlanner, HeuristicType

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

        self.cp_heuristics = [HeuristicType.VERTICAL]
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

    def publish_path(self):
        compare_tb = []

        cp = CoveragePlanner(self.grid)

        for heuristic in self.cp_heuristics:
            for orientation in self.orientations:
                cp.start(initial_orientation=orientation, cp_heuristic=heuristic)
                cp.compute()

                res = [heuristic.name, orientation]
                res.extend(cp.result())
                compare_tb.append(res)

        compare_tb.sort(key=lambda x: (x[3], x[4]))

        segment_list = SegmentListMsg()
        for i in compare_tb[0][6]:
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