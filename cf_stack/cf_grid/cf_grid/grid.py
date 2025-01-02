import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cf_messages.msg import SegmentMsg
from cf_messages.msg import SegmentListMsg
from cf_grid.segment import Segment
from typing import List
import json

class Grid(Node):

    def __init__(self):
        super().__init__("Grid")
        self.get_logger().info("Started Grid")

        #Parameter
        self.declare_parameter(name='input_grid_path', value='default.json')
        self.declare_parameter(name='x_segment_size', value=0.2)
        self.declare_parameter(name='y_segment_size', value=0.2)
        self.declare_parameter(name='x_size_offset', value=0.2)
        self.declare_parameter(name='y_size_offset', value=0.2)

        self.input_grid_path: str = self.get_parameter('input_grid_path').get_parameter_value().string_value
        self.x_segment_size: float = self.get_parameter('x_segment_size').get_parameter_value().double_value
        self.y_segment_size: float = self.get_parameter('y_segment_size').get_parameter_value().double_value
        self.x_size_offset: float = self.get_parameter('x_size_offset').get_parameter_value().double_value
        self.y_size_offset: float = self.get_parameter('y_size_offset').get_parameter_value().double_value

        self.get_logger().info(f"Input Grid Path: {self.input_grid_path}")
        self.get_logger().info(f"X Segment Size: {self.x_segment_size}")
        self.get_logger().info(f"Y Segment Size: {self.y_segment_size}")
        self.get_logger().info(f"X Size Offset: {self.x_size_offset}")
        self.get_logger().info(f"Y Size Offset: {self.y_size_offset}")


        self.segments : Segment = []

        self.grid = self.get_grid_out_of_json_file(json_file=self.get_json_file(input_grid_path=self.input_grid_path))

        self.create_grid_of_segments(   x_size_offset=self.x_size_offset,
                                        y_size_offset=self.y_size_offset,
                                        x_segment_size=self.x_segment_size,
                                        y_segment_size=self.y_segment_size,
                                        grid=self.grid)
        
        self.segment_publisher = self.create_publisher(SegmentListMsg, '/segments', 10)
        
        self.timer = self.create_timer(1.0, self.publish_segments)
        

    def get_grid_out_of_json_file(  self,
                                    json_file) -> List[List[bool]]:
        grid : List[List[bool]] = []
        for row in json_file['arena']['segments']:
            seg_in_row : List[bool] = []
            for seg in row:
                seg_in_row.append(seg['obstacle'])
            grid.append(seg_in_row)
        return grid
    
    def get_json_file(self, input_grid_path : str):
        try:
            with open(input_grid_path, 'r') as file:
                return json.loads(file.read())
        except FileNotFoundError:
            raise RuntimeError(f'File: {input_grid_path} not found')
        except json.JSONDecodeError as e:
            raise RuntimeError(f'Json Decode Error: {e}')
        
        return None


    def create_grid_of_segments(    self, 
                                    x_size_offset : float,
                                    y_size_offset : float,
                                    x_segment_size : float,
                                    y_segment_size : float,
                                    grid : List[List[bool]]):
        for index_y, row in enumerate(grid):
            for index_x, seg in enumerate(row):
                segment : Segment = Segment(    x_pos=(x_segment_size * index_x) + x_size_offset + (x_segment_size / 2),
                                                y_pos=(y_segment_size * index_y) + y_size_offset + (y_segment_size / 2),
                                                z_pos=0.0,
                                                obstacle=seg,
                                                start=True if index_x == 0 and index_y == 0 else False)
                
                self.segments.append(segment)


    
    def publish_segments(self):
        pointList = SegmentListMsg()
        for seg in self.segments:
            pointList.segments.append(seg.position_message())
        self.segment_publisher.publish(pointList)
    
def main():
    rclpy.init()
    pathPlanning = Grid()

    try:
        rclpy.spin(pathPlanning)
        pathPlanning.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
