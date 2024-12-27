from cf_path_planning.segment import Segment
from typing import List
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Grid:
    def __init__(   self, 
                    x_size_offset : float, 
                    y_size_offset : float,
                    x_segment_size : float,
                    y_segment_size : float,
                    json_file,
                    node : Node):
        
        self.segments : Segment = []
        self.node_ref = node

        self.x_segment_size = x_segment_size
        self.y_segment_size = y_segment_size

        grid = self.get_grid_out_of_json_file(json_file=json_file)

        self.create_grid_of_segments(   x_size_offset=x_size_offset,
                                        y_size_offset=y_size_offset,
                                        x_segment_size=x_segment_size,
                                        y_segment_size=y_segment_size,
                                        grid=grid)
        

    def get_grid_out_of_json_file(  self,
                                    json_file) -> List[List[bool]]:
        grid : List[List[bool]] = []
        for row in json_file['arena']['segments']:
            seg_in_row : List[bool] = []
            for seg in row:
                seg_in_row.append(seg['obstacle'])
            grid.append(seg_in_row)
        return grid


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
                                                id=index_y*len(grid) + index_x)
                
                self.segments.append(segment)

    def get_rviz_marker_messages(self) -> List[Marker]:
        marker_segments : List[Marker] = []
        for seg in self.segments:
            marker = seg.get_marker()
            marker.scale.x = self.x_segment_size * 0.5
            marker.scale.y = self.y_segment_size * 0.5
            marker.scale.z = 0.2
            marker.header.stamp = self.node_ref.get_clock().now().to_msg()
            marker_segments.append(marker)
            print(seg)
        return marker_segments