import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from typing import List

from cf_path_planning.grid import Grid

import json

import time


class PathPlanning(Node):
    def __init__(self):
        super().__init__("PathPlanning")
        self.get_logger().info("Test Path Planning")

        self.declare_parameter(name='input_grid_path', value='default.json')

        grid_path : str = self.get_parameter('input_grid_path').get_parameter_value().string_value

        self.grid : Grid = Grid(    x_size_offset=0.2,
                                    y_size_offset=0.2,
                                    x_segment_size=0.2,
                                    y_segment_size=0.2,
                                    json_file=self.get_json_file(input_grid_path=grid_path),
                                    node=self)
        
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 1000)

        self.publish_segments()


    def get_json_file(self, input_grid_path : str):
        try:
            with open(input_grid_path, 'r') as file:
                return json.loads(file.read())
        except FileNotFoundError:
            raise RuntimeError(f'File: {input_grid_path} not found')
        except json.JSONDecodeError as e:
            raise RuntimeError(f'Json Decode Error: {e}')
        
        return None


    def publish_segments(self):

        for count, marker in enumerate(self.grid.get_rviz_marker_messages()):
            self.publisher.publish(marker)

def main():
    rclpy.init()
    pathPlanning = PathPlanning()

    try:
        while rclpy.ok():
            rclpy.spin_once(pathPlanning, timeout_sec=0)
        pathPlanning.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
