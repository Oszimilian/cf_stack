import rclpy
from rclpy.node import Node

import rclpy.time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cf_messages.msg import SegmentMsg
from cf_messages.msg import SegmentListMsg
from cf_grid.segment import Segment
from typing import List
import json
from std_msgs.msg import Empty
from std_msgs.msg import String
from enum import Enum
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point

class State(Enum):
    DETECT = 0
    ERROR = 1
    CREATE = 2
    GRID = 3
    EXIT = 4

class Grid(Node):

    def __init__(self):
        super().__init__("Grid")
        self.get_logger().info("Started Grid")

        # node parameter from the launch-file
        self.declare_parameter(name='input_grid_path', value='default.json')
        self.declare_parameter(name='x_segment_size', value=0.2)
        self.declare_parameter(name='y_segment_size', value=0.2)
        self.declare_parameter(name='x_size_offset', value=0.2)
        self.declare_parameter(name='y_size_offset', value=0.2)
        self.declare_parameter(name='id', value=0)
        self.input_grid_path: str = self.get_parameter('input_grid_path').get_parameter_value().string_value
        self.x_segment_size: float = self.get_parameter('x_segment_size').get_parameter_value().double_value
        self.y_segment_size: float = self.get_parameter('y_segment_size').get_parameter_value().double_value
        self.x_size_offset: float = self.get_parameter('x_size_offset').get_parameter_value().double_value
        self.y_size_offset: float = self.get_parameter('y_size_offset').get_parameter_value().double_value
        self.id : int = self.get_parameter('id').get_parameter_value().integer_value

        self.get_logger().info(f"Input Grid Path: {self.input_grid_path}")
        self.get_logger().info(f"X Segment Size: {self.x_segment_size}")
        self.get_logger().info(f"Y Segment Size: {self.y_segment_size}")
        self.get_logger().info(f"X Size Offset: {self.x_size_offset}")
        self.get_logger().info(f"Y Size Offset: {self.y_size_offset}")
        self.get_logger().info(f"ID: {self.id}")

        # tf_listener for receiving the drone position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_name : str = f'cf{int(self.id)}'

        # List of Segments which represents the grid
        self.segments : List[Segment] = []

        # Reading the deliverd grid out of the json file
        self.grid : List[List[bool]] = self.get_grid_out_of_json_file(json_file=self.get_json_file(input_grid_path=self.input_grid_path))

        # subscriber for receiving constantly hight measurements
        self.z_measure_subscriber = self.create_subscription(   Point,
                                                                '/grid/z',
                                                                self.z_meassure_callback,
                                                                10)
        
        # publisher for publishing the segments (grid)
        self.segment_publisher = self.create_publisher(SegmentListMsg, '/segments', 10)

        # debug publisher for publishing the state of the grid
        self.state_publisher = self.create_publisher(   String,
                                                        '/grid/state',
                                                        10)
        
        # publisher for notifing other nodes that the drone has been successfully detected in the grid
        self.detect_publisher = self.create_publisher(  Empty,
                                                        '/grid/detect',
                                                        10)
        
        # timer for updating the grid
        self.timer = self.create_timer(1.0, self.timer_callback)

        # timer for publishing the state of the node
        self.state_timer = self.create_timer(   0.5,
                                                self.state_callback)


        self.start_x : int = 0
        self.start_y : int = 0

        self.state : State = State.DETECT
        self.state_callback()

    def state_callback(self):
        """!
        Timer callback-function for publishing the state of the grid
        """
        state_msg = String()
        match self.state:
            case State.DETECT:
                state_msg.data = "DETECT"
            case State.ERROR:
                state_msg.data = "ERROR"
            case State.CREATE:
                state_msg.data = "CREATE"
            case State.GRID:
                state_msg.data = "GRID"
            case State.EXIT:
                state_msg.data = "EXIT"
            case _:
                return
        self.state_publisher.publish(state_msg)

    def z_meassure_callback(self, msg : Point):
        """!
        Callback for the hight measurement
        @param msg measured point
        """
        # going over all segments of the grid and try to update the position of the right segment
        for section in self.segments:
            section.set_new_z(x=msg.x, y=msg.y, z=msg.z)
        

    def get_drone_position(self) -> List[float]:
        """!
        Try to read the drone position out of the tf-graph.
        If it is not possible, the state of the grid transit to ERROR
        @return list with the drone position x=0 y=1 z=2 or a empty list in case of an error
        """
        try:
            t = self.tf_buffer.lookup_transform('world', 
                                                self.tf_name,
                                                rclpy.time.Time())
            return [t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z]
        except Exception as ex:
            self.state = State.ERROR
            return []
        

    def get_grid_out_of_json_file(  self, json_file : str) -> List[List[bool]]:
        """!
        function for reading the json-file
        @param json_file object
        """
        
        # iterating over all rows in the json file and build a 2d-List-representation with booleans out of it
        grid : List[List[bool]] = []
        for row in json_file['arena']['segments']:
            seg_in_row : List[bool] = []
            for seg in row:
                seg_in_row.append(seg['obstacle'])
            grid.append(seg_in_row)
        return grid
    
    def get_json_file(self, input_grid_path : str):
        """!
        Function for open the json-file
        @param input_grid_path path of the json-file
        """
        try:
            with open(input_grid_path, 'r') as file:
                return json.loads(file.read())
        except FileNotFoundError:
            raise RuntimeError(f'File: {input_grid_path} not found')
        except json.JSONDecodeError as e:
            raise RuntimeError(f'Json Decode Error: {e}')
        
        return None


    def create_grid_of_segments(self):
        """!
        Function for creating a real-grid with positions and additional stuff like obstical of start-segement
        """
        # iterating of the 2-d List-object
        for index_y, row in enumerate(self.grid):
            for index_x, seg in enumerate(row):
                # creating a list-object with the position in the middle of a segement
                # the position depends on the segement-size and the grid-offset
                segment : Segment = Segment(    x_pos=(self.x_segment_size * index_x) + self.x_size_offset + (self.x_segment_size / 2),
                                                y_pos=(self.y_segment_size * index_y) + self.y_size_offset + (self.y_segment_size / 2),
                                                z_pos=0.0,
                                                x_size=self.x_segment_size,
                                                y_size=self.y_segment_size,
                                                obstacle=seg,
                                                start=True if index_x == self.start_x and index_y == self.start_y else False)
                # add segement to the grid-list
                self.segments.append(segment)

    
    def get_drone_start_index(self, drone_position : List[float]) -> bool:
        """!
        Function for getting the start-position of the drone in the grid
        @param drone_position list which includes the x-y-z koordinates of the drone
        """
        found : bool = False

        # return if the drone_position list is empty => drone cant be located
        if len(drone_position) == 0: return found

        # iterate over the 2-d grid from the json-file
        for index_y, y in enumerate(self.grid):
            for index_x, x in enumerate(y):
                # calculate the segement middle-point of the grid-segement
                xpos : float = (self.x_segment_size * index_x) + self.x_size_offset + (self.x_segment_size / 2)
                ypos : float = (self.y_segment_size * index_y) + self.y_size_offset + (self.y_segment_size / 2)

                # check if the drone is in the calculated segment or not
                if drone_position[0] > (xpos - (self.x_segment_size / 2)):
                    if drone_position[0] < (xpos + (self.x_segment_size / 2)):
                        if drone_position[1] > (ypos - (self.y_segment_size / 2)):
                            if drone_position[1] < (ypos + (self.y_segment_size / 2)):
                                self.start_x = index_x
                                self.start_y = index_y
                                found = True
                                break
        return found

    def publish_segments(self):
        """!
        function for creating a message which contains the segments of a grid
        """
        pointList = SegmentListMsg()
        for seg in self.segments:
            pointList.segments.append(seg.position_message())
        self.segment_publisher.publish(pointList)

    def timer_callback(self):
        """!
        Timer callback function which menages the state of the grid
        """
        # Try to detect the drone
        if self.state == State.DETECT:
            drone_position = self.get_drone_position()
            # If the drone was successfully detect => create the segemets of the grid
            if self.get_drone_start_index(drone_position=drone_position) == True:
                self.state = State.CREATE
                self.detect_publisher.publish(Empty())
            else:
                # Transit into error-state if location was not possible
                self.state = State.ERROR
        
        # After creating the grid, go to the state which updates the grid with hight-informations
        if self.state == State.CREATE:
            self.create_grid_of_segments()
            self.state = State.GRID

        # publish the grid frequently for visualization
        if self.state == State.GRID:
            self.publish_segments()

        self.state_callback()
    
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
