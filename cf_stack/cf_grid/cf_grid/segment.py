from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cf_messages.msg import SegmentMsg
import math
from typing import List

class Segment:
    def __init__(   self,
                    x_pos : float,
                    y_pos : float,
                    z_pos : float,
                    x_size : float,
                    y_size : float,
                    obstacle : bool,
                    start : bool):
        
        self.x_pos : float = x_pos
        self.y_pos : float = y_pos
        self.z_pos : float = z_pos
        self.x_size : float = x_size
        self.y_size : float = y_size
        self.start : float = start
        self.obstacle : bool = obstacle
        self.measure_distance : float = 100000000
        self.z_samples : List[float] = []


    def __str__(self):
        return f'x: {self.x_pos} y: {self.y_pos} z: {self.z_pos}'

    
    def position_message(self) -> SegmentMsg:
        segment = SegmentMsg()
        segment.x = self.x_pos
        segment.y = self.y_pos
        segment.z = 0.0 if len(self.z_samples) == 0 else sum(self.z_samples) / len(self.z_samples)
        segment.obstacle = self.obstacle
        segment.start = self.start
        return segment
    
    def is_in_segment(self, x : float, y : float) -> bool:
        if self.obstacle == True or self.start == True : return False
        if x < (self.x_pos - (self.x_size / 2)) or x > (self.x_pos + (self.x_size / 2)):
            return False
        if y < (self.y_pos - (self.y_size / 2)) or y > (self.y_pos + (self.y_size / 2)):
            return False
        return True
    
    def get_distance(self, x : float, y : float):
        x_diff = abs(self.x_pos - x)
        y_diff = abs(self.y_pos - y)

        return math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))

    def set_new_z(self, x : float, y : float, z : float):
        if self.is_in_segment(x=x, y=y):
            if z > 0.07:
                self.z_samples.append(z)
            #dist : float = self.get_distance(x=x, y=y)
            #if dist < self.measure_distance:
            #    self.measure_distance = dist
            #    self.z_pos = z
