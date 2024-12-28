from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cf_messages.msg import SegmentMsg


class Segment:
    def __init__(   self,
                    x_pos : float,
                    y_pos : float,
                    z_pos : float,
                    obstacle : bool,
                    id : int ):
        
        self.x_pos : float = x_pos
        self.y_pos : float = y_pos
        self.z_pos : float = z_pos
        self.obstacle : bool = obstacle
        self.id : int = id

    def __str__(self):
        return f'x: {self.x_pos} y: {self.y_pos} z: {self.z_pos} id: {self.id}'

    
    def position_message(self) -> SegmentMsg:
        segment = SegmentMsg()
        segment.x = self.x_pos
        segment.y = self.y_pos
        segment.z = self.z_pos
        segment.obstacle = self.obstacle
        return segment

    
