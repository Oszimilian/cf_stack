from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


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

    def get_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'

        marker.ns = 'cube'
        marker.id = self.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x_pos
        marker.pose.position.y = self.y_pos
        marker.pose.position.z = self.z_pos
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0 if self.obstacle else 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    
