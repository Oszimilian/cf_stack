import rclpy
from dataclasses import dataclass
from typing import List, Tuple
from rclpy.node import Node
import math
from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from std_msgs.msg import String
from enum import Enum

@dataclass
class Pose:
    x : float
    y : float
    is_edge_pose : bool

class State(Enum):
    IDLE = 0
    FLY = 1
    WAIT_EXIT = 2
    EXIT = 3


class PPC(Node):
    def __init__(self):
        super().__init__("PPC")
        self.get_logger().info("Started PPC")


        # subscriber for the not ideal path
        self.segment_subscriber = self.create_subscription( SegmentListMsg,
                                                            '/path/opt',
                                                            self.segment_list_callback,
                                                            10)
        
        # publisher for the ppc-point
        self.ppc_publisher = self.create_publisher( Point,
                                                    '/ppc',
                                                    10)
        
        # publisher for publishing futur path points to the velocity-planner
        self.future_points = self.create_publisher( SegmentListMsg,
                                                    '/path/future',
                                                    10)
        
        # subscriber for the target-velocity of the ppc-point fromt the velocityplanner
        self.ppc_velocity_subscriber = self.create_subscription(    Float32,
                                                                    '/ppc/velocity',
                                                                    self.ppc_velocity_callback,
                                                                    10)
        
        # subscriber for starting the ppc-point move
        self.fly_subscriber = self.create_subscription( Empty,
                                                        '/drone/takeoffdone',
                                                        self.fly_callback,
                                                        10)
        
        # publisher for publishing that the ppc-point hase finished track
        self.fly_done_publisher = self.create_publisher(    Empty,
                                                            '/ppc/flydone',
                                                            10)
        
        # publisher for publishing the state of the ppc
        self.state_publisher = self.create_publisher(   String,
                                                        '/ppc/state',
                                                        10)
        
        # timer for publishing the state of the node frequently
        self.state_timer = self.create_timer(   0.5,
                                                self.state_callback)
        
        # timer for waiting after finishing the path
        self.wait_timer = self.create_timer(    3,
                                                self.wait_timer)
        

        # List of main-path points. This means all points which came from the path-planning or opt-path-planning
        self.main_path_points : List[Pose] = []
        self.main_path_pose : int = 0

        # List of local-path points between two main-path points. This is usefull for adapting the velocity
        self.local_path_points : List[Pose] = []
        self.local_path_pose : int = 0

        self.ppc_path_done : bool = False
        
        # distance for dividing the path between two path-points
        self.delta_dist : float = 0.001

        # timer for handling the movement of the ppc-point
        self.ppc_delta_time : float = 0.003
        self.timer = self.create_timer( self.ppc_delta_time,
                                        self.handle_ppc)
        

        self.state : State = State.IDLE
        self.wait_done = False

        self.state_callback()

    def wait_timer(self):
        """!
        Function for waiting after finishing the track
        """
        if self.state == State.WAIT_EXIT:
            if self.wait_done == False: 
                self.wait_done = True
            else:
                self.state = State.EXIT
                
        elif self.state == State.EXIT:
            self.fly_done_publisher.publish(Empty())
            self.state_callback()

    def state_callback(self):
        """!
        Function for publishing the state of the node
        """
        state_msg = String()
        match self.state:
            case State.IDLE:
                state_msg.data = "IDLE"
            case State.FLY:
                state_msg.data = "FLY"
            case State.EXIT:
                state_msg.data = "EXIT"

            case _:
                return
        self.state_publisher.publish(state_msg)


    def fly_callback(self, msg : Empty):
        """!
        This callback comes from the drone after taking off and sets the ppc into fly
        """
        self.state = State.FLY
        self.state_callback()

    def ppc_velocity_callback(self, msg : Float32):
        """!
        This callback comes from the velocity planner and calcultes the delta dist for deviding the sub-paths
        into splices
        @param msg target velocity
        """
        self.delta_dist = msg.data * self.ppc_delta_time


    def get_distance(self, a : SegmentMsg, b : SegmentMsg) -> float:
        """!
        This function calcultes the distance between two points
        @param a point a
        @param b point b 
        @return distance
        """
        delta_x : float = b.x - a.x
        delta_y : float = b.y - a.y
        return math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
    
    def get_angle(self, a : SegmentMsg, b : SegmentMsg) -> float:
        """!
        This function calcultes the angle between two points in the world frame
        @param a point a
        @param b point b
        @return angle
        """
        delta_x : float = b.x - a.x
        delta_y : float = b.y - a.y
        if delta_x != 0.0 and delta_y!= 0.0:
            return math.atan2(delta_y, delta_x)
        elif delta_x == 0.0 and delta_y > 0.0:
            return  math.pi / 2
        elif delta_x == 0.0 and delta_y < 0.0:
            return  math.pi / 2 * 3
        elif delta_x > 0.0 and delta_y == 0.0:
            return  0
        elif delta_x < 0.0 and delta_y == 0.0:
            return  math.pi
        else:
            self.get_logger().log("Error")
            return 0
        
    def get_new_point(self, start_point : Pose, angle : float, distance : float) -> Pose:
        """!
        This function returns a new point based on a base-point the angle and the distance
        @param start_point point from were a new point has to be calculated
        @param angle 
        @param distance
        @return new point
        """
        delta_x : float = math.cos(angle) * distance
        delta_y : float = math.sin(angle) * distance
        return Pose(x=start_point.x + delta_x, y=start_point.y + delta_y, is_edge_pose=False)
    
    def get_local_path_points(self, a : SegmentMsg, b : SegmentMsg) -> List[Pose]:
        """!
        This function generates a list of new path-points between two main-path-points
        @param a main-path-point a
        @param b main-path-point b
        @return list of path-points
        """
        path_points : List[Pose] = []
        dist : float = self.get_distance(a=a, b=b)
        steps : int = (int)(dist / self.delta_dist) - 1
        angle : float = self.get_angle(a=a, b=b)
        start_pose : Pose = Pose(a.x, a.y, is_edge_pose=True)
        path_points.append(start_pose)
        for _ in range(steps):
            new_pose : Pose = self.get_new_point(start_point=start_pose, angle=angle, distance=self.delta_dist)
            path_points.append(new_pose)
            start_pose = new_pose
        return path_points

    def segment_list_callback(self, msg: SegmentListMsg):
        """!
        Callback for main-path-points
        @param msg main-path-points
        """
        # This is only done ones
        if self.ppc_path_done == False:
            for a in msg.segments:
                self.main_path_points.append(Pose(x=a.x, y=a.y, is_edge_pose=True))
            self.ppc_path_done = True

    def publish_ppc_point(self, pose : Pose):
        """!
        This function publishes the ppc-point
        @param pose of the ppc-point
        """
        point = Point()
        point.x = pose.x
        point.y = pose.y
        point.z = 1.0
        self.ppc_publisher.publish(point)

    def publish_future_points(self, depth : int):
        """!
        This function publishes a depending amount on future path-points to the velocity-planner
        @param depth var for how much future path-points are published
        """
        future_poses : List[Pose] = []

        # only if the path-points are enough
        if depth + self.main_path_pose > len(self.main_path_points):
            return 
        
        for i in range(depth):
            future_poses.append(self.main_path_points[self.main_path_pose + i])

        segments = SegmentListMsg()
        for pose in future_poses:
            segment = SegmentMsg()
            segment.x = pose.x
            segment.y = pose.y
            segments.segments.append(segment)
            
        self.future_points.publish(segments)

    def handle_ppc(self):
        """!
        This function handles the ppc-point position.
        For that it calcultes the local-path-points beween two main-path-points after the ppc-point finished the local-path-points.
        """
        if self.state == State.FLY:
            if len(self.main_path_points) > self.main_path_pose + 1:
                
                if len(self.local_path_points) == 0 :
                    self.local_path_points = self.get_local_path_points(    self.main_path_points[self.main_path_pose], 
                                                                            self.main_path_points[self.main_path_pose + 1])
                    
                    self.main_path_pose += 1
                    self.local_path_pose = 0
                    self.publish_future_points(depth=7)
            
            if self.local_path_pose < len(self.local_path_points):
                self.publish_ppc_point(pose=self.local_path_points[self.local_path_pose])
                self.local_path_pose += 1

            if self.local_path_pose == len(self.local_path_points):
                self.local_path_points = []
                if len(self.main_path_points) <= self.main_path_pose + 1:
                    self.state = State.WAIT_EXIT

def main():
    rclpy.init()
    ppc = PPC()

    try:
        rclpy.spin(ppc)
        ppc.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()