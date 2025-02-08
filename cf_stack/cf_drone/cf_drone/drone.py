import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from crazyflies_interfaces.msg import SendTarget
from std_msgs.msg import Empty
from std_msgs.msg import String
from enum import Enum
from typing import List

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class State(Enum):
    IDLE = 0
    TAKEOFF = 1
    FLY = 2
    LAND = 3
    EXIT = 4
    ERROR = 5

class Drone(Node):

    def __init__(self):
        super().__init__('Drone')
        self.get_logger().info('Started Drone')

        self.declare_parameter(name='id', value=0)
        self.id : int = self.get_parameter('id').get_parameter_value().integer_value
        self.get_logger().info(f"ID: {self.id}")

        self.ppc_pos_subscriber = self.create_subscription( Point,
                                                            '/ppc',
                                                            self.ppc_pos_callback,
                                                            10)
        
        self.drone_pos_publisher = self.create_publisher(   SendTarget,
                                                            f'/safeflie{self.id}/send_target',
                                                            10)
        
        self.takeoff_pub = self.create_publisher(   Empty, 
                                                    f"/safeflie{self.id}/takeoff",
                                                    10)
        
        self.land_pub = self.create_publisher(  Empty,
                                                f"/safeflie{self.id}/land",
                                                10)
        
        self.takeoff_cmd_subscriber = self.create_subscription( Empty,
                                                                '/grid/detect',
                                                                self.takeoff_cmd_callback,
                                                                10)
        
        self.takeoff_done_publisher = self.create_publisher(    Empty,
                                                                '/drone/takeoffdone',
                                                                10)
        
        self.land_cmd_subscriber = self.create_subscription(    Empty,
                                                                '/ppc/flydone',
                                                                self.land_cmd_callback,
                                                                10)
        
        self.state_publisher = self.create_publisher(   String,
                                                        '/drone/state',
                                                        10)
        
        self.state_timer = self.create_timer(   0.5,
                                                self.state_callback)
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_name : str = f'cf{int(self.id)}'

        self.drone_pos_timer = self.create_timer(   0.02,
                                                    self.pos_callback)
        
        self.start_delay_timer = self.create_timer( 5.0,
                                                    self.start_delay_callback)
        
        self.pos_publisher= self.create_publisher(   Point,
                                                            '/drone/pos',
                                                            10)
        

        self.state : State = State.IDLE
        
        self.state_callback()

    def get_drone_position(self) -> List[float]:
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

    def pos_callback(self):
        pos = self.get_drone_position()
        if len(pos) == 0: return 
        point_msg = Point()
        point_msg.x = pos[0]
        point_msg.y = pos[1]
        point_msg.z = pos[2]
        self.pos_publisher.publish(point_msg)

    def state_callback(self):
        state_msg = String()
        match self.state:
            case State.IDLE:
                state_msg.data = "IDLE"
            case State.TAKEOFF:
                state_msg.data = "TAKEOFF"
            case State.FLY:
                state_msg.data = "FLY"
            case State.LAND:
                state_msg.data = "LAND"
            case State.EXIT:
                state_msg.data = "EXIT"
            case _:
                return
        self.state_publisher.publish(state_msg)

    def start_delay_callback(self):
        if self.state == State.TAKEOFF:
            self.state = State.FLY
            self.takeoff_done_publisher.publish(Empty())
        elif self.state == State.LAND:
            self.state = State.EXIT
            self.land_pub.publish(Empty())

    def takeoff_cmd_callback(self, msg : Empty):
        self.state = State.TAKEOFF
        self.takeoff_pub.publish(Empty())


    def land_cmd_callback(self, msg : Empty):
        self.state = State.LAND
        self.state_callback()

        
    def ppc_pos_callback(self, msg : Point):
        if self.state == State.FLY:
            self.send_target(x=msg.x, y=msg.y, z=1.0)

    def send_target(self, x : float, y : float, z : float):
        msg = SendTarget()
        msg.target.x = x
        msg.target.y = y
        msg.target.z = z
        msg.base_frame = "world"
        self.drone_pos_publisher.publish(msg)



def main():
    rclpy.init()
    drone = Drone()

    try:
        rclpy.spin(drone)
        drone.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()