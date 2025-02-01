import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from crazyflies_interfaces.msg import SendTarget
from std_msgs.msg import Empty
from std_msgs.msg import String
from enum import Enum

class State(Enum):
    IDLE = 0
    TAKEOFF = 1
    FLY = 2
    LAND = 3
    EXIT = 4

class Drone(Node):

    def __init__(self):
        super().__init__('Drone')
        self.get_logger().info('Started Drone')

        self.ppc_pos_subscriber = self.create_subscription( Point,
                                                            '/ppc',
                                                            self.ppc_pos_callback,
                                                            10)
        
        self.drone_pos_publisher = self.create_publisher(   SendTarget,
                                                            '/safeflie0/send_target',
                                                            10)
        
        self.takeoff_pub = self.create_publisher(   Empty, 
                                                    "/safeflie0/takeoff",
                                                    10)
        
        self.land_pub = self.create_publisher(  Empty,
                                                '/safeflie0/land',
                                                10)
        
        self.takeoff_cmd_subscriber = self.create_subscription( Empty,
                                                                '/drone/takeoff',
                                                                self.takeoff_cmd_callback,
                                                                10)
        
        self.takeoff_done_publisher = self.create_publisher(    Empty,
                                                                '/drone/takeoffdone',
                                                                10)
        
        self.land_cmd_subscriber = self.create_subscription(    Empty,
                                                                '/drone/land',
                                                                self.land_cmd_callback,
                                                                10)
        
        self.state_publisher = self.create_publisher(   String,
                                                        '/drone/state',
                                                        10)
        
        self.state_timer = self.create_timer(   0.5,
                                                self.state_callback)
        

        self.state : State = State.IDLE
        
        self.state_callback()

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

    def takeoff_cmd_callback(self, msg : Empty):
        self.state = State.TAKEOFF
        self.takeoff_pub.publish(Empty())
        self.timer = self.create_timer( 5.0, self.timer_callback)
        self.state_callback()
        
    def timer_callback(self):
        if self.state == State.TAKEOFF:
            self.takeoff_done_publisher.publish(Empty())
            self.state = State.FLY
        elif self.state == State.LAND:
            self.state = State.EXIT
            self.get_logger().info("")
        self.state_callback()

    def land_cmd_callback(self, msg : Empty):
        self.state = State.LAND
        self.land_pub.publish(Empty())
        self.timer = self.create_timer( 5.0, self.timer_callback)
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