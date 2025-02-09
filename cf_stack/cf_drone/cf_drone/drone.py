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

        # get drone and safeflie id from launchfile
        self.declare_parameter(name='id', value=0)
        self.id : int = self.get_parameter('id').get_parameter_value().integer_value
        self.get_logger().info(f"ID: {self.id}")

        # subscriber for following the ppc-point
        self.ppc_pos_subscriber = self.create_subscription( Point,
                                                            '/ppc',
                                                            self.ppc_pos_callback,
                                                            10)
        
        # publisher for transmitting the target-position of the drone
        self.drone_pos_publisher = self.create_publisher(   SendTarget,
                                                            f'/safeflie{self.id}/send_target',
                                                            10)
        
        # publisher for takeoff the drone
        self.takeoff_pub = self.create_publisher(   Empty, 
                                                    f"/safeflie{self.id}/takeoff",
                                                    10)
        
        # publisher for landing the drone
        self.land_pub = self.create_publisher(  Empty,
                                                f"/safeflie{self.id}/land",
                                                10)
        
        # publisher for notifing other nodes that the drone was detect in the grid => start calculating path
        self.takeoff_cmd_subscriber = self.create_subscription( Empty,
                                                                '/grid/detect',
                                                                self.takeoff_cmd_callback,
                                                                10)
        
        # publisher for notifing that the drone has take of
        self.takeoff_done_publisher = self.create_publisher(    Empty,
                                                                '/drone/takeoffdone',
                                                                10)
        
        # subscriber for receiving the notification when the ppc finished the path
        self.land_cmd_subscriber = self.create_subscription(    Empty,
                                                                '/ppc/flydone',
                                                                self.land_cmd_callback,
                                                                10)
        
        # debug publisher for the state of this node
        self.state_publisher = self.create_publisher(   String,
                                                        '/drone/state',
                                                        10)
        
        # debug timer for publishing the state of the drone
        self.state_timer = self.create_timer(   0.5,
                                                self.state_callback)
        

        # tf_listener for receiving the drone position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_name : str = f'cf{int(self.id)}'

        # timer for updating the target position of the drone
        self.drone_pos_timer = self.create_timer(   0.02,
                                                    self.pos_callback)
        
        # timer for delaying the state transition between takeoff and fly and fly and land
        self.start_delay_timer = self.create_timer( 5.0,
                                                    self.start_delay_callback)
        
        # publisher for visualizing the drone position
        self.pos_publisher= self.create_publisher(  Point,
                                                    '/drone/pos',
                                                    10)
        # initial state
        self.state : State = State.IDLE


    def get_drone_position(self) -> List[float]:
        """!
        Try to read the drone position out of the tf-graph.
        If it is not possible, the state of the drone transit to ERROR
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

    def pos_callback(self):
        """!
        Callback for publishing the drone position, for visualisation
        """
        pos = self.get_drone_position()
        # return when no pos is available
        if len(pos) == 0: return 
        point_msg = Point()
        point_msg.x = pos[0]
        point_msg.y = pos[1]
        point_msg.z = pos[2]
        self.pos_publisher.publish(point_msg)

    def state_callback(self):
        """!
        Timer callback-function for publishing the state of the drone
        """
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
        """!
        Logic for state transition. 
        This callback is executed every 5 seconds to generate a delay between takeoff and fly and fly and land
        """
        if self.state == State.TAKEOFF:
            self.state = State.FLY
            self.takeoff_done_publisher.publish(Empty())
        elif self.state == State.LAND:
            self.state = State.EXIT
            self.land_pub.publish(Empty())

    def takeoff_cmd_callback(self, msg : Empty):
        """!
        Callback for transfering the drone into takeoff state
        @param msg empty signal for transfering the command
        """
        self.state = State.TAKEOFF
        self.takeoff_pub.publish(Empty())


    def land_cmd_callback(self, msg : Empty):
        """!
        Callback for transfering the drone into land state.
        The real execution is done after 5 seconds.
        This is because the drone needs some time to reach the position of the ppc
        @param msg empty signal for transfering the command
        """
        self.state = State.LAND
        self.state_callback()

        
    def ppc_pos_callback(self, msg : Point):
        """!
        Timer callback-function which updates the target-position of the drone
        @param msg position of the ppc-point
        """
        # only update if the drone is in fly-state
        if self.state == State.FLY:
            self.send_target(x=msg.x, y=msg.y, z=1.0)

    def send_target(self, x : float, y : float, z : float):
        """!
        Function which creates the position-message in the world frame
        @param x target x position
        @param y target y position
        @param z target z position
        """
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