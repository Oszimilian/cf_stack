import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from crazyflies_interfaces.msg import SendTarget
from std_msgs.msg import Empty

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
        
        self.timer = self.create_timer( 5.0,
                                        self.timer_callback)
        
        self.enaple_target : float = False

        self.takeoff_pub.publish(Empty())

    def land(self):
        self.land_pub.publish(Empty())

    def timer_callback(self):
        self.enaple_target = True
        
    def ppc_pos_callback(self, msg : Point):
        if self.enaple_target == True:
            self.send_target(x=msg.x, y=msg.y, z=1.0)

    def send_target(self, x : float, y : float, z : float):
        msg = SendTarget()
        msg.target.x = x
        msg.target.y = y
        msg.target.z = z
        msg.base_frame = "world"
        self.drone_pos_publisher.publish(msg)

    def _sleep(self, duration: float) -> None:
        """Sleeps for the provided duration in seconds."""
        start = self.__time()
        end = start + duration
        while self.__time() < end:
            rclpy.spin_once(self, timeout_sec=0)


def main():
    rclpy.init()
    drone = Drone()

    try:
        rclpy.spin(drone)
        drone.land()
        drone.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()