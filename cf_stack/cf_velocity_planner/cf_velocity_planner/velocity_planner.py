import rclpy
from rclpy.node import Node

from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from std_msgs.msg import Float32

from dataclasses import dataclass
import math

@dataclass
class Pose:
    x : float
    y : float

class VelocityPlanner(Node):
    def __init__(self):
        super().__init__("velocity_planner")
        self.get_logger().info("Started Velocity Planner")

        self.future_points_subscriber = self.create_subscription(    SegmentListMsg,
                                                                    '/path/future', 
                                                                    self.future_points_callback,
                                                                    10);
    
        self.angle_publisher = self.create_publisher(    Float32,
                                                        '/path/angle',
                                                        10)

        self.ppc_velocity_publisher = self.create_publisher(    Float32,
                                                                '/ppc/velocity',
                                                                10)
        
        self.target_velocity : float = 0.2
        self.act_velocity : float = 0.2
        self.velocity_inc : float = 0.03

    def get_angle_abc(self, a: SegmentMsg, b: SegmentMsg, c: SegmentMsg) -> float:
        ab_x = b.x - a.x
        ab_y = b.y - a.y
        bc_x = c.x - b.x
        bc_y = c.y - b.y

        dot_product = ab_x * bc_x + ab_y * bc_y

        magnitude_ab = math.sqrt(ab_x**2 + ab_y**2)
        magnitude_bc = math.sqrt(bc_x**2 + bc_y**2)

        cos_theta = dot_product / (magnitude_ab * magnitude_bc)
        cos_theta = max(-1.0, min(1.0, cos_theta))  # Numerische Stabilität
        angle_rad = math.acos(cos_theta)

        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def get_velocity(self, angle : float) -> float:
        if angle >= 0 and angle < 44.0:
            return 0.30
        elif angle >= 44 and angle < 90:
            return 0.10
        else:
            return 0.20

    def future_points_callback(self, msg : SegmentListMsg):
        angle : float = self.get_angle_abc(msg.segments[0], msg.segments[1], msg.segments[2])

        path_angle : Float32 = Float32()
        path_angle.data = angle
        self.angle_publisher.publish(path_angle)

        self.target_velocity = self.get_velocity(angle)

        if self.target_velocity - self.act_velocity > 0:
            self.act_velocity += self.velocity_inc
        elif self.target_velocity - self.act_velocity < 0:
            self.act_velocity = self.target_velocity


        ppc_velocity : Float32 = Float32()
        ppc_velocity.data = self.act_velocity
        self.ppc_velocity_publisher.publish(ppc_velocity)
        

def main():
    rclpy.init()
    velocityPlanner = VelocityPlanner()

    try:
        rclpy.spin(velocityPlanner)
        velocityPlanner.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()