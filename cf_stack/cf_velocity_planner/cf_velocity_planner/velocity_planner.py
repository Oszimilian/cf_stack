import rclpy
from rclpy.node import Node

from cf_messages.msg import SegmentListMsg
from cf_messages.msg import SegmentMsg
from std_msgs.msg import Float32

from dataclasses import dataclass
from typing import List
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
        
        self.timer = self.create_timer( 0.05,
                                        self.velocity_pub)
        
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
            return 0.40
        elif angle >= 44 and angle < 90:
            return 0.10
        else:
            return 0.20

    def get_angles(self, msg: SegmentListMsg, samples: int) -> List[float]:
        angles: List[float] = []


        num_segments = len(msg.segments)
        if num_segments < 3:
            return angles

        max_index = min(samples, num_segments) - 2
        
        for i in range(max_index):
            a, b, c = msg.segments[i], msg.segments[i+1], msg.segments[i+2]
            angles.append(self.get_angle_abc(a=a, b=b, c=c))

        return angles
    
    def get_dyn_speed_factor(self, angles : List[float]) -> float:
        avg_angle : float = sum(angles) / len(angles)

        path_angle : Float32 = Float32()
        path_angle.data = avg_angle
        self.angle_publisher.publish(path_angle)

        speed_factor : float = 1.0


        if avg_angle < 5.0:
            speed_factor = 0.42
        elif avg_angle < 10.0:
            speed_factor = 0.38
        elif avg_angle < 15.0:
            speed_factor = 0.35
        elif avg_angle < 25.0: 
            speed_factor = 0.30
        elif avg_angle < 35.0:
            speed_factor = 0.25
        elif avg_angle < 45.0:
            speed_factor = 0.20
        else:
            speed_factor = 0.15


        return speed_factor

    def velocity_pub(self):
        if self.target_velocity - self.act_velocity > 0:
            self.act_velocity += 0.001
        else: 
            self.act_velocity -= 0.001

        ppc_velocity : Float32 = Float32()
        ppc_velocity.data = self.act_velocity
        self.ppc_velocity_publisher.publish(ppc_velocity)

    def future_points_callback(self, msg : SegmentListMsg):

        angles : List[float] = self.get_angles(msg=msg, samples=len(msg.segments))
        

        speed_factor : float = self.get_dyn_speed_factor(angles=angles)


        self.target_velocity = speed_factor





        

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