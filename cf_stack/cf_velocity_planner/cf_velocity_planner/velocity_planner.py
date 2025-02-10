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

        # subscriber for future points
        self.future_points_subscriber = self.create_subscription(    SegmentListMsg,
                                                                    '/path/future', 
                                                                    self.future_points_callback,
                                                                    10);
    
        # debug publisher for the avg angle of the future points
        self.angle_publisher = self.create_publisher(    Float32,
                                                        '/path/angle',
                                                        10)

        # publisher for the target-ppc-point-velocity
        self.ppc_velocity_publisher = self.create_publisher(    Float32,
                                                                '/ppc/velocity',
                                                                10)
        
        # timer for publishing the target_velocity
        self.timer = self.create_timer( 0.05,
                                        self.velocity_pub)
        
        # target velocity
        self.target_velocity : float = 0.2
        
        # act velocity - velocity which is send by the ppc_veloicty_publisher
        self.act_velocity : float = 0.2
        
        # velocity increment for reaching the target-velocity with the act velocity
        self.velocity_inc : float = 0.03

    def get_angle_abc(self, a: SegmentMsg, b: SegmentMsg, c: SegmentMsg) -> float:
        """!
        This function calculates the angle between three points.
        Angle ab bc
        @param a point a
        @param b point b
        @param c point c
        @return angle
        """
        ab_x = b.x - a.x
        ab_y = b.y - a.y
        bc_x = c.x - b.x
        bc_y = c.y - b.y

        dot_product = ab_x * bc_x + ab_y * bc_y

        magnitude_ab = math.sqrt(ab_x**2 + ab_y**2)
        magnitude_bc = math.sqrt(bc_x**2 + bc_y**2)

        cos_theta = dot_product / (magnitude_ab * magnitude_bc)
        cos_theta = max(-1.0, min(1.0, cos_theta))
        angle_rad = math.acos(cos_theta)

        angle_deg = math.degrees(angle_rad)
        return angle_deg


    def get_angles(self, msg: SegmentListMsg) -> List[float]:
        """!
        This function calcultes all path-angles of a given path
        @param msg path-points
        @param list of angles
        """
        angles: List[float] = []
        samples = len(msg.segments)


        num_segments = len(msg.segments)
        if num_segments < 3:
            return angles

        max_index = min(samples, num_segments) - 2
        
        for i in range(max_index):
            a, b, c = msg.segments[i], msg.segments[i+1], msg.segments[i+2]
            angles.append(self.get_angle_abc(a=a, b=b, c=c))

        return angles
    
    def get_dyn_speed_factor(self, angles : List[float]) -> float:
        """!
        This function calculates a speed_factor based on the future path angles.
        @param angles futur path-angles
        @return velocity
        """
        avg_angle : float = sum(angles) / len(angles)

        path_angle : Float32 = Float32()
        path_angle.data = avg_angle
        self.angle_publisher.publish(path_angle)

        speed_factor : float = 1.0
        speed_factor_2 : float = 1.0


        if avg_angle < 5.0:
            speed_factor = 0.35
        elif avg_angle < 10.0:
            speed_factor = 0.33
        elif avg_angle < 15.0:
            speed_factor = 0.29
        elif avg_angle < 25.0: 
            speed_factor = 0.25
        elif avg_angle < 35.0:
            speed_factor = 0.23
        elif avg_angle < 45.0:
            speed_factor = 0.20
        else:
            speed_factor = 0.15


        return speed_factor * speed_factor_2

    def velocity_pub(self):
        """!
        This callback sends the target-speed to the ppc node
        For this, it minimises the error between act and target velocity
        """
        if self.target_velocity - self.act_velocity > 0:
            self.act_velocity += 0.001
        else: 
            self.act_velocity -= 0.001

        ppc_velocity : Float32 = Float32()
        ppc_velocity.data = self.act_velocity
        self.ppc_velocity_publisher.publish(ppc_velocity)

    def future_points_callback(self, msg : SegmentListMsg):
        """!
        This callback comes from the ppc and contains the futur path-points.
        Based on that, the target-speed is calculated
        @param msg future path points
        """
        angles : List[float] = self.get_angles(msg=msg)
        
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