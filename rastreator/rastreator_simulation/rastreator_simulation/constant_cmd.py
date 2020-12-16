import rclpy
from rclpy.node import Node

# ROS2 messages needed
from geometry_msgs.msg import Twist

# My libraries
import utils.ekf as ekf


class ekf_sim(Node):

    def __init__(self):
        super().__init__('rastreator_simulation')
      
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('velocity', None),
                ('yaw_rate', None),
                ('std_range', None),
                ('std_bearing', None),
                ('landmarks', None),
                ('dt', None)
            ])


        # Load parameters
        self.velocity = self.get_parameter_or('velocity', 2.0).value
        self.yaw_rate = self.get_parameter_or('yaw_rate', 1.8).value
        self.std_range = self.get_parameter_or('std_range', 0.3).value
        self.std_bearing = self.get_parameter_or('std_bearing', 0.1).value
        self.landmarks = self.get_parameter_or('landmarks', '[[5, 10], [10, 5]]').value
        self.dt = self.get_parameter_or('dt', 1.0).value

        # Publisher cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Class parameters
        self.first_run = True
        
        self.get_logger().info(f'Start EKF simulation test')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.yaw_rate
    
        self.cmd_vel_pub.publish(msg)

        if self.first_run:
            self.get_logger().info(f'Publishing constant u = [{msg.linear.x}, {msg.angular.z}]')
            self.first_run = False


def main(args=None):
    rclpy.init(args=args)

    run_ekf = ekf_sim()

    rclpy.spin(run_ekf)

    # Destroy the node explicitly
    ekf_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()