import rclpy
from rclpy.node import Node
import numpy as np

# ROS2 messages needed
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

# My libraries
import utils.ekf as ekf

# Custom messages
from rastreator_interfaces.msg import EKF


class ekf_sim(Node):

    def __init__(self):
        super().__init__('rastreator_simulation')

        ###############
        # YAML CONFIG
        ###############

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('velocity', 2.0),
                ('yaw_rate', 1.8),
                ('std_range', 0.3),
                ('std_bearing', 0.1),
                ('landmarks', '[[5, 10], [10, 5]]'),
                ('dt', 1.0)
            ])


        # Load parameters
        self.velocity = self.get_parameter_or('velocity', 2.0).value
        self.yaw_rate = self.get_parameter_or('yaw_rate', 1.8).value
        self.std_range = self.get_parameter_or('std_range', 0.3).value
        self.std_bearing = self.get_parameter_or('std_bearing', 0.1).value
        self.landmarks = self.get_parameter_or('landmarks', '[[5, 10], [10, 5]]').value
        self.dt = self.get_parameter_or('dt', 1.0).value

        ###############
        # PUB/SUB
        ###############

        # Publisher cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        # Publisher ekf
        self.estimation_pub = self.create_publisher(EKF, 'ekf', 10)
        # Subscriber odom (pos robot from odom topic)
        self.rastreator_pos = self.create_subscription(
            Odometry,
            'odom',
            self.rastreator_pos_callback,
            10)
        self.rastreator_pos  # prevent unused variable warning

        ###############
        # INIT
        ###############
        self.ekf_test = ekf.EKF(dt=self.dt, wheelbase=0.5) # EKF instance
        self.new_position = [0,0,0]
        self.first_run = True

        self.get_logger().info(f'Start EKF simulation test')

    ###################
    # CLASS FUNCTIONS
    ###################

    def rastreator_pos_callback(self, msg):
        '''Get odometry position &
           Call ekf_estimation function
        '''
        ekf = EKF()

        # Get new position
        self.new_position  = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.twist.twist.angular.z]

        # Only show start position
        if self.first_run:
            self.first_run = False
            self.get_logger().info(f'Rastreator START POS: {self.new_position}')

        # EKF Predict step (based on curret position & velocity and yaw rate) 
        self.ekf_test.predict(self.new_position, [self.velocity, self.yaw_rate])
        # EKF Update step (get state estimation)
        self.pos_estimated = self.ekf_test.update_odom_method(self.new_position)[0][0]

        ekf.true_state.x = self.new_position[0]
        ekf.true_state.y = self.new_position[1]
        ekf.true_state.a = self.new_position[2]
        ekf.estimated_state.x = self.pos_estimated[0]
        ekf.estimated_state.y = self.pos_estimated[1]
        ekf.estimated_state.a = self.pos_estimated[2]
        ekf.control.velocity = self.velocity
        ekf.control.yaw_rate = self.yaw_rate
        self.estimation_pub.publish(ekf)

    def timer_callback(self):
        '''When called publish constant velocity
        '''
        u = Twist()
        u.linear.x = self.velocity
        u.angular.z = self.yaw_rate
        self.cmd_vel_pub.publish(u)

def main(args=None):
    rclpy.init(args=args)

    run_ekf = ekf_sim()
    
    rclpy.spin(run_ekf)

    # Destroy the node explicitly
    ekf_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()