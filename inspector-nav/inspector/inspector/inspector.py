import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class vel_filter(Node):
    def __init__(self):
        super().__init__('AZ_velocity_smoother')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel_inspector', self.vel_callback, 10)
        self.decay_factor = 1.5
        
        self.get_logger().info('AZ_velocity_smoother node started - listening to /cmd_vel_inspector and publishing to /cmd_vel')
        self.get_logger().info(f'Decay factor: {self.decay_factor}')

    def vel_callback(self, msg: Twist):
        
        # Create filtered message
        filtered_msg = Twist()
        filtered_msg.linear.x = msg.linear.x * self.decay_factor
        filtered_msg.linear.y = msg.linear.y * self.decay_factor
        filtered_msg.linear.z = msg.linear.z * self.decay_factor
        filtered_msg.angular.x = msg.angular.x * self.decay_factor
        filtered_msg.angular.y = msg.angular.y * self.decay_factor
        filtered_msg.angular.z = msg.angular.z * self.decay_factor

        # Publish filtered message 
        self.vel_pub.publish(filtered_msg)
        

def main(args=None):
    rclpy.init(args=args)
    filter_node = vel_filter()
    
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()