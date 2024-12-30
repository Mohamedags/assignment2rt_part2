import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("MoveRobotNode has been started")
        
        self.velocity = Twist()
        
    def set_velocity(self, linear_x, linear_y, linear_z, angular_roll, angular_pitch, angular_yaw):
        """Set the linear and angular velocity for the robot."""
        self.velocity.linear.x = linear_x
        self.velocity.linear.y = linear_y
        self.velocity.linear.z = linear_z
        self.velocity.angular.x = angular_roll
        self.velocity.angular.y = angular_pitch
        self.velocity.angular.z = angular_yaw
        self.get_logger().info(f"Setting velocity: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_roll}, {angular_pitch}, {angular_yaw})")

    def send_velocity(self):
        """Send the velocity command."""
        self.publisher_.publish(self.velocity)
        self.get_logger().info("Publishing velocity command.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    
    try:
        # User interface : input for linear and angular velocities (6 values)
        linear_x = float(input("Enter linear velocity X (m/s): "))
        linear_y = float(input("Enter linear velocity Y (m/s): "))
        linear_z = float(input("Enter linear velocity Z (m/s): "))
        
        angular_roll = float(input("Enter angular velocity Roll (rad/s): "))
        angular_pitch = float(input("Enter angular velocity Pitch (rad/s): "))
        angular_yaw = float(input("Enter angular velocity Yaw (rad/s): "))
        
        node.set_velocity(linear_x, linear_y, linear_z, angular_roll, angular_pitch, angular_yaw)
        
        node.send_velocity()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

