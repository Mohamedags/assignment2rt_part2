import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("MoveRobotNode has been started")
        
        self.velocity = Twist()

    def set_velocity(self, linear_x, angular_yaw):
        """Set the linear and angular velocity for the robot."""
        self.velocity.linear.x = linear_x
        self.velocity.angular.z = angular_yaw
        self.get_logger().info(f"Setting velocity: linear=({linear_x}, {angular_yaw})")

    def send_velocity_for_10s(self):
        """Send velocity command for 10 seconds."""
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < 10:
            self.publisher_.publish(self.velocity)
            time.sleep(0.1)  
            
        self.get_logger().info("Stopping the robot after 10 seconds.")
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot by setting velocity to zero."""
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher_.publish(self.velocity)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    
    try:
        while True:  
            # User interface : input for linear and angular velocities (Only linear X and angular Yaw)
            print("\nEnter new velocities for the robot:")
            linear_x = float(input("Enter linear velocity X (m/s): "))

            angular_yaw = float(input("Enter angular velocity Yaw (rad/s): "))
            
            node.set_velocity(linear_x, angular_yaw)
                     
            node.send_velocity_for_10s()
            
            user_input = input("\nDo you want to enter new velocities? (yes/no): ").strip().lower()
            if user_input != "yes":
                break
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

