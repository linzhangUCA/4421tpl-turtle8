import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pi


class Figure8Node(Node):
    def __init__(self):
        super().__init__("turtle8")
        self.pose_listener = self.create_subscription(
            Pose, "/turtle1/pose", self.pin_turtle, 1
        )
        self.cmd_talker = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.cmd_pub_timer = self.create_timer(0.05, self.cmd_pub)
        # Variables
        self.circle_counter = 0
        self.ang_z = pi / 4
        self.reverse_circle = False

    def pin_turtle(self, pose_msg):
        self.get_logger().info(f"Turtle's status: \n{pose_msg}")

    def cmd_pub(self):
        twist_msg = Twist()
        if not (self.circle_counter + 1) % 160:
            self.reverse_circle = not self.reverse_circle
        if self.reverse_circle:
            twist_msg.linear.x = pi / 2
            twist_msg.angular.z = -self.ang_z
        else:
            twist_msg.linear.x = pi / 4
            twist_msg.angular.z = self.ang_z
        self.cmd_talker.publish(twist_msg)
        self.get_logger().debug(f"Velocity command: {twist_msg}")
        self.circle_counter += 1


def main(args=None):
    rclpy.init(args=args)

    turtle8 = Figure8Node()

    rclpy.spin(turtle8)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle8.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
