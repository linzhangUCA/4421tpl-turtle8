import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from turtlesim.srv import SetPen
from math import pi


class Figure8Node(Node):
    def __init__(self):
        super().__init__("turtle8")
        self.set_red_cli = self.create_client(SetPen, "/turtle1/set_pen")
        while not self.set_red_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.set_red_req()

        # self.pose_listener = self.create_subscription(
        #     Pose, "/turtle1/pose", self.pin_turtle, 1
        # )
        # self.cmd_talker = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        # self.cmd_pub_timer = self.create_timer(0.05, self.cmd_pub)
        # # Variables
        # self.circle_counter = 0
        # self.ang_z = pi / 4
        # self.reverse_circle = False

    def set_red_req(self):
        "Set pen color to red, width to 2"
        req = SetPen.Request()
        req.r = 255
        req.width = 2
        # Call the service
        future = self.set_red_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # Check if the service call was successful
        if future.result() is not None:
            self.get_logger().info(f"Set pen color to {req}")
        else:
            self.get_logger().error("Failed to call service /turtle1/set_pen")

    # def pin_turtle(self, pose_msg):
    #     self.get_logger().info(f"Turtle's status: \n{pose_msg}")
    #
    # def cmd_pub(self):
    #     twist_msg = Twist()
    #     if not (self.circle_counter + 1) % 160:
    #         self.reverse_circle = not self.reverse_circle
    #     if self.reverse_circle:
    #         twist_msg.linear.x = pi / 2
    #         twist_msg.angular.z = -self.ang_z
    #     else:
    #         twist_msg.linear.x = pi / 4
    #         twist_msg.angular.z = self.ang_z
    #     self.cmd_talker.publish(twist_msg)
    #     self.get_logger().debug(f"Velocity command: {twist_msg}")
    #     self.circle_counter += 1


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
