import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
import math


class SpiralSearchNode(Node):
    """
    Spiral search strategy for peg-in-hole insertion.

    Publishes Twist commands that trace an expanding spiral in X-Y
    while applying a constant downward force in Z. When the peg drops
    into the hole (detected by a Z force drop), the spiral stops.
    """

    def __init__(self):
        super().__init__('spiral_search')

        # --- Parameters ---
        self.declare_parameter('linear_speed', 0.005)       # m/s tangential speed
        self.declare_parameter('spiral_growth', 0.0005)      # m per revolution growth
        self.declare_parameter('downward_force', -2.0)       # N push-down in Z
        self.declare_parameter('force_threshold', -0.5)      # N: insertion detected when Z-force drops above this
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('wrench_topic', '/ft_sensor')
        self.declare_parameter('rate', 50.0)                 # Hz

        self.linear_speed = self.get_parameter('linear_speed').value
        self.spiral_growth = self.get_parameter('spiral_growth').value
        self.downward_force = self.get_parameter('downward_force').value
        self.force_threshold = self.get_parameter('force_threshold').value
        rate = self.get_parameter('rate').value

        # Publisher
        cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Subscriber for force/torque feedback
        wrench_topic = self.get_parameter('wrench_topic').value
        self.wrench_sub = self.create_subscription(
            Wrench, wrench_topic, self.wrench_callback, 10)

        # State
        self.theta = 0.0          # current angle on the spiral (radians)
        self.z_force = 0.0        # latest Z-force reading
        self.inserted = False

        # Timer
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Spiral search node started.')

    def wrench_callback(self, msg: Wrench):
        self.z_force = msg.force.z

    def timer_callback(self):
        if self.inserted:
            return

        # Check if the peg has dropped into the hole
        if self.theta > 0.0 and self.z_force > self.force_threshold:
            self.get_logger().info('Insertion detected! Stopping spiral.')
            self.inserted = True
            # Publish a zero-velocity stop command
            self.cmd_pub.publish(Twist())
            return

        # Archimedean spiral: r = a * theta
        a = self.spiral_growth / (2.0 * math.pi)
        r = a * self.theta

        # Angular velocity: omega = linear_speed / r  (avoid division by zero)
        if r < 1e-6:
            omega = self.linear_speed / 1e-3  # bootstrap: tight circle
            r = 1e-3
        else:
            omega = self.linear_speed / r

        self.theta += omega * self.dt

        # Convert polar velocity to Cartesian X-Y velocities
        vx = self.linear_speed * (-math.sin(self.theta))
        vy = self.linear_speed * (math.cos(self.theta))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = self.downward_force   # constant push-down
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpiralSearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
