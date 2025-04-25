import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import sys
import os

class ForceTorqueLogger(Node):
    def __init__(self, filename):
        super().__init__('force_torque_logger')
        self.filename = filename

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/xarm/uf_ftsensor_raw_states',  # <-- Replace with your actual topic
            self.listener_callback,
            10
        )

        # Open CSV and write header
        self.csvfile = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z'])

        self.get_logger().info(f"Logging to {self.filename}")

    def listener_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.writer.writerow([force.x, force.y, force.z, torque.x, torque.y, torque.z])

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package ft_sensor_listener.py <output_filename>")
        return

    filename = sys.argv[1]
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    filename+='.csv'
    data_dir = os.path.join(root_dir, 'data')
    filename = os.path.join(data_dir, filename)

    os.makedirs(data_dir, exist_ok=True)
    logger_node = ForceTorqueLogger(filename)

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
