import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import sys
import os
import threading
import serial
import time
import re

BAUDRATE = 115200

class ForceTorqueLogger(Node):
    def __init__(self, filename, duration_s=10):
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
        self.writer.writerow(['timestamp', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z'])

        self.get_logger().info(f"Logging to {self.filename}")
        self.create_timer(duration_s, self.stop_logging)

    def listener_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        print(force.z)
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        self.writer.writerow([timestamp, force.x, force.y, force.z, torque.x, torque.y, torque.z])

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()

    def stop_logging(self):
        self.get_logger().info("Finished logging. Shutting down.")
        self.csvfile.close()
        rclpy.shutdown()


def serial_logger_thread(port, serial_filename, duration_sec=10):
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"Povezano na {port} @ {BAUDRATE} baud.")
        print(f"Snimanje serijskih podataka u fajl: {serial_filename}")
    except serial.SerialException as e:
        print(f"Greška pri povezivanju sa serijskim portom: {e}")
        return

    with open(serial_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'x', 'y', 'z'])

        print("Pocetak snimanja serijskih podataka...")

        x = y = z = 0.0
        start_time = time.time()

        while time.time() - start_time < duration_sec:
            try:
                line = ser.readline().decode('utf-8').strip()

                if 'x :' in line:
                    x = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                elif 'y :' in line:
                    y = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                elif 'z :' in line:
                    z = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    writer.writerow([timestamp, x, y, z])
                    csvfile.flush()
                    # print(f"{timestamp} -> x: {x}, y: {y}, z: {z}")
                    print(f"{timestamp} -> z: {z}")
            except Exception as e:
                print(f"Greška u očitavanju serijskih podataka: {e}")
                continue

        ser.close()
        print("Serijsko snimanje završeno.")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package ft_sensor_listener.py <output_filename>")
        return

    filename = sys.argv[1]
    serial_port = sys.argv[2]

    print(serial_port)

    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(root_dir, 'data')
    ros_filename = os.path.join(data_dir, filename + '_ros.csv')
    ser_filename = os.path.join(data_dir, filename + '_ser.csv')

    # print(ros_filename)
    # print(ser_filename)
    # exit()

    os.makedirs(data_dir, exist_ok=True)
    logger_node = ForceTorqueLogger(ros_filename)

    thread = threading.Thread(target=serial_logger_thread, args=(serial_port, ser_filename))
    thread.start()

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    # finally:
    #     logger_node.destroy_node()
    #     rclpy.shutdown()

    thread.join()

if __name__ == '__main__':
    main()
