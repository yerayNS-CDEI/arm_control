#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header, Float32MultiArray
import os
import serial
import re
import sys
import threading
from collections import deque
from statistics import median

# Stable name created by the udev rule in arm_control/udev/99-arduino-sensors.rules.
# Do not use a raw /dev/ttyACM* index: it shifts whenever the board re-enumerates.
DEFAULT_SERIAL_PORT = '/dev/arduino_sensors'
BY_ID_DIR = '/dev/serial/by-id'


class SerialPortUnavailable(RuntimeError):
    """Raised when no Arduino serial port could be opened."""


def list_by_id_ports():
    """Return every /dev/serial/by-id entry, sorted (empty if the dir is missing)."""
    try:
        return sorted(os.path.join(BY_ID_DIR, n) for n in os.listdir(BY_ID_DIR))
    except OSError:
        return []


def find_arduino_by_id():
    """Return the first /dev/serial/by-id entry that looks like an Arduino, or None."""
    for path in list_by_id_ports():
        if 'Arduino' in os.path.basename(path):
            return path
    return None


class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_node')

        # Declare parameters
        self.declare_parameter('autostart', False)
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        autostart = self.get_parameter('autostart').value
        serial_port = self.get_parameter('serial_port').value

        self.serial = self.open_serial(serial_port, 115200)

        # Publishers
        self.pub_s4 = self.create_publisher(Range, 'vl6180/sensor1', 10)
        self.pub_s5 = self.create_publisher(Range, 'vl6180/sensor2', 10)
        self.pub_s6 = self.create_publisher(Range, 'vl6180/sensor3', 10)
        self.pub_s1 = self.create_publisher(Range, 'hcsr04/sensor1', 10)
        self.pub_s2 = self.create_publisher(Range, 'hcsr04/sensor2', 10)
        self.pub_s3 = self.create_publisher(Range, 'hcsr04/sensor3', 10)
        self.pub_sensors = self.create_publisher(Float32MultiArray, 'distance_sensors', 10)

        # Timer
        self.timer = self.create_timer(0.2, self.read_serial)

        # Almacenamiento de últimos datos
        self.last_data = None
        self.publish_now = autostart
        self.publish_mode = 1 if autostart else 0   # Publication mode --> 0: single publication, 1: continuous publication
        self.batch_size = 1
        self.buffer_ultra = [deque(maxlen=self.batch_size) for _ in range(3)]  # U1, U2, U3
        self.buffer_vl = [deque(maxlen=self.batch_size) for _ in range(3)]     # S1, S2, S3
        self.calc_type = 0      # Computation type --> 0: median, 1: average

        # Hilo para leer teclado sin bloqueo
        threading.Thread(target=self.listen_for_key, daemon=True).start()

        self.get_logger().info(f"Multi-sensor node initialized (autostart={autostart}).")
        if autostart:
            self.get_logger().info("Autostart enabled: continuous publishing started")
        else:
            self.get_logger().info("Press 'c' to enable continuous publishing, 'p' for single publish")

    def open_serial(self, port, baud_rate):
        """Open `port`, falling back to the Arduino's by-id path if that name is absent."""
        candidates = [port]
        fallback = find_arduino_by_id()
        if fallback and fallback not in candidates:
            candidates.append(fallback)

        for candidate in candidates:
            try:
                connection = serial.Serial(candidate, baud_rate, timeout=1)
            except serial.SerialException as exc:
                self.get_logger().warn(f"Could not open '{candidate}': {exc}")
                continue

            if candidate != port:
                self.get_logger().warn(
                    f"'{port}' is not available, using '{candidate}' instead. "
                    "Install arm_control/udev/99-arduino-sensors.rules for a stable name.")
            else:
                self.get_logger().info(f"Opened serial port '{candidate}' at {baud_rate} baud.")
            return connection

        raise SerialPortUnavailable(self.serial_help_text(port))

    @staticmethod
    def serial_help_text(port):
        """Build an actionable message listing the serial ports that do exist."""
        available = list_by_id_ports()
        if available:
            listing = "\n".join(f"  {path}" for path in available)
            detail = f"Serial devices currently present:\n{listing}"
        else:
            detail = ("No serial devices are present under /dev/serial/by-id at all - "
                      "check that the Arduino is plugged in and powered.")
        return (
            f"Could not open the Arduino on '{port}'.\n"
            f"{detail}\n"
            "Fixes:\n"
            "  - install the udev rule once:\n"
            "      sudo cp <ws>/src/arm_control/udev/99-arduino-sensors.rules /etc/udev/rules.d/\n"
            "      sudo udevadm control --reload-rules && sudo udevadm trigger\n"
            "  - or point the node at a port explicitly:\n"
            "      ros2 run arm_control arduino_sensors --ros-args -p serial_port:=/dev/ttyACM1\n"
            "  - also confirm you are in the 'dialout' group (groups | grep dialout).")

    def listen_for_key(self):
        while rclpy.ok():
            inp = input("Press 'p' + Enter or 'c' + Enter to publish once or continuously: ").strip()
            if inp == 'p':
                self.publish_now = True
                self.publish_mode = 0
            if inp == 'c':
                self.publish_now = True
                self.publish_mode = 1                 

    def create_range_msg(self, frame_id, distance_m):
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0349
        msg.min_range = 0.01
        msg.max_range = 0.18
        msg.range = min(max(distance_m, msg.min_range), msg.max_range)
        return msg

    def read_serial(self):
        try:
            # Reading data from serial port
            line = self.serial.readline().decode('utf-8').strip()
            while self.serial.in_waiting > 0:   # obtaining always last recieved line (more accurate data)
                line = self.serial.readline().decode('utf-8').strip()
            self.get_logger().warn(line)

            # Extracting data from message
            match = re.match(r'U1:(\d+)cm U2:(\d+)cm U3:(\d+)cm S1:(\d+)mm S2:(\d+)mm S3:(\d+)mm', line)
            if match:
                du1, du2, du3, d1, d2, d3 = map(int, match.groups())
                self.last_data = (du1, du2, du3, d1, d2, d3)

                # Checking for invalid values
                valid_ultras = [dist for dist in (du1, du2, du3) if dist != 0 and dist < 400]
                valid_vls = [dist for dist in (d1, d2, d3) if dist != 255]
                # if len(valid_ultras) == 3 and len(valid_vls) == 3:
                #     # Save into buffers
                #     for i in range(3):
                #         self.buffer_ultra[i].append([du1, du2, du3][i])
                #         self.buffer_vl[i].append([d1, d2, d3][i])
                # else:
                #         self.get_logger().warn("Invalid sensor data received (255 or too high).")
                #         return
                
                for i in range(3):
                        self.buffer_ultra[i].append([du1, du2, du3][i])
                        self.buffer_vl[i].append([d1, d2, d3][i])
                
                # Publishing sensor values
                if self.publish_now:
                    if any(len(buf) < self.batch_size for buf in self.buffer_ultra + self.buffer_vl):
                        self.get_logger().warn("Not enough valid data in buffers to publish.")
                        if self.publish_mode == 0:
                            self.publish_now = False
                        return

                    # Presenting results in the terminal
                    sensor_ranges = []
                    for i in range(3):
                        if self.calc_type == 0:
                            avg = median(self.buffer_ultra[i]) / 100.0
                        elif self.calc_type == 1:
                            avg = sum(self.buffer_ultra[i]) / len(self.buffer_ultra[i])
                        else:
                            self.get_logger().error("Wrong computation type selected.")

                        self.get_logger().info(f"Buffer sensor hcsr04_{i+1}: {list(self.buffer_ultra[i])}")
                        self.get_logger().info(f"Value sensor hcsr04_{i+1}: {avg:.3f} m")
                        msg = self.create_range_msg(f"hcsr04_sensor{i+1}", avg)
                        getattr(self, f'pub_s{i+1}').publish(msg)
                        sensor_ranges.append(f"Sensor {i+1}: {msg.range:.3f} m")

                    for i in range(3):
                        if self.calc_type == 0:
                            avg = median(self.buffer_vl[i]) / 100.0
                        elif self.calc_type == 1:
                            avg = sum(self.buffer_vl[i]) / len(self.buffer_vl[i])
                        else:
                            self.get_logger().error("Wrong computation type selected.")
                        
                        self.get_logger().info(f"Buffer sensor vl6180_{i+1}: {list(self.buffer_vl[i])}")
                        self.get_logger().info(f"Value sensor vl6180_{i+1}: {avg:.3f} m")
                        msg = self.create_range_msg(f"vl6180_sensor{i+1}", avg)
                        getattr(self, f'pub_s{i+4}').publish(msg)
                        sensor_ranges.append(f"Sensor {i+4}: {msg.range:.3f} m")

                    log_msg = "\n".join(sensor_ranges)
                    self.get_logger().info(f"\n{log_msg}")

                    # Publishing data into topic
                    distances_array = Float32MultiArray()
                    if self.calc_type == 0:
                        distances_array.data = [median(self.buffer_ultra[i]) / 100.0 for i in range(3)] + [(median(self.buffer_vl[i]) / 1000.0)+0.083 for i in range(3)]
                    elif self.calc_type == 1:
                        distances_array.data = [sum(self.buffer_ultra[i]) / len(self.buffer_ultra[i]) / 100.0 for i in range(3)] + [(sum(self.buffer_vl[i]) / len(self.buffer_vl[i]) / 1000.0)+0.083 for i in range(3)]
                    else:
                        self.get_logger().error("Wrong computation type selected.")
                    
                    self.pub_sensors.publish(distances_array)
                    self.get_logger().info(f"Float32MultiArray: {['%.3f' % v for v in distances_array.data]}")

                    if self.publish_mode == 0:
                        self.publish_now = False  # Reset flag after publishing

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MultiSensorNode()
    except SerialPortUnavailable as exc:
        rclpy.logging.get_logger('multi_sensor_node').error(str(exc))
        rclpy.shutdown()
        sys.exit(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
