#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header, Float32MultiArray
import math
import threading
from collections import deque
from statistics import median
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SimulatedSensorNode(Node):
    def __init__(self):
        super().__init__('simulated_sensor_node')
        
        # Declare parameters
        self.declare_parameter('batch_size', 5)
        self.declare_parameter('calc_type', 0)  # 0: median, 1: average
        self.declare_parameter('publish_rate', 0.5)  # Hz
        
        # Get parameters
        self.batch_size = self.get_parameter('batch_size').value
        self.calc_type = self.get_parameter('calc_type').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # QoS for Gazebo sensors (RELIABLE as confirmed)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Subscribe to simulated ultrasonic sensors (A, B, C)
        self.create_subscription(
            Range, '/distance_sensors/A', 
            lambda msg: self.sensor_callback(msg, 1), sensor_qos)
        self.create_subscription(
            Range, '/distance_sensors/B', 
            lambda msg: self.sensor_callback(msg, 2), sensor_qos)
        self.create_subscription(
            Range, '/distance_sensors/C', 
            lambda msg: self.sensor_callback(msg, 0), sensor_qos)
        
          # Publishers (matching Arduino node)
        self.pub_s1 = self.create_publisher(Range, 'hcsr04/sensor1', 10)
        self.pub_s2 = self.create_publisher(Range, 'hcsr04/sensor2', 10)
        self.pub_s3 = self.create_publisher(Range, 'hcsr04/sensor3', 10)
        
        # ToF sensor publishers (hardcoded values for now)
        self.pub_s4 = self.create_publisher(Range, 'vl6180/sensor1', 10)
        self.pub_s5 = self.create_publisher(Range, 'vl6180/sensor2', 10)
        self.pub_s6 = self.create_publisher(Range, 'vl6180/sensor3', 10)
        
        # Combined array publisher
        self.pub_sensors = self.create_publisher(Float32MultiArray, 'distance_sensors', 10)
        
        # Buffers for smoothing (A, B, C)
        self.buffer_ultra = [deque(maxlen=self.batch_size) for _ in range(3)]
        self.buffer_vl = [deque(maxlen=self.batch_size) for _ in range(3)]
        
        # Publication control
        self.publish_now = False
        self.publish_mode = 1  # Start in continuous mode for simulation
        
        # Timer for publishing at fixed rate
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_sensor_data)
        
        # Keyboard listener thread
        threading.Thread(target=self.listen_for_key, daemon=True).start()
        
        # Hardcoded ToF values (not used in scanning)
        self.tof_hardcoded = [0.10, 0.10, 0.10]  # 10cm default
        for i in range(3):
            self.buffer_vl[i].append(self.tof_hardcoded[i] * 1000.0)  # Convert to mm
        
        self.get_logger().info(f"Simulated sensor node initialized (batch_size={self.batch_size}, calc_type={self.calc_type})")
        self.get_logger().info("ToF sensors hardcoded to 0.10m (not used in scanning)")
        self.get_logger().info("Press 'c' to enable continuous publishing, 'p' for single publish")
    
    def listen_for_key(self):
        """Keyboard input thread for manual publish control"""
        while rclpy.ok():
            try:
                inp = input().strip()
                if inp == 'p':
                    self.publish_now = True
                    self.publish_mode = 0
                    self.get_logger().info("Single publish triggered")
                elif inp == 'c':
                    self.publish_now = True
                    self.publish_mode = 1
                    self.get_logger().info("Continuous publish enabled")
            except:
                pass
    
    def sensor_callback(self, msg, sensor_idx):
        """Store incoming Range messages from Gazebo sensors A, B, C"""
        try:
            # Validate range
            if not math.isfinite(msg.range):
                self.get_logger().warn(f'Sensor {sensor_idx}: Invalid range (inf/nan), skipping')
                return
            
            if msg.range < msg.min_range or msg.range > msg.max_range:
                # Out of range - use max as fallback
                self.buffer_ultra[sensor_idx].append(msg.max_range * 100.0) 
                self.get_logger().debug(f'Sensor {sensor_idx}: Out of range, using max={msg.max_range}')
            else:
                # Valid range - convert to cm for buffer (like Arduino)
                self.buffer_ultra[sensor_idx].append(msg.range * 100.0)
                
        except Exception as e:
            self.get_logger().error(f'Sensor {sensor_idx} callback error: {e}')
    
    def create_range_msg_ultrasonic(self, frame_id, distance_m):
        """Create Range message for ultrasonic sensors"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.174  # ~10 degrees (matches Gazebo config)
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = min(max(distance_m, msg.min_range), msg.max_range)
        return msg
    
    def create_range_msg_tof(self, frame_id, distance_m):
        """Create Range message for ToF sensors (VL6180x)"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0349  # ~2 degrees
        msg.min_range = 0.01
        msg.max_range = 0.18
        msg.range = min(max(distance_m, msg.min_range), msg.max_range)
        return msg
    
    def publish_sensor_data(self):
        """Publish sensor data at fixed rate"""
        if not self.publish_now:
            return
        
        # Check if enough data in buffers
        if any(len(buf) < 1 for buf in self.buffer_ultra):
            self.get_logger().warn("Waiting for sensor data...", throttle_duration_sec=2.0)
            return
        
        sensor_ranges = []
        
        # Publish ultrasonic sensors (A=1, B=2, C=3)
        for i in range(3):
            if self.calc_type == 0:
                avg_cm = median(self.buffer_ultra[i])
            elif self.calc_type == 1:
                avg_cm = sum(self.buffer_ultra[i]) / len(self.buffer_ultra[i])
            else:
                self.get_logger().error("Invalid calc_type")
                return
            
            avg_m = avg_cm / 100.0
            msg = self.create_range_msg_ultrasonic(f"hcsr04_sensor{i+1}", avg_m)
            getattr(self, f'pub_s{i+1}').publish(msg)
            sensor_ranges.append(f"HCSR04-{i+1}: {msg.range:.3f}m")
        
        # Publish ToF sensors (hardcoded)
        for i in range(3):
            if self.calc_type == 0:
                avg_mm = median(self.buffer_vl[i])
            else:
                avg_mm = sum(self.buffer_vl[i]) / len(self.buffer_vl[i])
            
            avg_m = avg_mm / 1000.0
            msg = self.create_range_msg_tof(f"vl6180_sensor{i+1}", avg_m)
            getattr(self, f'pub_s{i+4}').publish(msg)
            sensor_ranges.append(f"VL6180-{i+1}: {msg.range:.3f}m (hardcoded)")
        
        # Publish combined array (matches Arduino format)
        distances_array = Float32MultiArray()
        if self.calc_type == 0:
            ultra_values = [median(self.buffer_ultra[i]) / 100.0 for i in range(3)]
            vl_values = [median(self.buffer_vl[i]) / 1000.0 for i in range(3)]
        else:
            ultra_values = [sum(self.buffer_ultra[i]) / len(self.buffer_ultra[i]) / 100.0 for i in range(3)]
            vl_values = [sum(self.buffer_vl[i]) / len(self.buffer_vl[i]) / 1000.0 for i in range(3)]
        
        distances_array.data = ultra_values + vl_values
        self.pub_sensors.publish(distances_array)
        
        # Log summary
        log_msg = "\n".join(sensor_ranges)
        self.get_logger().info(f"\n{log_msg}\nArray: {['%.3f' % v for v in distances_array.data]}")
        
        if self.publish_mode == 0:
            self.publish_now = False  # Reset after single publish

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
