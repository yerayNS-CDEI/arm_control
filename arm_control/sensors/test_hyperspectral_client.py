#!/usr/bin/env python3

##################################################
#### TEST CLIENT FOR HYPERSPECTRAL NODE
##################################################

import rclpy
from rclpy.node import Node
from navi_wall_interfaces.srv import HyperspectralCommand, HyperspectralConfig


class TestHyperspectralClient(Node):

    def __init__(self):
        super().__init__('test_hyperspectral_client')
        
        # Create service clients
        self.config_client = self.create_client(
            HyperspectralConfig, 
            'hyperspectral/configure'
        )
        self.measurement_client = self.create_client(
            HyperspectralCommand, 
            'hyperspectral/measurement'
        )
        
        # Wait for services
        self.get_logger().info("Waiting for services...")
        self.config_client.wait_for_service()
        self.measurement_client.wait_for_service()
        self.get_logger().info("Services available!")


    def configure_sensor(self, sensor, command, value):
        """Send configuration request."""
        req = HyperspectralConfig.Request()
        req.sensor = sensor
        req.command = command
        req.value = value
        
        self.get_logger().info(f"Configuring {sensor} {command}={value}...")
        future = self.config_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✓ {response.message}")
        else:
            self.get_logger().error(f"✗ {response.message}")
        
        return response.success


    def take_measurement(self, command):
        """Send measurement request."""
        req = HyperspectralCommand.Request()
        req.command = command
        
        self.get_logger().info(f"Requesting {command} measurement...")
        future = self.measurement_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        self.get_logger().info(f"Response: {response.message}")
        self.get_logger().info(f"  VIS: {'OK' if response.vis_ok else 'FAIL'}, "
                              f"{len(response.vis_spectrum)} points, status={response.vis_status}")
        self.get_logger().info(f"  NIR: {'OK' if response.nir_ok else 'FAIL'}, "
                              f"{len(response.nir_spectrum)} points, status={response.nir_status}")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    client = TestHyperspectralClient()
    
    try:
        # Example 1: Configure VIS sensor
        client.configure_sensor("VIS", "MTI", 65000)
        
        # Example 2: Configure NIR sensor
        client.configure_sensor("NIR", "MTR", 25000)
        
        # Example 3: Take GDS measurement
        input("\nPress ENTER to take GDS (Dark Sample) measurement...")
        client.take_measurement("GDS")
        
        # Example 4: Take GDR measurement
        input("\nPress ENTER to take GDR (Dark Reference) measurement...")
        client.take_measurement("GDR")
        
        # Example 5: Take GRF measurement
        input("\nPress ENTER to take GRF (White Reference) measurement...")
        client.take_measurement("GRF")
        
        # Example 6: Take GSM measurements
        count = 0
        while True:
            user_input = input("\nPress ENTER for GSM measurement (q to quit): ")
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
            count += 1
            client.take_measurement("GSM")
            client.get_logger().info(f"GSM measurement #{count} completed")
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
